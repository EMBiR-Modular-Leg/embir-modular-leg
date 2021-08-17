#include <sys/mman.h>

#include <vector>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <cstdio>
#include <ctime>
#include <csignal>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "actuator.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "libFilter/filters.h"
#include "iir/iir.h"
#include "nlohmann/json.hpp"
#include "cxxopts/cxxopts.hpp"

#define PI 3.1415926

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;
namespace chron = std::chrono;

char cstr_buffer[128];

volatile sig_atomic_t interrupted=false; 

void sig_handle(int s) {
  interrupted = true;
}

struct DemoSettings {
  float period_s;
  float duration_s;
  float gear1;
  float gear2;

  uint8_t actuator_1_id;
  uint8_t actuator_1_bus;
  uint8_t actuator_2_id;
  uint8_t actuator_2_bus;

  uint8_t main_cpu;
  uint8_t can_cpu;

  cxxopts::ParseResult demo_opts;

  uint32_t status_period_us;

  float replay_vel_scale;
  float replay_trq_scale;
};

DemoSettings parse_settings(cxxopts::ParseResult demo_opts) {
  DemoSettings demoset;
  demoset.demo_opts = demo_opts;
  demoset.period_s = 1.0/demo_opts["frequency"].as<float>();
  demoset.duration_s = demo_opts["duration"].as<float>();
  demoset.gear1 = demo_opts["gear1"].as<float>();
  demoset.gear2 = demo_opts["gear2"].as<float>();
  demoset.actuator_1_id = demo_opts["actuator-1-id"].as<uint8_t>();
  demoset.actuator_2_id = demo_opts["actuator-2-id"].as<uint8_t>();
  demoset.actuator_1_bus = demo_opts["actuator-1-bus"].as<uint8_t>();
  demoset.actuator_2_bus = demo_opts["actuator-2-bus"].as<uint8_t>();

  demoset.main_cpu = demo_opts["main-cpu"].as<uint8_t>();
  demoset.can_cpu = demo_opts["can-cpu"].as<uint8_t>();

  demoset.replay_vel_scale = demo_opts["replay-vel-scale"].as<float>();
  demoset.replay_trq_scale = demo_opts["replay-trq-scale"].as<float>();
  return demoset;
}

namespace Color {
  enum Code {
    FG_BLACK    = 30,
    FG_RED      = 31,
    FG_GREEN    = 32,
    FG_YELLOW   = 33,
    FG_BLUE     = 34,
    FG_DEFAULT  = 39,
    BG_RED      = 41,
    BG_GREEN    = 42,
    BG_YELLOW   = 43,
    BG_BLUE     = 44,
    BG_WHITE    = 47,
    BG_DEFAULT  = 49
  };
  class Modifier {
    Code code;
  public:
    Modifier(Code pCode) : code(pCode) {}
    friend std::ostream&
    operator<<(std::ostream& os, const Modifier& mod) {
      return os << "\033[" << mod.code << "m";
    }
    Modifier& operator=(const Modifier& mod) {
      this->code = mod.code;
      return *this;
    }
  };
}

namespace CMod {
  Color::Modifier fg_blk(Color::FG_BLACK);
  Color::Modifier fg_red(Color::FG_RED);
  Color::Modifier fg_grn(Color::FG_GREEN);
  Color::Modifier fg_blu(Color::FG_BLUE);
  Color::Modifier fg_yel(Color::FG_YELLOW);
  Color::Modifier fg_def(Color::FG_DEFAULT);

  Color::Modifier bg_red(Color::BG_RED);
  Color::Modifier bg_grn(Color::BG_GREEN);
  Color::Modifier bg_blu(Color::BG_BLUE);
  Color::Modifier bg_yel(Color::BG_YELLOW);
  Color::Modifier bg_wht(Color::BG_WHITE);
  Color::Modifier bg_def(Color::BG_DEFAULT);
}

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

void ConfigureRealtime(const uint8_t realtime) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(realtime, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << (int)realtime << "\n";
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    if (r < 0) {
      throw std::runtime_error("Error setting realtime scheduler");
    }
  }

  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

cxxopts::Options demo_opts() {
  cxxopts::Options options("dynamometer", "Run dual actuator dynamometer for characterization");

  options.add_options()
    ("c,comment", "enter comment string to be included in output csv.",         cxxopts::value<std::string>())
    ("p,path", "path to output csv.", cxxopts::value<std::string>()->default_value("/home/pi/embir-modular-leg/dynamometer-data/"))
    ("replay-file", "path to csv of torque, velocity data to replay.", cxxopts::value<std::string>()->default_value(""))
    ("replay-vel-scale", "scale velocity from replay data", cxxopts::value<float>()->default_value("1.0"))
    ("replay-trq-scale", "scale torque from replay data", cxxopts::value<float>()->default_value("1.0"))
    ("gear1", "gear ratio of actuator 1, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("gear2", "gear ratio of actuator 2, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("actuator-1-id", "actuator 1 CAN ID", cxxopts::value<uint8_t>()->default_value("1"))
    ("actuator-2-id", "actuator 2 CAN ID", cxxopts::value<uint8_t>()->default_value("2"))
    ("actuator-1-bus", "actuator 1 CAN bus", cxxopts::value<uint8_t>()->default_value("3"))
    ("actuator-2-bus", "actuator 2 CAN bus", cxxopts::value<uint8_t>()->default_value("4"))
    ("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
    ("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
    ("duration", "test duration in seconds", cxxopts::value<float>())
    ("frequency", "test sampling and command frequency in Hz", cxxopts::value<float>()->default_value("250"))
    ("skip-cal", "skip recalibration")
    ("swap-actuators", "start with actuator roles swapped (driving vs. load)")
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}

void Run(std::ofstream& data_file, DemoSettings& demoset, Actuator& a1,         Actuator& a2) {

  // * SETUP *
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handle;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
  moteus::ConfigureRealtime(demoset.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = demoset.can_cpu;
  moteus_options.servo_bus_map = {
    { demoset.actuator_1_id, demoset.actuator_1_bus },
    { demoset.actuator_2_id, demoset.actuator_2_bus },
  };
  MoteusInterface moteus_interface{moteus_options};

  // ** CONTAINER FOR COMMANDS **
  std::vector<MoteusInterface::ServoCommand> commands;
  std::vector<MoteusInterface::ServoCommand> saved_commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    commands.push_back({});
    commands.back().id = pair.first;
    saved_commands.push_back({});
    saved_commands.back().id = pair.first;
  }

  // ** CONTAINER FOR REPLIES **
  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  // ** INITIALIZE COMMANDS **
  commands[0] = a1.get_curr_cmd();
  commands[1] = a2.get_curr_cmd();

  // ** PACKAGE COMMANDS AND REPLIES IN moteus_data **
  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;

  // ** TEST PERIOD **
  const auto period =
      chron::microseconds(static_cast<int64_t>(demoset.period_s * 1e6));
  auto next_cycle = chron::steady_clock::now() + period;

  // ** TERMINAL STATUS UPDATE PERIOD **
  const auto status_period = chron::microseconds(demoset.status_period_us);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  uint64_t total_skip_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;
  uint64_t reply_miss_count = 0;

  std::string c1_str, c2_str, a1_str, a2_str, sensor_str;

  // dynamometer->set_t0(chron::steady_clock::now());
  auto t0 = chron::steady_clock::now();

  data_file << "time [s],";
  data_file << a1.stringify_actuator_header() << ","
    << a1.stringify_moteus_reply_header() << ","
    << a2.stringify_actuator_header() << ","
    << a2.stringify_moteus_reply_header() << "\n";

  float t_prog_s = 0;

  // * MAIN LOOP *
  while (!interrupted && t_prog_s < demoset.duration_s) {
    cycle_count++; margin_cycles++;
    // Terminal status update
    {
      const auto now = chron::steady_clock::now();
      if (now > next_status) {
        // Insert code here that you want to run at the status frequency
        next_status += status_period;
        total_margin = 0; margin_cycles = 0;

        // dynamometer->print_status_update();
        std::cout << "here " << cycle_count << " " << t_prog_s << std::endl;
      }

      int skip_count = 0;
      while (now > next_cycle) skip_count++; next_cycle += period;

      total_skip_count += skip_count;
      if (skip_count)
        std::cout << "Skipped " << total_skip_count << "/" << cycle_count <<" cycles in total" << std::endl;

      if (skip_count > 50) {
        std::cout << "too many skipped cycles, exiting..." << std::endl;
        data_file.close();
        std::exit(EXIT_FAILURE);
      }
    }
    
    // Sleep current thread until next control interval, per the period setting.
    {
      const auto pre_sleep = chron::steady_clock::now();
      std::this_thread::sleep_until(next_cycle);
      const auto post_sleep = chron::steady_clock::now();
      chron::duration<double> elapsed = post_sleep - pre_sleep;
      total_margin += elapsed.count();
    }
    next_cycle += period;

    // **** MANIPULATE COMMANDS HERE FOR OPERATION ****
    // dynamometer->Run(saved_replies, &commands);
    // a1.make_stop();
    // a2.make_stop();

    // a1.make_act_velocity(1.0*std::sin(t_prog_s), 0);
    a1.make_act_position(1.0*std::sin(t_prog_s), 0);
    // a2.make_act_velocity(1.0*std::sin(t_prog_s), 0);
    a2.make_act_position(1.0*std::sin(t_prog_s), 0);

    // TODO: make FSM

    // commands vector is memory linked to moteus_data
    commands[0] = a1.get_curr_cmd();
    commands[1] = a2.get_curr_cmd();

    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new one.
      const auto current_values = can_result.get();
      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      saved_replies.resize(rx_count);
      std::copy(replies.begin(), replies.begin() + rx_count, 
        saved_replies.begin());
    }
    a1.retrieve_reply(saved_replies);
    a2.retrieve_reply(saved_replies);

    // Then we can immediately ask them to be used again.
    auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
    // Cycle out commands to drivers
    moteus_interface.Cycle(
        moteus_data,
        [promise](const MoteusInterface::Output& output) {
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();

    if (cycle_count > 5 && saved_replies.size() >= 2) {
      reply_miss_count = 0;

      auto time_span = chron::steady_clock::now() - t0;
      t_prog_s = double(time_span.count()) * chron::steady_clock::period::num / 
            chron::steady_clock::period::den;

      data_file << std::setw(10) << std::setprecision(4) << std::fixed
        << t_prog_s << ",";
        c1_str = a1.stringify_moteus_reply();
        c2_str = a2.stringify_moteus_reply();
        a1_str = a1.stringify_actuator();
        a2_str = a2.stringify_actuator();
        data_file << a1_str << "," << c1_str << "," << a2_str << "," << c2_str;
      data_file << "\n";
      data_file.flush();
    }
    else if (cycle_count > 5 && saved_replies.size() < 2)
      reply_miss_count++;

    // kill loop if we miss all these replies
    if (reply_miss_count > 20) {
      break; std::cout << "missed too many replies! ending..." << std::endl;
    }

    if (cycle_count > 1) std::copy(commands.begin(), commands.end(), saved_commands.begin());
    // dynamometer->safety_check(saved_replies);
  }
  // IF INTERRUPTED
  std::cout << std::endl << "exiting..." << std::endl;

  // while (!can_result.valid()) {
  //   // do nothing
  // }
  // for (auto& cmd : moteus_data.commands) cmd.mode = moteus::Mode::kStopped;
  // moteus_interface.Cycle(moteus_data, nullptr);
  data_file.close();
  std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
  auto options = demo_opts();
  auto opts = options.parse(argc, argv);
  std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

  chron::time_point<chron::system_clock> now = chron::system_clock::now();
  std::time_t nowc = chron::system_clock::to_time_t(now);
  std::ostringstream filename_stream;
  filename_stream << std::put_time(std::localtime(&nowc), "cpp_api_test_%d_%m_%Y_%H-%M-%S.csv");
  std::string filename = filename_stream.str();
  std::cout << "outputting to file \"" << filename << "\"" << std::endl;

  // std::ofstream data_file("/home/pi/embir-modular-leg/dynamometer-data/"+filename);
  std::ofstream data_file(opts["path"].as<std::string>()+filename);

  std::vector<std::string> arg_list(argv, argv+argc);
  data_file << "# ";
  for (auto arg_str : arg_list) data_file << arg_str << " ";
  data_file << std::endl;
  // return 0;

  Adafruit_ADS1015 ads;
  Adafruit_INA260 ina1;
  Adafruit_INA260 ina2;
  if (!bcm2835_init()) {
    std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
    return 1;
  }
  bcm2835_i2c_begin();

  LockMemory();

  auto demoset = parse_settings(opts);
  demoset.status_period_us = static_cast<int64_t>(
    (1e6)/10);
  data_file << "# \n# user comment: " << opts["comment"]
    .as<std::string>() << "\n# \n";
  data_file << "# period s: " << 1.0/opts["frequency"].as<float>() << "\n";
  data_file << "# duration s: " << opts["duration"].as<float>() << "\n";
  data_file << "# gear 1: " << opts["gear1"].as<float>() << "\n";
  data_file << "# gear 2: " << opts["gear2"].as<float>() << "\n";
  data_file << "# actuator 1 id: " << (int)(opts["actuator-1-id"]
    .as<uint8_t>()) << "\n";
  data_file << "# actuator 2 id: " << (int)(opts["actuator-2-id"]
    .as<uint8_t>()) << "\n";
  data_file << "# actuator 1 bus: " << (int)(opts["actuator-1-bus"]
    .as<uint8_t>()) << "\n";
  data_file << "# actuator 2 bus: " << (int)(opts["actuator-2-bus"]
    .as<uint8_t>()) << "\n";
  data_file << "# main cpu: " << (int)(opts["main-cpu"].as<uint8_t>()) << "\n";
  data_file << "# can cpu: " << (int)(opts["can-cpu"]
    .as<uint8_t>()) << "\n# \n";

  ConfigureRealtime(demoset.main_cpu);
  ConfigureRealtime(demoset.can_cpu);

  Actuator a1(demoset.actuator_1_id, demoset.actuator_1_bus,
    demoset.gear1, 1.0);
  Actuator a2(demoset.actuator_2_id, demoset.actuator_2_bus,
    demoset.gear2, 1.0);

  a1.zero_offset();
  a2.zero_offset();

  // return 0;
  Run(data_file, demoset, a1, a2);

  data_file.close();
  return 0;
}
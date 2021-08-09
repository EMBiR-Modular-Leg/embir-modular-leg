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

#include "leg.h"
#include "utils.h"

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

void Run(Leg& leg) {

  // * SETUP *
  // ** CTRL+C CATCH **
	Leg::LegSettings& legset = leg.get_legset();
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handle;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
  std::cout << "configuring realtime and constructing moteus interface... ";
  moteus::ConfigureRealtime(legset.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = legset.can_cpu;
  moteus_options.servo_bus_map = {
    { legset.act_femur_id, legset.act_femur_bus },
    { legset.act_tibia_id, legset.act_tibia_bus },
  };
  MoteusInterface moteus_interface{moteus_options};
  std::cout << "done.\n" << std::flush;

  // ** CONTAINER FOR COMMANDS **
  std::vector<MoteusInterface::ServoCommand> curr_commands;
  std::vector<MoteusInterface::ServoCommand> prev_commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    curr_commands.push_back({});
    curr_commands.back().id = pair.first;
    prev_commands.push_back({});
    prev_commands.back().id = pair.first;
  }
  // distributes pointers to this data into the MoteusController objects and
  // initializes the commands (resolution settings etc.)
  // leg.share_commands(curr_commands, prev_commands);

  // ** CONTAINER FOR REPLIES **
  std::vector<MoteusInterface::ServoReply> replies{curr_commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies{curr_commands.size()};

  // ** PACKAGE COMMANDS AND REPLIES IN moteus_data **
  std::cout << "setting up moteus_data container... ";
  MoteusInterface::Data moteus_data;
  moteus_data.commands = { curr_commands.data(), curr_commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };
  std::cout << "done.\n" << std::flush;

  std::future<MoteusInterface::Output> can_result;

  // ** TIMING **
  const auto period =
      chron::microseconds(static_cast<int64_t>(legset.period_s * 1e6));
  auto next_cycle = chron::steady_clock::now() + period;

  const auto status_period = chron::microseconds(legset.status_period_us);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  uint64_t total_skip_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;
  uint64_t reply_miss_count = 0;

  std::string c1_str, c2_str, a1_str, a2_str, sensor_str;

  auto t0 = chron::steady_clock::now();
  leg.set_time0(t0);

  leg.log_headers();

  // * MAIN LOOP *
  std::cout << "beginning while loop..." << std::endl;
  while (!interrupted && leg.get_time_prog() < legset.duration_s) {
    cycle_count++; margin_cycles++;
    // Terminal status update
    {
      const auto now = chron::steady_clock::now();
      if (now > next_status) {
        // Insert code here that you want to run at the status frequency
        next_status += status_period;
        total_margin = 0; margin_cycles = 0;

        leg.print_status_update();
        // std::cout << cycle_count << " " << leg.get_time_prog() << std::endl;
      }

      int skip_count = 0;
      while (now > next_cycle) {skip_count++; next_cycle += period;}

      total_skip_count += skip_count;
      if (skip_count)
        std::cout << "Skipped " << total_skip_count << "/" << cycle_count 
          <<" cycles in total" << std::endl;

      if (skip_count > 50) {
        std::cout << "too many skipped cycles, exiting..." << std::endl;
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
    // fsm will create commands stored in the leg member actuators
    // leg.print_status_update();
    leg.iterate_fsm();
    // retrieve the commands (copy)
    curr_commands[0] = leg.get_femur_cmd();
    curr_commands[1] = leg.get_tibia_cmd();

    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new one.
      const auto current_values = can_result.get();
      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      // saved_replies.resize(rx_count);
  	  // std::cout << "cycle_count: " << cycle_count
      //   << " replies.size(): " << replies.size() 
      //   << " saved_replies.size(): " << saved_replies.size() 
      //   << " rx_count: " << rx_count << std::endl;
      // std::copy(replies.begin(), replies.begin() + rx_count, 
      //   saved_replies.begin());
      for (size_t ii = 0; ii < replies.size(); ii++)  {
        saved_replies[ii] = replies[ii];
      }
    }
  	if(replies.size() < 2) std::cout << "main: incorrect number of replies: " << replies.size() << std::endl;

    // copy the replies over to the member actuators; they look for ID match. If
    // there's no matching ID response, fault is raised
    leg.retrieve_replies(saved_replies);

    // Then we can immediately ask them to be used again.
    auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
    // Cycle out curr_commands to drivers
    moteus_interface.Cycle(
        moteus_data,
        [promise](const MoteusInterface::Output& output) {
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();

    if (cycle_count > 5 && saved_replies.size() >= 2) reply_miss_count = 0;
    else if (cycle_count > 5 && saved_replies.size() < 2) reply_miss_count++;

    // kill loop if we miss all these replies
    if (reply_miss_count > 20) {
      std::cout << "missed too many replies in a row! ending..." << std::endl;
      break;
    }

    if (cycle_count > 1) {
      // std::copy(curr_commands.begin(), curr_commands.end(), prev_commands.begin());
      for (size_t ii = 0; ii < curr_commands.size(); ii++)  {
        prev_commands[ii] = curr_commands[ii];
      }
    }
    // dynamometer->safety_check(saved_replies);
  }
  // IF INTERRUPTED
  std::cout << std::endl << "exiting..." << std::endl;

  std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
  auto options = leg_opts();
  auto opts = options.parse(argc, argv);
  std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

  chron::time_point<chron::system_clock> now = chron::system_clock::now();
  std::time_t nowc = chron::system_clock::to_time_t(now);
  std::ostringstream filename_stream;
  filename_stream << std::put_time(std::localtime(&nowc), "leg_test_%d_%m_%Y_%H-%M-%S.csv");
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

  Leg::LegSettings legset(opts);
  legset.status_period_us = static_cast<int64_t>(
    (1e6)/10);
  data_file << "# \n# user comment: " << opts["comment"]
    .as<std::string>() << "\n# \n";
  data_file << "# period s: " << 1.0/opts["frequency"].as<float>() << "\n";
  data_file << "# duration s: " << opts["duration"].as<float>() << "\n";
  data_file << "# gear femur: " << opts["gear-femur"].as<float>() << "\n";
  data_file << "# gear tibia: " << opts["gear-tibia"].as<float>() << "\n";
  data_file << "# femur actuator id: " << (int)(opts["act-femur-id"]
    .as<uint8_t>()) << "\n";
  data_file << "# tibia actuator id: " << (int)(opts["act-tibia-id"]
    .as<uint8_t>()) << "\n";
  data_file << "# femur actuator bus: " << (int)(opts["act-femur-bus"]
    .as<uint8_t>()) << "\n";
  data_file << "# tibia actuator bus: " << (int)(opts["act-tibia-bus"]
    .as<uint8_t>()) << "\n";
  data_file << "# main cpu: " << (int)(opts["main-cpu"].as<uint8_t>()) << "\n";
  data_file << "# can cpu: " << (int)(opts["can-cpu"]
    .as<uint8_t>()) << "\n# \n";

  ConfigureRealtime(legset.main_cpu);
  ConfigureRealtime(legset.can_cpu);

	Leg leg(
    legset,
    data_file,
    "/home/pi/embir-modular-leg/urdf/modleg.urdf");

  // return 0;
  Run(leg);
  std::cout << "(returned to main)" << std::endl;
  data_file.close();
  std::exit(EXIT_SUCCESS);
}
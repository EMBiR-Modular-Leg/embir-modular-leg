#ifndef LEG_H
#define LEG_H

#include <ostream>
#include <vector>
#include <functional>

#include "actuator.h"

#include "cxxopts/cxxopts.hpp"

class Leg {
public:

	enum FSMState : uint8_t {
		kIdle,
		kRunning,
		kRecovery,
		kQuitting,
	};

	struct LegSettings {
		float period_s;
		float duration_s;
		float gear1;
		float gear2;

		uint8_t act_femur_id;
		uint8_t act_femur_bus;
		uint8_t act_tibia_id;
		uint8_t act_tibia_bus;

		uint8_t main_cpu;
		uint8_t can_cpu;

		cxxopts::ParseResult leg_opts;

		uint32_t status_period_us;

		float replay_vel_scale;
		float replay_trq_scale;
	};

	Leg(Actuator& act_femur, Actuator& act_tibia, std::ostream& datastream,
		LegSettings& legset);

	LegSettings parse_settings(cxxopts::ParseResult leg_opts);

	void iterate_fsm();

	void log_data();

	inline FSMState get_fsm_curr_state() {return curr_state_;}
	inline FSMState get_fsm_next_state() {return next_state_;}
	inline bool ready_to_quit() {return ready_to_quit;}
	inline LegSettings& get_legset() {return legset_;}

	inline void share_commands(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& curr_commands,
		std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& prev_commands) {
		act_femur_.share_curr_cmd(curr_commands[0]);
		act_tibia_.share_curr_cmd(curr_commands[1]);
		act_femur_.share_prev_cmd(prev_commands[0]);
		act_tibia_.share_prev_cmd(prev_commands[1]);
	}

	inline void retrieve_replies(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& prev_replies) {
		act_femur_.retrieve_reply(prev_replies);
		act_tibia_.retrieve_reply(prev_replies);
	}
private:
	Actuator& act_femur_;
	Actuator& act_tibia_;
	std::ostream& datastream_;

	// These are vectors of references to ServoCommand's that basically act like
	// vectors of ServoCommand's. This means that the command data is only stored
	// in the MoteusController objects and everyone else just has references to
	// that data.
	std::vector<
		std::reference_wrapper<
			mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>> commands_;
	std::vector<
		std::reference_wrapper<
			mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>> prev_commands_;

	LegSettings legset_;

	FSMState curr_state_ = FSMState::kIdle;
	FSMState next_state_ = FSMState::kIdle;

  std::chrono::steady_clock::time_point time0_s_;
	float time_prog_s_ = 0;
	float time_prog_old_s_ = 0;
	float time_fcn_s_ = 0;
	size_t cycles_ = 0;

	uint8_t quit_cycle = 0;
	uint8_t quit_cycle_thresh = 3;
	bool ready_to_quit = false;

	uint8_t recovery_cycle = 0;
	uint8_t recovery_cycle_thresh = 3;
};

cxxopts::Options leg_opts() {
  cxxopts::Options options(
		"leg", "Run 2D leg");

  options.add_options()
    ("c,comment", "enter comment string to be included in output csv.",         
			cxxopts::value<std::string>())
    ("p,path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/dynamometer-data/"))
    ("replay-file", "path to csv of torque, velocity data to replay.", 
			cxxopts::value<std::string>()->default_value(""))
    ("replay-vel-scale", "scale velocity from replay data", 
			cxxopts::value<float>()->default_value("1.0"))
    ("replay-trq-scale", "scale torque from replay data",
			cxxopts::value<float>()->default_value("1.0"))
    ("gear-femur", "gear ratio of femur actuator, as a reduction", 
			cxxopts::value<float>()->default_value("1.0"))
    ("gear-tibia", "gear ratio of tibia actuator, as a reduction",	
			cxxopts::value<float>()->default_value("1.0"))
    ("act-femur-id", "femur actuator CAN ID",
			cxxopts::value<uint8_t>()->default_value("1"))
		("act_tibia-id", "tibia actuator CAN ID",
			cxxopts::value<uint8_t>()->default_value("2"))
    ("act-femur-bus", "femur actuator CAN bus",
			cxxopts::value<uint8_t>()->default_value("3"))
    ("act_tibia-bus", "tibia actuator CAN bus",
			cxxopts::value<uint8_t>()->default_value("4"))
    ("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
    ("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
    ("duration", "test duration in seconds", cxxopts::value<float>())
    ("frequency", "test sampling and command frequency in Hz", 
			cxxopts::value<float>()->default_value("250"))
    ("skip-cal", "skip recalibration")
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}

Leg::LegSettings parse_settings(cxxopts::ParseResult leg_opts) {
  Leg::LegSettings legset;
  legset.leg_opts = leg_opts;
  legset.period_s = 1.0/leg_opts["frequency"].as<float>();
  legset.duration_s = leg_opts["duration"].as<float>();
  legset.gear1 = leg_opts["gear1"].as<float>();
  legset.gear2 = leg_opts["gear2"].as<float>();
  legset.act_femur_id = leg_opts["act-femur-id"].as<uint8_t>();
  legset.act_tibia_id = leg_opts["act-tibia-id"].as<uint8_t>();
  legset.act_femur_bus = leg_opts["act-femur-bus"].as<uint8_t>();
  legset.act_tibia_bus = leg_opts["act-tibia-bus"].as<uint8_t>();

  legset.main_cpu = leg_opts["main-cpu"].as<uint8_t>();
  legset.can_cpu = leg_opts["can-cpu"].as<uint8_t>();

  legset.replay_vel_scale = leg_opts["replay-vel-scale"].as<float>();
  legset.replay_trq_scale = leg_opts["replay-trq-scale"].as<float>();
  return legset;
}

#endif
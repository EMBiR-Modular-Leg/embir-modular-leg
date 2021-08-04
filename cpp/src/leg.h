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

	Leg(LegSettings& legset, std::ostream& datastream);

	void iterate_fsm();

	void log_data();

	void print_status_update();

	inline FSMState get_fsm_curr_state() {return curr_state_;}
	inline FSMState get_fsm_next_state() {return next_state_;}
	inline bool is_ready_to_quit() {return ready_to_quit;}
	inline bool leg_fault() {return ((bool)act_femur_.fault() || (bool)act_tibia_.fault());}
	inline LegSettings& get_legset() {return legset_;}

	inline void set_time0(std::chrono::steady_clock::time_point t0) {time0_s_ = t0;}

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
	Actuator act_femur_;
	Actuator act_tibia_;
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

cxxopts::Options leg_opts();
Leg::LegSettings parse_settings(cxxopts::ParseResult leg_opts);

#endif
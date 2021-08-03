#include <iomanip>

#include "leg.h"

namespace chron = std::chrono;

Leg::Leg(Actuator& act_femur, Actuator& act_tibia, std::ostream& datastream,
	LegSettings& legset) : 
	act_femur_(act_femur), act_tibia_(act_tibia_), datastream_(datastream),
	legset_(legset), commands_() {
	
	act_femur_.zero_offset();
  act_tibia_.zero_offset();

	commands_.resize(2);
	commands_[0] = (act_femur_.get_curr_cmd());
	commands_[1] = (act_tibia_.get_curr_cmd());

	prev_commands_.resize(2);
	prev_commands_[0] = (act_femur_.get_prev_cmd());
	prev_commands_[1] = (act_tibia_.get_prev_cmd());
}

void Leg::iterate_fsm() {
	cycles_++;
	// immediately respond to fault condition
	if (bool(act_femur_.fault()) || bool(act_tibia_.fault())) next_state_ = 
		FSMState::kRecovery;

	auto time_span = chron::steady_clock::now() - time0_s_;
	time_prog_s_ = double(time_span.count()) * chron::steady_clock::period::num / 
		chron::steady_clock::period::den;
	if (curr_state_ == FSMState::kRunning) time_fcn_s_ += time_prog_s_ - 
		time_prog_old_s_;
  time_prog_old_s_ = time_prog_s_;

	switch (curr_state_) {
		case FSMState::kIdle:
			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;
		case FSMState::kRunning:
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states
			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;
		case FSMState::kRecovery:
			act_femur_.make_stop();
			act_tibia_.make_stop();
			recovery_cycle++;
			// wait at least a few cycles
			if(recovery_cycle < recovery_cycle_thresh) break;

			// if recovery hasn't occurred, quit
			if(bool(act_femur_.fault()) || bool(act_tibia_.fault())) next_state_ =
				FSMState::kQuitting;
			
			// else, recovery has occurred, and we go back to running
			else next_state_ = FSMState::kRunning;
			break;
		case FSMState::kQuitting:
			act_femur_.make_stop();
			act_tibia_.make_stop();
			quit_cycle++;
			// wait at least a few cycles, then raise flag (main will have to quit)
			if(quit_cycle > quit_cycle_thresh) ready_to_quit = true;
			break;
		
		default:
			break;
	}

	log_data();
	return;
}

void Leg::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ","
    << act_femur_.stringify_actuator() << ","
		<< act_femur_.stringify_moteus_reply() << ","
    << act_tibia_.stringify_actuator() << ","
    << act_tibia_.stringify_moteus_reply() << "\n";
	
	return;
}
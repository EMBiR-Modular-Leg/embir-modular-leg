#ifndef MOTEUS_CONTROLLER_H
#define MOTEUS_CONTROLLER_H

#include <string>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

class MoteusController {
public:
	enum class errc : int {
		// https://github.com/mjbots/moteus/blob/main/fw/error.h
		kSuccess = 0,

		kDmaStreamTransferError = 1,
		kDmaStreamFifoError = 2,
		kUartOverrunError = 3,
		kUartFramingError = 4,
		kUartNoiseError = 5,
		kUartBufferOverrunError = 6,
		kUartParityError = 7,

		kCalibrationFault = 32,
		kMotorDriverFault = 33,
		kOverVoltage = 34,
		kEncoderFault = 35,
		kMotorNotConfigured = 36,
		kPwmCycleOverrun = 37,
		kOverTemperature = 38,
		kStartOutsideLimit = 39,
		kUnderVoltage = 40,

		// custom addition
		kMissingReply = 63,
		kOutsideLimit = 64,
		kUninitialized = 65,
	};

	MoteusController(id_t id, uint8_t bus);

	inline id_t get_id() {return id_;}
	inline void set_id(id_t new_id) {id_ = new_id;}

	void restore_cal(std::string path);
	void restore_cfg(std::string path);
	void zero_offset();

	void make_stop();
	void make_mot_position(float pos_rot, float kps=1, float kds=1,
		float ff_trq_Nm=0);
	void make_mot_velocity(float vel_Hz, float kps=1, float kds=1,
		float ff_trq_Nm=0);
	void make_mot_torque(float trq_Nm);
	void make_mot_full_pos(float pos_rot, float vel_Hz, float trq_Nm,  float kps=1, float kds=1);
	
	void retrieve_reply(
		std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies);

	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand
		get_curr_cmd() {
		return curr_cmd_;
	}
	inline void share_curr_cmd(
		mjbots::moteus::Pi3HatMoteusInterface::ServoCommand& command) {
		curr_cmd_ = command;
		curr_cmd_.resolution = res;
		curr_cmd_.query.velocity = mjbots::moteus::Resolution::kFloat;
		curr_cmd_.query.position = mjbots::moteus::Resolution::kFloat;
		curr_cmd_.query.torque = mjbots::moteus::Resolution::kFloat;
	}
	inline void share_prev_cmd(
		mjbots::moteus::Pi3HatMoteusInterface::ServoCommand& command) {
		prev_cmd_ = command;
		prev_cmd_.resolution = res;
		prev_cmd_.query.velocity = mjbots::moteus::Resolution::kFloat;
		prev_cmd_.query.position = mjbots::moteus::Resolution::kFloat;
		prev_cmd_.query.torque = mjbots::moteus::Resolution::kFloat;
	}
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_prev_cmd() {
		return prev_cmd_;
	}
	inline mjbots::moteus::Mode get_mode() {return mode_;}
	inline errc fault() {return fault_code_;}

	inline bool outside_limit() {return (
		prev_reply_.result.position < pos_lower_bound_rot_ 
		|| prev_reply_.result.position > pos_upper_bound_rot_);}

	std::string stringify_moteus_reply();

	std::string stringify_moteus_reply_header();
protected:
	id_t id_;
	uint8_t bus_;
	// this class only holds pointers to the command data; true data is owned by
	// main, where it interfaces with pi3hat
	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand curr_cmd_;
	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand prev_cmd_;
	// replies will not be by reference, as the replies vector in main is at the
	// whim of getting good replies thru CAN
	mjbots::moteus::Pi3HatMoteusInterface::ServoReply prev_reply_;
	mjbots::moteus::Mode mode_;

	mjbots::moteus::PositionResolution res;

	errc fault_code_ = errc::kSuccess;

	float pos_lower_bound_rot_ = -0.5;
	float pos_upper_bound_rot_ = 0.5;

	char cstr_buffer[128];
};

#endif
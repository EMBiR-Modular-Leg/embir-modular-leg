#ifndef MOTEUS_CONTROLLER_H
#define MOTEUS_CONTROLLER_H

#include <string>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

class MoteusController {
public:
	MoteusController(id_t id, uint8_t bus);

	inline id_t get_id() {return id_;}
	inline void set_id(id_t new_id) {id_ = new_id;}

	void restore_cal(std::string path);
	void zero_offset();

	void make_stop();
	void make_mot_position(float pos_rot, float kps=1, float kds=1,
		float ff_trq_Nm=0);
	void make_mot_velocity(float vel_Hz, float kps=1, float kds=1,
		float ff_trq_Nm=0);
	void make_mot_torque(float trq_Nm);
	
	void retrieve_reply(std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> replies);

	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_curr_cmd() {
		return curr_cmd_;
	}
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_prev_cmd() {
		return prev_cmd_;
	}

	std::string stringify_moteus_reply();

	std::string stringify_moteus_reply_header();
protected:
	id_t id_;
	uint8_t bus_;
	mjbots::moteus::Pi3HatMoteusInterface::ServoReply prev_reply_;
	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand curr_cmd_;
	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand prev_cmd_;

	char cstr_buffer[128];
};

#endif
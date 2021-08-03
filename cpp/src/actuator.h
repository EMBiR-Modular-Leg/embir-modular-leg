#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "moteus_controller.h"

#define PI 3.1415926

class Actuator : public MoteusController {
public:
	Actuator(id_t id, uint8_t bus, float gear_ratio, float trq_efficiency);
	inline float get_gear_ratio() {return gear_ratio_;}
	inline void set_gear_ratio(float new_ratio) {gear_ratio_ = new_ratio;}
	inline void make_act_position(float pos_rad, float ff_trq_Nm) {
		make_mot_position(
			pos_rad*gear_ratio_/(2*PI),
			kp_, kd_,
			ff_trq_Nm/(gear_ratio_*trq_efficiency_));
	}
	inline void make_act_velocity(float vel_rad_s, float ff_trq_Nm) {
		make_mot_velocity(
			vel_rad_s*gear_ratio_/(2*PI),
			kp_, kd_,
			ff_trq_Nm/(gear_ratio_*trq_efficiency_));
	}
	inline void make_act_torque(float trq_Nm) {
		make_mot_torque(trq_Nm/(gear_ratio_*trq_efficiency_));
	}

	std::string stringify_actuator();
	std::string stringify_actuator_header();
private:
	float gear_ratio_ = 1;
	float trq_efficiency_ = 1;
	float kp_ = 1;
	float kd_ = 1;
};

#endif
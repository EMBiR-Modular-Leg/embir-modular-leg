#ifndef LEG_H
#define LEG_H

#include <ostream>
#include <vector>
#include <functional>

#include "actuator.h"
#include "urdf.h"

#include "cxxopts/cxxopts.hpp"
#include "nlohmann/json.hpp"


class Leg {
public:

	enum FSMState : uint8_t {
		kIdle,
		kRunning,
		kRecovery,
		kQuitting,
	};

	enum ActionMode : uint8_t {
		kNone,
		kSafeZoneTest,
		kStand,
		kCyclicCrouch,
		kJump,
	};

	struct LegSettings {
		float period_s;
		float duration_s;
		float gear_femur;
		float gear_tibia;

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

	struct CyclicCrouchSettings {
		float center_m = 0;
		float amplitude_m = 0;
		float frequency_Hz = 0;
		inline CyclicCrouchSettings() {}
		inline CyclicCrouchSettings(nlohmann::json cyclic_crouch_j) {
			center_m = cyclic_crouch_j["center_m"];
			amplitude_m = cyclic_crouch_j["amplitude_m"];
			frequency_Hz = cyclic_crouch_j["frequency_Hz"];
		}
	};

	class LegKinematics {
	public:
		struct JointAngles {
			float femur_angle_rad;
			float tibia_angle_rad;

			friend JointAngles operator+(JointAngles lhs, const JointAngles& rhs) {
				return {lhs.femur_angle_rad + rhs.femur_angle_rad,
								lhs.tibia_angle_rad + rhs.tibia_angle_rad};
			}
			JointAngles operator-() {return {-femur_angle_rad, -tibia_angle_rad};}
		};
		struct AlphaAngles {
			float a1_rad;
			float a2_rad;

			friend AlphaAngles operator+(AlphaAngles lhs, const AlphaAngles& rhs) {
				return {lhs.a1_rad + rhs.a1_rad,
								lhs.a2_rad + rhs.a2_rad};
			}
			AlphaAngles operator-() {return {-a1_rad, -a2_rad};}
		};
		struct Position {
			float y_m;
			float z_m;
			friend Position operator+(Position lhs, const Position& rhs) {
				return {lhs.y_m + rhs.y_m,
								lhs.z_m + rhs.z_m};
			}
			Position operator-() {return {-y_m, -z_m};}
		};
		struct Jacobian {
			float J11, J12, J21, J22;
			Jacobian operator-() {return {-J11, -J12, -J21, -J22};}
		};
		LegKinematics(URDF& leg_urdf);
		inline LegKinematics() {}

		AlphaAngles joint2alpha(JointAngles angles);
		JointAngles	alpha2joint(AlphaAngles angles);
		std::vector<Position> fk_vec(JointAngles angles);
		std::vector<Position> fk_2link(JointAngles angles);
		JointAngles ik_2link(Position pos);
		Jacobian jacobian_alpha(AlphaAngles angles);
		Jacobian jacobian_joint(JointAngles angles);


	private:
		float l1_, l2_pll_, l2_perp_, l3_pll_, l3_perp_, r1_, r2_, gamma1_, gamma2_;
	};

	Leg(LegSettings& legset, std::ostream& datastream, std::string urdf_file);

	void iterate_fsm();

	void log_data();
	void log_headers();

	void print_status_update();

	inline FSMState get_fsm_curr_state() {return curr_state_;}
	inline FSMState get_fsm_next_state() {return next_state_;}
	inline bool is_ready_to_quit() {return ready_to_quit;}
	inline bool leg_fault() {return ((bool)act_femur_.fault() || (bool)act_tibia_.fault());}
	inline LegSettings& get_legset() {return legset_;}

	inline void set_time0(std::chrono::steady_clock::time_point t0) {time0_s_ = t0;}
	inline float get_time_prog() {return time_prog_s_;}

	// inline void share_commands(std::vector<
	// 	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& curr_commands,
	// 	std::vector<
	// 	mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>& prev_commands) {
	// 	act_femur_.share_curr_cmd(curr_commands[0]);
	// 	act_tibia_.share_curr_cmd(curr_commands[1]);
	// 	act_femur_.share_prev_cmd(prev_commands[0]);
	// 	act_tibia_.share_prev_cmd(prev_commands[1]);
	// }

	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_femur_cmd() {
		return act_femur_.get_curr_cmd();
	}
	inline mjbots::moteus::Pi3HatMoteusInterface::ServoCommand get_tibia_cmd() {
		return act_tibia_.get_curr_cmd();
	}
	
	inline void retrieve_replies(std::vector<
		mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& prev_replies) {
		act_femur_.retrieve_reply(prev_replies);
		act_tibia_.retrieve_reply(prev_replies);
	}

	LegKinematics leg_kinematics;
private:
	Actuator act_femur_;
	Actuator act_tibia_;
	std::ostream& datastream_;

	// *** LOW LEVEL ***

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

	FSMState prev_state_ = FSMState::kIdle;
	FSMState curr_state_ = FSMState::kIdle;
	FSMState next_state_ = FSMState::kIdle;

	FSMState recovery_return_state_ = FSMState::kIdle;

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

	// *** HIGH LEVEL ***

	URDF leg_urdf_;
	ActionMode action_mode_ = ActionMode::kCyclicCrouch;

  nlohmann::json cyclic_crouch_j;
	CyclicCrouchSettings cyclic_crouch_s;


	void run_action();
};

cxxopts::Options leg_opts();
Leg::LegSettings parse_settings(cxxopts::ParseResult leg_opts);

#endif
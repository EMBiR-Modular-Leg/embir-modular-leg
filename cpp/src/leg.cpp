#include <iomanip>
#include <cmath>

#include "leg.h"
#include "color.h"

namespace chron = std::chrono;

float clamp(float angle) {
	while (angle > PI) 
			angle -= 2 * PI;
	while (angle <= -PI) 
			angle += 2 * PI;
	return angle;
}

Leg::Leg(Leg::LegSettings& legset, std::ostream& datastream, std::string urdf_file) : 
	act_femur_(legset.act_femur_id, legset.act_femur_bus, legset.gear_femur, 1.0),
	act_tibia_(legset.act_tibia_id, legset.act_tibia_bus, legset.gear_tibia, 1.0),
	datastream_(datastream),
	legset_(legset),
	commands_(),
	leg_urdf_(urdf_file) {
	
	act_femur_.zero_offset();
  act_tibia_.zero_offset();
	act_femur_.restore_cfg("/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg");
	act_tibia_.restore_cfg("/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg");
	
	leg_kinematics = LegKinematics(leg_urdf_);
}

void Leg::iterate_fsm() {
	cycles_++;
	// immediately respond to fault condition
	if (leg_fault()) next_state_ = 
		FSMState::kRecovery;

	curr_state_ = next_state_;

	auto time_span = chron::steady_clock::now() - time0_s_;
	time_prog_s_ = double(time_span.count()) * chron::steady_clock::period::num / 
		chron::steady_clock::period::den;
	if (curr_state_ == FSMState::kRunning) time_fcn_s_ += time_prog_s_ - 
		time_prog_old_s_;
  time_prog_old_s_ = time_prog_s_;

	// state logic for simple test:
	// if (((int)time_prog_s_)%2) next_state_ = FSMState::kIdle;
	// else next_state_ = FSMState::kRunning;

	if (time_prog_s_ > 0.1) next_state_ = FSMState::kRunning;

	switch (curr_state_) {
		case FSMState::kIdle: {
			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states
			// act_femur_.make_stop();
			// act_tibia_.make_stop();
			// act_femur_.make_act_velocity(4.0*std::sin(time_fcn_s_), 0);
			act_femur_.make_act_position(0.8*2*PI*std::sin(4*time_fcn_s_), 0);
			// act_tibia_.make_act_velocity(4.0*std::sin(time_fcn_s_), 0);
			act_tibia_.make_act_position(0.8*2*PI*std::sin(4*time_fcn_s_), 0);
			break;}
		case FSMState::kRecovery: {
			// if coming from non-recovery, store the state so we can go back
			if(prev_state_ != FSMState::kRecovery) {
				recovery_return_state_ = prev_state_;
				recovery_cycle = 0;
			}
			act_femur_.make_stop();
			act_tibia_.make_stop();
			recovery_cycle++;
			// wait at least a few cycles
			if(recovery_cycle < recovery_cycle_thresh) break;

			// if recovery hasn't occurred, quit
			if(leg_fault()) next_state_ =
				FSMState::kQuitting;
			
			// else, recovery has occurred, and we go back to running
			else next_state_ = recovery_return_state_;
			break;}
		case FSMState::kQuitting: {
			act_femur_.make_stop();
			act_tibia_.make_stop();
			quit_cycle++;
			// wait at least a few cycles, then raise flag (main will have to quit)
			if(quit_cycle > quit_cycle_thresh) ready_to_quit = true;
			break;}
		
		default: {
			next_state_ = curr_state_;
			break;}
	}
	prev_state_ = curr_state_;
	log_data();
	return;
}

void Leg::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ","
    << act_femur_.stringify_actuator() << ","
		<< act_femur_.stringify_moteus_reply() << ","
    << act_tibia_.stringify_actuator() << ","
    << act_tibia_.stringify_moteus_reply() << ","
		<< (int)curr_state_ << "\n";
	
	return;
}

void Leg::log_headers() {
	datastream_ << "time [s],"
    << act_femur_.stringify_actuator_header() << ","
		<< act_femur_.stringify_moteus_reply_header() << ","
    << act_tibia_.stringify_actuator_header() << ","
    << act_tibia_.stringify_moteus_reply_header() << ","
		<< "leg fsm state\n";
	
	return;
}

void Leg::print_status_update() {
  Color::Modifier bg_temp_m(Color::BG_DEFAULT);
  Color::Modifier bg_temp_h(Color::BG_DEFAULT);
  Color::Modifier bg_safe(Color::BG_DEFAULT);
  
  Color::Modifier bg_temp_latch(Color::BG_DEFAULT);

  std::cout << CMod::fg_blk << CMod::bg_wht << "  t_p:"
    << std::setw(7) << std::setprecision(1) << std::fixed << time_prog_s_
    << "|t_f:"
    << std::setw(7) << std::setprecision(1) << std::fixed << time_fcn_s_
    << CMod::fg_def << CMod::bg_def << "|" ;
	std::cout << "FSM:"
		<< std::setw(2) << std::setprecision(2) << std::fixed << (int)curr_state_ << "|";
  
  if (!(bool)leg_fault()) std::cout << CMod::bg_grn << CMod::fg_blk << " **SAFE**";
  else std::cout << "|" << CMod::bg_red << CMod::fg_blk << "**FAULT**";
  std::cout << CMod::fg_def << CMod::bg_def << "\r";
  std::cout.flush();
  return;
}

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
		("act-tibia-id", "tibia actuator CAN ID",
			cxxopts::value<uint8_t>()->default_value("2"))
    ("act-femur-bus", "femur actuator CAN bus",
			cxxopts::value<uint8_t>()->default_value("3"))
    ("act-tibia-bus", "tibia actuator CAN bus",
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
  legset.gear_femur = leg_opts["gear-femur"].as<float>();
  legset.gear_tibia = leg_opts["gear-tibia"].as<float>();
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




Leg::LegKinematics::LegKinematics(URDF& leg_urdf) {
	l1_ = fabs(leg_urdf.joint_dict[std::string("knee_joint")].joint_origin.z_m);

	l2_pll_ = fabs(leg_urdf.joint_dict[std::string("ankle_joint")].joint_origin.z_m);
	l2_perp_ = fabs(leg_urdf.joint_dict[std::string("ankle_joint")].joint_origin.y_m);

	l3_pll_ = fabs(leg_urdf.joint_dict[std::string("tibia_foot_joint")].joint_origin.y_m);
	l3_perp_ = fabs(leg_urdf.joint_dict[std::string("tibia_foot_joint")].joint_origin.z_m);
	
	r1_ = std::sqrt( (l1_ + l3_perp_)*(l1_ + l3_perp_) + l3_pll_*l3_pll_);
	r2_ = std::sqrt( (l2_perp_)*(l2_perp_) + l2_pll_*l2_pll_);

	gamma1_ = std::atan2(l3_pll_, l1_+l3_perp_);
	gamma2_ = std::atan2(l2_pll_, l2_perp_);

	return;
}

Leg::LegKinematics::AlphaAngles
Leg::LegKinematics::joint2alpha(Leg::LegKinematics::JointAngles angles) {
	float a1 = -0.5*PI - angles.femur_angle_rad - gamma1_;
	float a2 = -0.5*PI - angles.tibia_angle_rad + gamma2_ + gamma1_;
	return {a1, a2};
}

Leg::LegKinematics::JointAngles
Leg::LegKinematics::alpha2joint(Leg::LegKinematics::AlphaAngles angles) {
	float femur_angle_rad = -angles.a1_rad - gamma1_ - 0.5*PI;
	float tibia_angle_rad = -angles.a2_rad + gamma1_ + gamma2_ - 0.5*PI;
	return {femur_angle_rad, tibia_angle_rad};
}

std::vector<Leg::LegKinematics::Position>
Leg::LegKinematics::fk_vec(Leg::LegKinematics::JointAngles angles) {

	float femur_angle_rad = angles.femur_angle_rad;
	float tibia_angle_rad = angles.tibia_angle_rad;
	Position p0 = Position({0,0});
	Position p1 = p0 + Position(
		{l1_*std::sin(-femur_angle_rad),
		-l1_*std::cos(-femur_angle_rad)});
	Position p21 = p1+ Position(
		{l2_pll_*std::sin(-femur_angle_rad-tibia_angle_rad),
		-l2_pll_*std::cos(-femur_angle_rad-tibia_angle_rad)});
	Position p22 = p21+ Position(
		{-l2_perp_*std::cos(-femur_angle_rad-tibia_angle_rad),
		-l2_perp_*std::sin(-femur_angle_rad-tibia_angle_rad)});
	Position p31 = p22+ Position(
		{-l3_pll_*std::cos(-femur_angle_rad),
		-l3_pll_*std::sin(-femur_angle_rad)});
	Position p32 = p31+ Position(
		{l3_perp_*std::sin(-femur_angle_rad),
		-l3_perp_*std::cos(-femur_angle_rad)});

	return {p0, p1, p21, p22, p31, p32};
}

std::vector<Leg::LegKinematics::Position>
Leg::LegKinematics::fk_2link(JointAngles angles) {
	auto alpha = joint2alpha({angles.femur_angle_rad, angles.tibia_angle_rad});
	float a1 = alpha.a1_rad;
	float a2 = alpha.a2_rad;

	Position p0 = {0,0};
	Position p1 = p0 + Position(
		{r1_*std::cos(alpha.a1_rad), r1_*std::sin(a1)});
	Position p2 = p1 + Position(
		{r2_*std::cos(alpha.a1_rad+alpha.a2_rad), r2_*std::sin(a1+a2)});

	return {p0,p1,p2};
}

Leg::LegKinematics::JointAngles
Leg::LegKinematics::ik_2link(Position pos) {
	float cosarg = (pos.y_m*pos.y_m + pos.z_m*pos.z_m - r1_*r1_ - r2_*r2_) 
		/ (2*r1_*r2_);
	if (cosarg > 1 or cosarg < -1)
		std::cerr << "invalid arccos arg: " << cosarg << std::endl;
	
	float a2 = -std::acos(cosarg);
	float a1 = std::atan2(pos.z_m,pos.y_m) 
		- std::atan2(r2_*std::sin(a2), r1_+r2_*std::cos(a2));

	auto angles = alpha2joint({a1, a2});

	return {clamp(angles.femur_angle_rad), clamp(angles.tibia_angle_rad)};
}

Leg::LegKinematics::Jacobian
Leg::LegKinematics::jacobian_alpha(Leg::LegKinematics::AlphaAngles angles) {
	float a1 = angles.a1_rad;
	float a2 = angles.a2_rad;
	float s1 = std::sin(a1);
	float c1 = std::cos(a1);
	float s12 = std::sin(a1+a2);
	float c12 = std::cos(a1+a2);

	float J11 = -r1_*s1 - r2_*s12;
	float J21 = r1_*c1 + r2_*c12;
	float J12 = -r2_*s12;
	float J22 = r2_*c12;

	return {J11, J12, J21, J22};
}

Leg::LegKinematics::Jacobian
Leg::LegKinematics::jacobian_joint(Leg::LegKinematics::JointAngles angles) {
	auto alpha = joint2alpha({angles.femur_angle_rad, angles.tibia_angle_rad});
	return -jacobian_alpha(alpha);
}
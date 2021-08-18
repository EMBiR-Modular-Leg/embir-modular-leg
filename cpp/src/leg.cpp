#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <utility>

#include <cmath>

#include "leg.h"
#include "color.h"
#include "rapidcsv/rapidcsv.h"

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
	leg_urdf_(urdf_file),
	lpf_femur_(legset.lpf_order, legset.lpf_cutoff_freq_Hz, legset.period_s),
	lpf_tibia_(legset.lpf_order, legset.lpf_cutoff_freq_Hz, legset.period_s) {
		
	action_mode_ = legset_.action_mode;
	if (legset_.playback_file != "") setup_playback();

	if (!legset_.skip_zero) {
		act_femur_.zero_offset();
		act_tibia_.zero_offset();
	}
	if (!legset_.skip_cfg_load) {
		act_femur_.restore_cfg(legset_.cfg_path);
		act_tibia_.restore_cfg(legset_.cfg_path);
	}
	
	leg_kinematics = LegKinematics(leg_urdf_);

	std::cout << "loading configs... ";
  std::ifstream cyclic_crouch_if("configs/cyclic_crouch.json");
  cyclic_crouch_if >> cyclic_crouch_j;
	cyclic_crouch_s = CyclicCrouchSettings(cyclic_crouch_j);

	std::ifstream jump_if("configs/jump.json");
  jump_if >> jump_j;
	jump_s = JumpSettings(jump_j);
	std::cout << "done.\n";
}

void Leg::setup_playback() {
	std::cout
		<< "\tloading playback file from " << legset_.playback_file << std::endl;
	rapidcsv::Document doc(legset_.playback_file,
		rapidcsv::LabelParams(),
		rapidcsv::SeparatorParams(',',
															true),
		rapidcsv::ConverterParams(true, std::numeric_limits<float>::signaling_NaN(), 0),
		rapidcsv::LineReaderParams(true /* pSkipCommentLines */,
															'#' /* pCommentPrefix */,
															true /* pSkipEmptyLines */));

	std::vector<std::string> columnNames = doc.GetColumnNames();
	// for (auto& name : columnNames) std::cout << name << "\n";
	std::cout << "\t" << doc.GetColumnCount() << " columns, ";
  std::vector<float> time = doc.GetColumn<float>("time [s]");
	std::cout << time.size() << " rows" << std::endl;
	size_t delay_idx = std::upper_bound (
		time.begin(), time.end(), legset_.playback_delay) - time.begin();
	std::vector<float>(time.begin()+delay_idx, time.end()).swap(time);
	
	// grab columns and erase elements before delay_idx
  femur_trq = doc.GetColumn<float>("a1 torque [Nm]");
	std::vector<float>(femur_trq.begin()+delay_idx, femur_trq.end()).swap(femur_trq);
	tibia_trq = doc.GetColumn<float>("a2 torque [Nm]");
	std::vector<float>(tibia_trq.begin()+delay_idx, tibia_trq.end()).swap(tibia_trq);

	// pad values for filtering
	std::cout << "\tpadding values for filtering... " << std::flush;
	for (size_t ii = 0; ii < legset_.lpf_order; ii++) {
		femur_trq.push_back(femur_trq.back());
		femur_trq.insert(femur_trq.begin(), femur_trq.front());
		tibia_trq.push_back(tibia_trq.back());
		tibia_trq.insert(tibia_trq.begin(), tibia_trq.front());
		time.push_back(time.back());
		time.insert(time.begin(), time.front());
	}
	size_t data_length = femur_trq.size();
	// take care of nans
	std::cout << "using " << data_length << " lines, taking care of nans...\n";
	for (size_t ii = 0; ii < data_length; ii++) {
		if (std::isnan(femur_trq[ii]))
			femur_trq[ii] = (ii == 0) ? 0 : femur_trq[ii-1];
		if (std::isnan(tibia_trq[ii]))
			tibia_trq[ii] = (ii == 0) ? 0 : tibia_trq[ii-1];
	}
	data_length = femur_trq.size();
	std::cout << "\tallocating new vectors... " << std::flush;
	femur_trq_temp.resize(data_length);
	tibia_trq_temp.resize(data_length);
	std::cout << "filtering playback data..." << std::flush;

	for (size_t ii = 0; ii < data_length-1; ii++) {
		// if (!(ii%100)) std::cout << ii << " " << std::flush;
		femur_trq_temp[ii] = lpf_femur_.iterate_filter(femur_trq[ii]);
		tibia_trq_temp[ii] = lpf_tibia_.iterate_filter(tibia_trq[ii]);
	}
	lpf_femur_.flush_buffers();
	lpf_tibia_.flush_buffers();
	
	// filter backwards to eliminate phase lag
	std::cout << " backward filter on " << data_length << " elements..." << std::flush;
	for (int ii = data_length-1; ii >= 0; ii--) {
		// if (!(ii%100)) std::cout << ii << " " << std::flush;
		femur_trq_temp[ii] = lpf_femur_.iterate_filter(femur_trq_temp[ii]);
		tibia_trq_temp[ii] = lpf_tibia_.iterate_filter(tibia_trq_temp[ii]);
	}

	std::cout << " done.\n\twriting file...";
	// testing 
	std::ofstream data_file("filtering_test.csv");
	data_file << "time [s],a1 torque [Nm] in,a2 torque [Nm] in,a1 torque [Nm],a2 torque [Nm]\n";
	for (size_t ii = 0; ii < femur_trq.size(); ii++)
		data_file
			<< time[ii] << ", "
			<< femur_trq[ii] << ", "
			<< tibia_trq[ii] << ", "
			<< femur_trq_temp[ii] << ", "
			<< tibia_trq_temp[ii] << "\n";
	data_file.flush();
	std::cout << " done." << std::endl;

	// cut off padded elements
	std::vector<float>(
		femur_trq_temp.begin()+legset_.lpf_order,
		femur_trq_temp.end()-legset_.lpf_order).swap(femur_trq_temp);
	std::vector<float>(
		tibia_trq_temp.begin()+legset_.lpf_order,
		tibia_trq_temp.end()-legset_.lpf_order).swap(tibia_trq_temp);


	//swap filtered data into vectors
	std::swap(femur_trq_temp, femur_trq);
	std::swap(tibia_trq_temp, tibia_trq);
}

void Leg::iterate_fsm() {
	cycles_++;
	// std::cout << "in iterate_fsm(), cycle = " << cycles_ << "\n" << std::flush;
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

	if (time_prog_s_ > legset_.action_delay) next_state_ = FSMState::kRunning;
	// std::cout << "1 " << std::flush;

	switch (curr_state_) {
		case FSMState::kIdle: {
			// std::cout << "2 " << std::flush;

			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;}
		case FSMState::kRunning: {
			// ***TODO***: Implement your operation here
			// feel free to expand into multiple states

			run_action();
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
	// std::cout << "3 " << std::flush;
	prev_state_ = curr_state_;
	log_data();
	// std::cout << "4 " << std::flush;
	return;
}

void Leg::log_data() {
	datastream_ << std::setw(10) << std::setprecision(4) << std::fixed
		<< time_prog_s_ << ",";
	// std::cout << " 3.1 " << std::flush;
  datastream_ << act_femur_.stringify_actuator() << ",";
	// std::cout << " 3.2 " << std::flush;
	datastream_ << act_femur_.stringify_moteus_reply() << ",";
	// std::cout << " 3.3 " << std::flush;
  datastream_ << act_tibia_.stringify_actuator() << ",";
	// std::cout << " 3.4 " << std::flush;
  datastream_ << act_tibia_.stringify_moteus_reply() << ",";
	// std::cout << " 3.5 " << std::flush;
	datastream_ << (int)curr_state_;
	
	if (legset_.action_mode == Leg::ActionMode::kTorquePlayback)
		datastream_ << "," << femur_trq[playback_idx] <<","
		<< tibia_trq[playback_idx];
		
	datastream_ << "\n";
	
	return;
}

void Leg::log_headers() {
	datastream_ << "time [s],"
    << act_femur_.stringify_actuator_header() << ","
		<< act_femur_.stringify_moteus_reply_header() << ","
    << act_tibia_.stringify_actuator_header() << ","
    << act_tibia_.stringify_moteus_reply_header() << ","
		<< "leg fsm state";
	if (legset_.action_mode == ActionMode::kTorquePlayback)
		datastream_ << 	",femur playback torque [Nm],tibia playback torque [Nm]";
	
	datastream_ << "\n";
	
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
				"/home/pi/embir-modular-leg/leg-data/"))
    ("playback-file", "path to csv of torque, velocity data to replay.", 
			cxxopts::value<std::string>()->default_value(""))
		("playback-delay", "jump into playback file after <delay> seconds", 
			cxxopts::value<float>()->default_value("1.0"))
    ("playback-vel-scale", "scale velocity from playback data", 
			cxxopts::value<float>()->default_value("1.0"))
    ("playback-trq-scale", "scale torque from playback data",
			cxxopts::value<float>()->default_value("1.0"))
		("lowpass-order", "butterworth lowpass filter order",
			cxxopts::value<int>()->default_value("5"))
		("lowpass-cutoff-frequency", "butterworth lowpass filter cutoff",
			cxxopts::value<float>()->default_value("50"))
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
    ("skip-zero", "skip setting actuator zero")
    ("skip-cfg-load", "skip loading actuator config")
		("config-path", "path to output csv.",
			cxxopts::value<std::string>()->default_value(
				"/home/pi/embir-modular-leg/moteus-setup/moteus-cfg/a_gen.cfg"))
    ("action-mode", "choose between [none|cyclic-crouch|torque-playback|jump]",
			cxxopts::value<std::string>()->default_value("none"))
		("action-delay", "delay start of action", 
			cxxopts::value<float>()->default_value("2"))
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}

Leg::LegSettings::LegSettings(cxxopts::ParseResult& leg_opts_in) {
  leg_opts = leg_opts_in;
  period_s = 1.0/leg_opts["frequency"].as<float>();
  duration_s = leg_opts["duration"].as<float>();
  gear_femur = leg_opts["gear-femur"].as<float>();
  gear_tibia = leg_opts["gear-tibia"].as<float>();
  act_femur_id = leg_opts["act-femur-id"].as<uint8_t>();
  act_tibia_id = leg_opts["act-tibia-id"].as<uint8_t>();
  act_femur_bus = leg_opts["act-femur-bus"].as<uint8_t>();
  act_tibia_bus = leg_opts["act-tibia-bus"].as<uint8_t>();

  main_cpu = leg_opts["main-cpu"].as<uint8_t>();
  can_cpu = leg_opts["can-cpu"].as<uint8_t>();

  playback_vel_scale = leg_opts["playback-vel-scale"].as<float>();
  playback_trq_scale = leg_opts["playback-trq-scale"].as<float>();

  lpf_order = leg_opts["lowpass-order"].as<int>();
  lpf_cutoff_freq_Hz = leg_opts["lowpass-cutoff-frequency"].as<float>();

  playback_file = leg_opts["playback-file"].as<std::string>();
  playback_delay = leg_opts["playback-delay"].as<float>();

	auto test_str = leg_opts["action-mode"].as<std::string>();
  // case invariance -- convert to user input to lowercase
  std::transform(test_str.begin(), test_str.end(), test_str.begin(),
    [](unsigned char c){ return std::tolower(c); });

  // std::cout << test_str << (test_str == std::string("TV-sweep")) << std::endl;
  if (test_str == std::string("none")) action_mode = Leg::ActionMode::kNone;
  else if (test_str == std::string("cyclic-crouch")) action_mode = Leg::ActionMode::kCyclicCrouch;
  else if (test_str == std::string("torque-playback")) action_mode = Leg::ActionMode::kTorquePlayback;
  else if (test_str == std::string("jump")) action_mode = Leg::ActionMode::kJump;
  else {
    action_mode = Leg::ActionMode::kNone;
    std::cout << "no test mode selected (input was \"" << test_str << "\")." << std::endl;
  }
  std::cout << "test mode \"" << test_str << "\" selected" << std::endl;

  action_delay = leg_opts["action-delay"].as<float>();

	skip_cal = leg_opts["skip-cal"].as<bool>();
	skip_zero = leg_opts["skip-zero"].as<bool>();
	skip_cfg_load = leg_opts["skip-cfg-load"].as<bool>();
	cfg_path = leg_opts["config-path"].as<std::string>();
}

void Leg::run_action() {
	// std::cout << "in run_action()" << std::endl;
	
	switch (action_mode_) {
		case ActionMode::kNone: {
			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;}
		case ActionMode::kSafeZoneTest: {
			break;}
		case ActionMode::kStand: {
			break;}
		case ActionMode::kCyclicCrouch: {
			// std::cout << "in kCyclicCrouch" << std::endl;
			float z = cyclic_crouch_s.amplitude_m
				*std::sin(2*PI*cyclic_crouch_s.frequency_Hz*time_fcn_s_) 
				+ cyclic_crouch_s.center_m;
			LegKinematics::Position desired_foot_pos = {0,z};

			LegKinematics::JointAngles curr_angles
				= {act_femur_.get_position_rad(), act_tibia_.get_position_rad()};
			LegKinematics::Position curr_foot_pos
				= leg_kinematics.fk_2link(curr_angles).back();
			
			auto delta_foot_pos = desired_foot_pos - curr_foot_pos;
			float max_linear_vel = 0.5; // m/s
			float max_dist = max_linear_vel * legset_.period_s
				+ (!arrived_at_action_latch) * delta_foot_pos.magnitude() * time_fcn_s_;
				// spoofing integral winding up effect

			float arrival_tolerance_m = 0.01;
			if (!arrived_at_action_latch) {
				if (delta_foot_pos.magnitude() < arrival_tolerance_m) {
					arrived_at_action_latch = true;
					std::cout << "\narrived at crouch location\n";
				}
				else
					delta_foot_pos = delta_foot_pos.magnitude() > max_dist ?
						max_dist*delta_foot_pos/delta_foot_pos.magnitude() : // slow down
						delta_foot_pos; // original
			}

			auto foot_pos = curr_foot_pos + delta_foot_pos;

			// std::cout 
			// 	<< "curr_ang: (" << curr_angles.femur_angle_rad << ", " << curr_angles.tibia_angle_rad << "), "
			// 	<< "curr: (" << curr_foot_pos.y_m << ", " << curr_foot_pos.z_m << "), "
			// 	<< "desired: (" << desired_foot_pos.y_m << ", " << desired_foot_pos.z_m << "), "
			// 	<< "delta: (" << delta_foot_pos.y_m << ", " << delta_foot_pos.z_m << "), "
			// 	<< "cmd: (" << foot_pos.y_m << ", " << foot_pos.z_m << ")\n";

			auto joint_angles = leg_kinematics.ik_2link(foot_pos);

			float dzdt = cyclic_crouch_s.amplitude_m
				*2*PI*cyclic_crouch_s.frequency_Hz
				*std::cos(2*PI*cyclic_crouch_s.frequency_Hz*time_fcn_s_);

			LegKinematics::Position foot_vel = {0,dzdt};
			// auto foot_vel = (foot_pos - curr_foot_pos)/legset_.period_s;
			auto joint_vels = leg_kinematics.task2joint(foot_vel, joint_angles); 

			act_femur_.make_act_full_pos(
				joint_angles.femur_angle_rad, joint_vels.femur_angle_rad, 0);
			act_tibia_.make_act_full_pos(
				joint_angles.tibia_angle_rad, joint_vels.tibia_angle_rad, 0);

			// std::cout << "leaving kCyclicCrouch" << std::endl;
			break;}
		case ActionMode::kTorquePlayback: {
			act_femur_.make_act_torque(
				femur_trq[playback_idx]*legset_.playback_trq_scale);
			act_tibia_.make_act_torque(
				tibia_trq[playback_idx]*legset_.playback_trq_scale);
			playback_idx++;

			break;}
		case ActionMode::kJump: {
			LegKinematics::JointAngles curr_angles
				= {act_femur_.get_position_rad(), act_tibia_.get_position_rad()};
			LegKinematics::Position curr_foot_pos
				= leg_kinematics.fk_2link(curr_angles).back();
			
			float t_quad = (jump_s.time_0 >= 0.0f) ?
				std::max(time_fcn_s_-jump_s.time_0, 0.0f) : 0.0;
			

			float z = 0.5*jump_s.accel_s_m_s2*t_quad*t_quad + jump_s.initial_ext_m;
			z = std::max(z, jump_s.final_ext_m);
			LegKinematics::Position desired_foot_pos = {0,z};
			
			auto delta_foot_pos = desired_foot_pos - curr_foot_pos;
			float max_linear_vel = 0.5; // m/s
			float max_dist = max_linear_vel * legset_.period_s
				+ (!arrived_at_action_latch) * delta_foot_pos.magnitude() * time_fcn_s_;
				// spoofing integral winding up effect

			float arrival_tolerance_m = 0.01;
			if (!arrived_at_action_latch) {
				if (delta_foot_pos.magnitude() < arrival_tolerance_m) {
					arrived_at_action_latch = true;
					std::cout << "\narrived at jump location\n";
					jump_s.time_0 = time_fcn_s_ + 1; // 1s delay before starting jump
				}
				else
					delta_foot_pos = delta_foot_pos.magnitude() > max_dist ?
						max_dist*delta_foot_pos/delta_foot_pos.magnitude() : // slow down
						delta_foot_pos; // original
			}

			auto foot_pos = curr_foot_pos + delta_foot_pos;

			auto joint_angles = leg_kinematics.ik_2link(foot_pos);

			float dzdt = z > jump_s.final_ext_m ? jump_s.accel_s_m_s2*t_quad : 0.0f;

			LegKinematics::Position foot_vel = {0,dzdt};
			// auto foot_vel = (foot_pos - curr_foot_pos)/legset_.period_s;
			auto joint_vels = leg_kinematics.task2joint(foot_vel, joint_angles); 

			act_femur_.make_act_full_pos(
				joint_angles.femur_angle_rad, joint_vels.femur_angle_rad, 0);
			act_tibia_.make_act_full_pos(
				joint_angles.tibia_angle_rad, joint_vels.tibia_angle_rad, 0);

			// std::cout << "leaving kJump" << std::endl;
			break;}
		default: {
			act_femur_.make_stop();
			act_tibia_.make_stop();
			break;}
	}
	return;
}


// KINEMATICS

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

Leg::LegKinematics::JointAngles Leg::LegKinematics::task2joint(
	Leg::LegKinematics::Position task, Leg::LegKinematics::JointAngles angles) {
	
	auto J = jacobian_joint(angles);
	float det = ( J.J11*J.J22 - J.J12*J.J21 );
	if (fabs(det) < 0.001) 
		std::cerr << "non invertible jacobian!!" << std::endl;
	
	return {( J.J22*task.y_m - J.J12*task.z_m)/det,
					(-J.J21*task.y_m + J.J11*task.z_m)/det};
}
// Leg::LegKinematics::JointAngles Leg::LegKinematics::task2joint(
// 	Leg::LegKinematics::Position& task, Leg::LegKinematics::JointAngles& angles) {
// 	return task2joint(task, jacobian_joint(angles));
// }
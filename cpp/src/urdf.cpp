
#include <iostream>
#include <fstream>
#include <sstream>

#include "urdf.h"

using namespace rapidxml;

std::vector<std::string> split (std::string s, std::string delimiter) {
	// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
	size_t pos_start = 0, pos_end, delim_len = delimiter.length();
	std::string token;
	std::vector<std::string> res;

	while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
		token = s.substr (pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		res.push_back (token);
	}

	res.push_back (s.substr (pos_start));
	return res;
}

float get_inertia(xml_node<> *inertial_ptr, std::string i_label) {
	std::string i_str =
		inertial_ptr->first_node("inertia")->first_attribute(i_label.c_str())->value();
	return std::stof(i_str);
}

URDF::RobotElement::Origin URDF::RobotElement::get_origin(
	rapidxml::xml_node<> *origin_node_ptr) {
	
	std::string xyz_str =
		origin_node_ptr->first_attribute("xyz")->value();
	std::string rpy_str =
		origin_node_ptr->first_attribute("rpy")->value();
	std::string::size_type sz;
	URDF::RobotElement::Origin origin;
	std::stringstream ss_xyz(xyz_str);
  // origin.x_m = std::stof (xyz_str,&sz);
  // origin.y_m = std::stof (xyz_str.substr(sz), &sz);
  // origin.z_m = std::stof (xyz_str.substr(sz), &sz);

	ss_xyz >> origin.x_m;
	ss_xyz >> origin.y_m;
	ss_xyz >> origin.z_m;

	std::stringstream ss_rpy(rpy_str);
	// origin.r_rad = std::stof (rpy_str,&sz);
  // origin.p_rad = std::stof (rpy_str.substr(sz), &sz);
  // origin.y_rad = std::stof (rpy_str.substr(sz), &sz);

	ss_rpy >> origin.r_rad;
	ss_rpy >> origin.p_rad;
	ss_rpy >> origin.y_rad;

	// std::cout << "origin: "
	// 	<< origin.x_m << ", "
	// 	<< origin.y_m << ", "
	// 	<< origin.z_m << ", "
	// 	<< origin.r_rad << ", "
	// 	<< origin.p_rad << ", "
	// 	<< origin.y_rad << std::endl;

	return origin;
}

URDF::URDF(std::string path) : path_(path) {
	// https://gist.github.com/JSchaenzle/2726944
	std::cout << "parsing robot URDF from " << path << std::endl;
	xml_node<> * root_node;
	// Read the xml file into a vector
	std::ifstream urdf_file (path_);
	std::vector<char> buffer((std::istreambuf_iterator<char>(urdf_file)),
		std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	// Parse the buffer using the xml file parsing library into doc 
	doc.parse<0>(&buffer[0]);
	// Find our root node
	root_node = doc.first_node("robot");

	for (xml_node<> * rob_elem_node = root_node->first_node("link");
		rob_elem_node;
		rob_elem_node = rob_elem_node->next_sibling()) {

		std::string elem_name = rob_elem_node->name();

		if (elem_name == "link") links.push_back(Link(rob_elem_node));
		else if (elem_name == "joint") joints.push_back(Joint(rob_elem_node));
		else std::cerr 
			<< "unrecognized element in URDF: " << elem_name << std::endl;
	}

	// populate map dict
	// consider changing this to an index lookup to the vectors
	for (size_t ii = 0; ii < joints.size(); ii++)
		joint_dict[joints[ii].name] = joints[ii];

	for (size_t ii = 0; ii < links.size(); ii++)
		link_dict[links[ii].name] = links[ii];

	return;
}

URDF::RobotElement::RobotElement(xml_node<> * rob_elem_node) {
	name = rob_elem_node->first_attribute("name")->value();
	// std::cout << name << ":  ";
	return;
}

URDF::Link::Link(xml_node<> * link_node) : RobotElement(link_node) {
	type_ = ElementType::kLink;
	auto inertial_ptr = link_node->first_node("inertial");
	
  mass_origin = get_origin(inertial_ptr->first_node("origin"));
	
	std::string mass_str =
		inertial_ptr->first_node("mass")->first_attribute("value")->value();
	mass_kg = std::stof(mass_str);
	// std::cout << "mass: " << mass_kg << std::endl;

	inertia.i_xx_kgm2 = get_inertia(inertial_ptr, "ixx");
	inertia.i_xy_kgm2 = get_inertia(inertial_ptr, "ixy");
	inertia.i_xz_kgm2 = get_inertia(inertial_ptr, "ixz");
	inertia.i_yy_kgm2 = get_inertia(inertial_ptr, "iyy");
	inertia.i_yz_kgm2 = get_inertia(inertial_ptr, "iyz");
	inertia.i_zz_kgm2 = get_inertia(inertial_ptr, "izz");
	return;
}

URDF::Joint::Joint(xml_node<> * joint_node) : RobotElement(joint_node) {
	type_ = ElementType::kJoint;
	joint_origin = get_origin(joint_node->first_node("origin"));
	determine_joint_type(joint_node->first_attribute("type")->value());

	parent_name =
		joint_node->first_node("parent")->first_attribute("link")->value();
	child_name =
		joint_node->first_node("child")->first_attribute("link")->value();

	std::string xyz_str =
		joint_node->first_node("axis")->first_attribute("xyz")->value();
	std::stringstream ss(xyz_str);
	ss >> joint_axis.x;
	ss >> joint_axis.y;
	ss >> joint_axis.z;
	// std::cout << "axis: "
	// 	<< joint_axis.x << ", "
	// 	<< joint_axis.y << ", "
	// 	<< joint_axis.z << std::endl;

	auto limit_ptr = joint_node->first_node("limit");
	if (!limit_ptr) return;
	lower_lim    = std::stof(limit_ptr->first_attribute(   "lower")->value());
	upper_lim    = std::stof(limit_ptr->first_attribute(   "upper")->value());
	effort_lim   = std::stof(limit_ptr->first_attribute(  "effort")->value());
	velocity_lim = std::stof(limit_ptr->first_attribute("velocity")->value());

	return;
}

void URDF::Joint::determine_joint_type(std::string label) {
	if (label == "revolute") joint_type = JointType::kRevolute;
	else if (label == "continuous") joint_type = JointType::kContinuous;
	else if (label == "prismatic") joint_type = JointType::kPrismatic;
	else if (label == "fixed") joint_type = JointType::kFixed;
	else if (label == "floating") joint_type = JointType::kFloating;
	else if (label == "planar") joint_type = JointType::kPlanar;
	return;
}
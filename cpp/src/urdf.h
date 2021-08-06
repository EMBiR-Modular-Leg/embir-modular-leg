#ifndef URDF_H
#define URDF_H

#include <vector>
#include <string>
#include <functional>
#include <map>

#include "rapidxml/rapidxml.hpp"


class URDF {
public :

	class RobotElement {
	public:
		enum ElementType {
			kLink,
			kJoint
		};
		struct Origin {
			float x_m = 0;
			float y_m = 0;
			float z_m = 0;
			float r_rad = 0;
			float p_rad = 0;
			float y_rad = 0;
		};
		struct Axis {
			float r_rad = 0;
			float p_rad = 0;
			float y_rad = 0;
		};

		RobotElement(rapidxml::xml_node<> * elem_node);

		inline virtual bool is_link() {return false;}
		inline virtual bool is_joint() {return false;}

		Origin get_origin(rapidxml::xml_node<> *origin_node_ptr);

		std::string name;
		
	protected :
		ElementType type_;
	};

	class Link : public RobotElement {
	public:
		struct Inertia {
			float i_xx_kgm2 = 0;
			float i_xy_kgm2 = 0;
			float i_xz_kgm2 = 0;
			float i_yy_kgm2 = 0;
			float i_yz_kgm2 = 0;
			float i_zz_kgm2 = 0;
		};
		Link(rapidxml::xml_node<> * link_node);
		inline bool is_link() {return true;}
		float mass_kg; //
		Origin mass_origin; //
		Inertia inertia; //
	};

	class Joint : public RobotElement {
	public:
		enum JointType {
			kRevolute,
			kContinuous,
			kPrismatic,
			kFixed,
			kFloating,
			kPlanar
		};
		struct Axis {
			float x = 0;
			float y = 0;
			float z = 0;
		};

		Joint(rapidxml::xml_node<> * joint_node);
		inline bool is_joint() {return true;}
		std::string parent_name; //
		std::string child_name; //
		JointType joint_type = JointType::kRevolute; //

		void determine_joint_type(std::string label);

		float lower_lim; //
		float upper_lim; //
		float effort_lim; //
		float velocity_lim; //

		Origin joint_origin; //
		Axis joint_axis; //
	};

	URDF(std::string path);

	std::vector<std::reference_wrapper<RobotElement>> elems;
	std::vector<Joint> joints;
	std::vector<Link> links;
	std::map<std::string, std::reference_wrapper<RobotElement>> elem_dict;

private :

	std::string path_;
	rapidxml::xml_document<> doc;

};

#endif
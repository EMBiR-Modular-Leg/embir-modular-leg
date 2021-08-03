#include <sstream>
#include "actuator.h"

Actuator::Actuator(id_t id, uint8_t bus, float gear_ratio, float trq_efficiency) : MoteusController(id, bus), gear_ratio_(gear_ratio),
	trq_efficiency_(trq_efficiency) {
	return;
	
}

std::string Actuator::stringify_actuator() {
  std::ostringstream result;
  auto& cmd_data = curr_cmd_.position;
  sprintf(cstr_buffer, "% -f,% -f,% -f,",
    cmd_data.position*2*PI,
    cmd_data.velocity*2*PI,
    cmd_data.feedforward_torque);
  result << cstr_buffer;

  auto& reply_data = prev_reply_.result;
  sprintf(cstr_buffer, "% -f,% -f,% -f",
    reply_data.position*2*PI,
    reply_data.velocity*2*PI,
    reply_data.torque);
  result << cstr_buffer;
  return result.str();
}

std::string Actuator::stringify_actuator_header() {
  std::ostringstream result;
  std::string aN = "a"+std::to_string(id_)+" ";
  result << aN << "position cmd [rad]," << aN << "velocity cmd [rad/s]," << aN << "ff torque cmd [Nm],"
    << aN << "position [rad]," << aN << "velocity [rad/s]," << aN << "torque [Nm]";
  return result.str();
}
#include "mirena_car.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MirenaCar::_bind_methods()
{
	// GAS
	ClassDB::bind_method(D_METHOD("get_gas"), &MirenaCar::get_gas);
	ClassDB::bind_method(D_METHOD("set_gas", "_gas"), &MirenaCar::set_gas);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "gas"), "set_gas", "get_gas");
	// BRAKE
	ClassDB::bind_method(D_METHOD("get_brake"), &MirenaCar::get_brake);
	ClassDB::bind_method(D_METHOD("set_brake", "_brake"), &MirenaCar::set_brake);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "brake"), "set_brake", "get_brake");
	// STEER_ANGLE
	ClassDB::bind_method(D_METHOD("get_steer_angle"), &MirenaCar::get_steer_angle);
	ClassDB::bind_method(D_METHOD("set_steer_angle", "_steer_angle"), &MirenaCar::set_steer_angle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer_angle"), "set_steer_angle", "get_steer_angle");
}

MirenaCar::MirenaCar()
{
}

MirenaCar::~MirenaCar()
{

}

//------------------------------------------------------------ GODOT ---------------------------------------------------------//

void MirenaCar::_ros_ready()
{
	// Zero internal variables
	gas = 0;
	brake = 0;
	steer_angle = 0;
	rosSub = ros_node->create_subscription<mirena_msg::msg::CarControls>(
		"car_control", 10, std::bind(&MirenaCar::topic_callback, this, std::placeholders::_1));
}

// Getters and setters
void MirenaCar::set_gas(uint8_t _gas)
{
	gas = _gas;
}
uint8_t MirenaCar::get_gas()
{
	return gas;
}
void MirenaCar::set_brake(uint8_t _brake)
{
	brake = _brake;
}
uint8_t MirenaCar::get_brake()
{
	return brake;
}
void MirenaCar::set_steer_angle(float _steer_angle)
{
	steer_angle = _steer_angle;
}
float MirenaCar::get_steer_angle()
{
	return steer_angle;
}

//------------------------------------------------------------ ROS ---------------------------------------------------------//
void MirenaCar::topic_callback(const mirena_msg::msg::CarControls::SharedPtr msg)
{
	gas = msg->gas;
	brake = msg->brake;
	steer_angle = msg->steer_angle;
}
#include "mirena_car.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MirenaCar::_bind_methods() {
	//GAS
	BIND_PROPERTY_RW(MirenaCar, gas, Variant::INT);
	//BRAKE
	BIND_PROPERTY_RW(MirenaCar, brake, Variant::INT);
	//STEER_ANGLE
	BIND_PROPERTY_RW(MirenaCar, steer_angle, Variant::FLOAT);
	//X_POS
	//BIND_PROPERTY(MirenaCar, x_pos, Variant::INT)
	//Y_POS
	//ANGLE

}

MirenaCar::MirenaCar() {
	//Zero internal variables
	gas = 0;
	brake = 0;
	steer_angle = 0;


	//ROS setup
	godot::UtilityFunctions::print("Arrancando ROS");
	if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr); // Initialize ROS2, if not already initialized
    }
	godot::UtilityFunctions::print("ROS Arrancado");

	if (!rosNode) {
       rosNode = rclcpp::Node::make_shared("MirenaCar");
    }
	godot::UtilityFunctions::print("Ros Node created");


    rosSub = rosNode->create_subscription<mirena_msg::msg::CarControls>(
    "carControl", 10, std::bind(&MirenaCar::topic_callback, this, std::placeholders::_1));

	godot::UtilityFunctions::print("Subscribed");

}

MirenaCar::~MirenaCar() {
	rosNode.reset();//Destroy Node
    rclcpp::shutdown(); //Deinit ros connection
	godot::UtilityFunctions::print("Class Destroyed");

}



//------------------------------------------------------------ GODOT ---------------------------------------------------------//
void MirenaCar::_process(double delta) {
	rclcpp::spin_some(rosNode);
}

//Getters and setters
	void MirenaCar::set_gas(uint8_t _gas){
		gas = _gas;
	}
	uint8_t MirenaCar::get_gas(){
		return gas;
	}
	void MirenaCar::set_brake(uint8_t _brake){
		brake = _brake;
	}
	uint8_t MirenaCar::get_brake(){
		return brake;
	}
	void MirenaCar::set_steer_angle(double _steer_angle){
		steer_angle = _steer_angle;
	}
	double MirenaCar::get_steer_angle(){
		return steer_angle;
	}
	

//------------------------------------------------------------ ROS ---------------------------------------------------------//
void MirenaCar::topic_callback(const mirena_msg::msg::CarControls::SharedPtr msg){
	gas = msg->gas;
	brake = msg->brake;
	steer_angle = msg->steer_angle;
}

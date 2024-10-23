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
	BIND_PROPERTY_W(MirenaCar, x_pos, Variant::FLOAT);
	//Y_POS
	BIND_PROPERTY_W(MirenaCar, y_pos, Variant::FLOAT);
	//ANGLE
	BIND_PROPERTY_W(MirenaCar, car_angle, Variant::FLOAT);	


}

MirenaCar::MirenaCar() {
	//Zero internal variables
	gas = 0; brake = 0; steer_angle = 0;

	x_pos = 0; y_pos = 0; car_angle = 0;


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


    rosControlsSub = rosNode->create_subscription<mirena_msg::msg::CarControls>(
    	"carControl", 10, std::bind(&MirenaCar::control_sub_callback, this, std::placeholders::_1)
	);
	rosPositionPub = rosNode->create_publisher<mirena_msg::msg::CarPosition>(
		"carPosition", 10 
	);
	rosAccelerationPub = rosNode->create_publisher<mirena_msg::msg::CarAcceleration>(
		"carAcceleration", 10 
	);

	godot::UtilityFunctions::print("Pubs/Subs Initialized");

}

MirenaCar::~MirenaCar() {
	rosNode.reset();//Destroy Node
    rclcpp::shutdown(); //Deinit ros connection
	godot::UtilityFunctions::print("Class Destroyed");

}



//------------------------------------------------------------ GODOT ---------------------------------------------------------//
void MirenaCar::_process(double delta) {
	rclcpp::spin_some(rosNode);
	this->publish_position();
	this->publish_acceleration(delta);
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

	void MirenaCar::set_x_pos(double _x_pos){
		x_pos = _x_pos;
	}

	void MirenaCar::set_y_pos(double _y_pos){
		y_pos = _y_pos;
	}

	void MirenaCar::set_car_angle(double _car_angle){
		car_angle = _car_angle;
	}	

//------------------------------------------------------------ ROS ---------------------------------------------------------//
void MirenaCar::control_sub_callback(const mirena_msg::msg::CarControls::SharedPtr msg){
	gas = msg->gas;
	brake = msg->brake;
	steer_angle = msg->steer_angle;
}

void MirenaCar::publish_position(){
	auto msg = mirena_msg::msg::CarPosition();
	msg.x_position = this->x_pos;
	msg.y_position = this->y_pos;
	msg.angle = this->car_angle;
	this->rosPositionPub->publish(msg);
}

void MirenaCar::publish_acceleration(double delta){
	// NOT IMPLEMENTED;
	// TODO: IMPLEMENT THIS
	auto msg = mirena_msg::msg::CarAcceleration();
	msg.x_acceleration = 0;
	msg.y_acceleration = 0;
	msg.tang_acceleration = 0;
	msg.norm_acceleration = 0;
	this->rosAccelerationPub->publish(msg);
}
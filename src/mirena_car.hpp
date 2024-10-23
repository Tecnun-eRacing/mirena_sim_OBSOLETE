#ifndef MIRENACAR_H
#define MIRENACAR_H

//GODOT
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "mirena_msg/msg/car_controls.hpp"  
#include "mirena_msg/msg/car_position.hpp"
#include "mirena_msg/msg/car_acceleration.hpp"  

//OTHER
#include "custom_macros.hpp"

namespace godot {

class MirenaCar : public Node {
	GDCLASS(MirenaCar, Node)
private:
    //ROS subscriber and callback
	rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<mirena_msg::msg::CarControls>::SharedPtr rosControlsSub;
	rclcpp::Publisher<mirena_msg::msg::CarPosition>::SharedPtr rosPositionPub;
	rclcpp::Publisher<mirena_msg::msg::CarAcceleration>::SharedPtr rosAccelerationPub;
    // Callback function for when a message is received

    // Subscription object
    rclcpp::Subscription<mirena_msg::msg::CarControls>::SharedPtr subscription_;


	//Internal Car Inputs
	uint8_t gas;
	uint8_t brake;
	double steer_angle;

	//Car position Outputs
	double x_pos;
	double y_pos;
	double car_angle;

protected:
	static void _bind_methods();

public:
	//Constructors
	MirenaCar();
	~MirenaCar();
	//Getters and setters
	void set_gas(uint8_t _gas);
	uint8_t get_gas();
	void set_brake(uint8_t _brake);
	uint8_t get_brake();
	void set_steer_angle(double _steer_angle);
	double get_steer_angle();
	void set_x_pos(double _x_pos);
	void set_y_pos(double _y_pos);
	void set_car_angle(double _car_angle);	

	//Godot
	void _process(double delta) override;

	//ROS
	void control_sub_callback(const mirena_msg::msg::CarControls::SharedPtr msg);
	void publish_position();
	void publish_acceleration(double delta);

};

}

#endif
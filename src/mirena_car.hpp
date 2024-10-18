#ifndef MIRENACAR_H
#define MIRENACAR_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "mirena_msg/msg/car_controls.hpp"  

namespace godot {

class MirenaCar : public Node {
	GDCLASS(MirenaCar, Node)
private:
    //ROS subscriber and callback
	rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<mirena_msg::msg::CarControls>::SharedPtr rosSub;
    // Callback function for when a message is received

    // Subscription object
    rclcpp::Subscription<mirena_msg::msg::CarControls>::SharedPtr subscription_;


	//Internal Car Inputs
	uint8_t gas;
	uint8_t brake;
	float steer_angle;

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
	void set_steer_angle(float _steer_angle);
	float get_steer_angle();
	//Godot
	void _process(double delta) override;

	//ROS
	void topic_callback(const mirena_msg::msg::CarControls::SharedPtr msg);

};

}

#endif
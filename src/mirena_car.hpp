#ifndef MIRENACAR_H
#define MIRENACAR_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "mirena_common/msg/car_controls.hpp"

#include "ros_node3d.hpp"

namespace godot
{

	class MirenaCar : public RosNode3D
	{
		GDCLASS(MirenaCar, RosNode3D)
	private:
		// ROS subscriber and callback
		rclcpp::Subscription<mirena_common::msg::CarControls>::SharedPtr rosSub;
		// Internal Car Inputs
		uint8_t gas;
		uint8_t brake;
		float steer_angle;

	protected:
		static void _bind_methods();

	public:
		// Constructors
		MirenaCar();
		~MirenaCar();
		// Getters and setters
		void set_gas(uint8_t _gas);
		uint8_t get_gas();
		void set_brake(uint8_t _brake);
		uint8_t get_brake();
		void set_steer_angle(float _steer_angle);
		float get_steer_angle();
		// Godot runtime
		void _ros_ready() override;
		// ROS
		void topic_callback(const mirena_common::msg::CarControls::SharedPtr msg);
	};

}

#endif

#ifndef MIRENA_ROS_PUBSUB_HPP
#define MIRENA_ROS_PUBSUB_HPP

#include <godot_cpp/classes/ref_counted.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define DEBUG_FULL_TRACK_TOPIC "debug/sim/full_track_path"
#define DEBUG_IMMEDIATE_TRACK_TOPIC "debug/sim/immediate_track_path"

#define FIXED_FRAME_NAME "world"

namespace godot
{
	// Refcounted object used as a binding for publishing and subscribing to topics, as well as hosting and requesting services
	class MirenaRosPubSub: public RefCounted 
	{
		GDCLASS(MirenaRosPubSub, RefCounted);

	private:
		rclcpp::Publisher<visualization_msgs::msg::Marker:SharedPtr> debugFullTrackPub; // Line strip
		rclcpp::Publisher<visualization_msgs::msg::Marker:SharedPtr> debugImmediateTrackPub; // Line strip

	protected:
		static void _bind_methods();

	public:
		// Constructors
		MirenaRosPubSub();
		~MirenaRosPubSub();
		// Getters and setters



		// Godot runtime
		void _ros_ready() override;
		void _ros_process(double delta) override;
		// ROS
		void topic_callback(const mirena_common::msg::CarInput::SharedPtr msg);
	};

}

#endif

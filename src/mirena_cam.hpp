#ifndef MIRENACAM_H
#define MIRENACAM_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/image.hpp>
//ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace godot {

class MirenaCam : public Node {
	GDCLASS(MirenaCam, Node)
private:
    //ROS publisher
	rclcpp::Node::SharedPtr rosNode;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPub;
	std::unique_ptr<sensor_msgs::msg::Image> frame; //Image to publish


protected:
	static void _bind_methods();

public:
	//Constructors
	MirenaCam();
	~MirenaCam();

	//Godot
	void _process(double delta) override;
	 void pubFrame(const Ref<Image> & img);
	//ROS

};

}

#endif
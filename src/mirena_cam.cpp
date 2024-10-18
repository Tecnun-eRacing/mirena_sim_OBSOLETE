#include "mirena_cam.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MirenaCam::_bind_methods()
{
	// Publish Frame
	ClassDB::bind_method(D_METHOD("pubFrame","img"), &MirenaCam::pubFrame);
	godot::UtilityFunctions::print("Methods binded");

}

MirenaCam::MirenaCam()
{
	// ROS setup
	godot::UtilityFunctions::print("Arrancando ROS");
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr); // Initialize ROS2, if not already initialized
	}
	godot::UtilityFunctions::print("ROS Arrancado");

	if (!rosNode)
	{
		rosNode = rclcpp::Node::make_shared("MirenaCam");
	}
	godot::UtilityFunctions::print("Ros Node created");

	rosPub = rosNode->create_publisher < sensor_msgs::msg::Image > ("cam0", 10);
		godot::UtilityFunctions::print("Publisher created");

}

MirenaCam::~MirenaCam()
{
	rosNode.reset();	// Destroy Node
	rclcpp::shutdown(); // Deinit ros connection
	godot::UtilityFunctions::print("Class Destroyed");
}

//------------------------------------------------------------ GODOT ---------------------------------------------------------//
void MirenaCam::_process(double delta)
{
	rclcpp::spin_some(rosNode);
}

void MirenaCam::pubFrame(const Ref<Image> &img)
{
	frame = std::make_unique<sensor_msgs::msg::Image>(); //Image to publish
	frame->height = img->get_height();
	frame->width = img->get_width();
	godot::UtilityFunctions::print("img created");


	frame->encoding = "rgb8";
	frame->is_bigendian = false;
	frame->step = img->get_data().size() / frame->height;
	frame->data.resize(img->get_data().size());
	godot::UtilityFunctions::print(" before memcpy");

	std::memcpy(&frame->data[0], img->get_data().ptrw(), img->get_data().size());
	rosPub->publish(std::move(frame));
}

//------------------------------------------------------------ ROS ---------------------------------------------------------//

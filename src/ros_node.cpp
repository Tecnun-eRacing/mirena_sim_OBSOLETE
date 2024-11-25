// ros2_node.cpp
#include "ros_node.hpp"

using namespace godot;

RosNode::_bind_methods()
{
}

RosNode::RosNode()
{
}

RosNode::~RosNode()
{
}

void RosNode::_ready()
{
    if (!Engine::get_singleton()->is_editor_hint())
    { // Comprobamos que estamos en el juego
        // Ensure ROS2 is initialized
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        // Create ROS2 node with the same name as the Godot node
        std::string node_name = get_name();
        ros_node = rclcpp::Node::make_shared(node_name); //Creamos un puntero compartido para nuestras derivadas
    }
}

void RosNode::_process(double delta)
{
    // Spin the ROS node to process callbacks
    if (ros_node)
    {
        rclcpp::spin_some(ros_node);
    }
}

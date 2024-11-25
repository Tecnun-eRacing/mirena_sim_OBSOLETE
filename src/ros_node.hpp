#pragma once
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace godot {

class RosNode : public Node {
    GDCLASS(RosNode, Node)

private:
    std::shared_ptr<rclcpp::Node> ros_node;

protected:
    static void _bind_methods();

public:
    ROS2Node();
    ~ROS2Node();

    void _ready() override;
    void _process(double delta) override;

    std::shared_ptr<rclcpp::Node> get_ros_node() { return ros_node; }
};

}
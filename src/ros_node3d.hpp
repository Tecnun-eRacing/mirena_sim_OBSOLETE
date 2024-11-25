// ros_node_3d.h
#ifndef ROS_NODE_3D_H
#define ROS_NODE_3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace godot {

class RosNode3D : public Node3D {
    GDCLASS(RosNode3D, Node3D)

private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher;
    
    bool publish_tf = true;
    double publish_rate = 0.1; // 10 Hz
    double elapsed_time = 0.0;

    RosNode3D* find_nearest_ros_parent();

protected:
    static void _bind_methods();

public:
    RosNode3D();
    ~RosNode3D();

    void _ready() override;
    void _process(double delta) override;

    void set_publish_transform(bool enable);
    bool get_publish_transform() const;

    void set_publish_rate(double rate);
    double get_publish_rate() const;

    void publish_transform();
};

}
#endif

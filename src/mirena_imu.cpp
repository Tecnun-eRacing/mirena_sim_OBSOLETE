#include "mirena_imu.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MirenaImu::_bind_methods()
{

}

MirenaImu::MirenaImu()
{
}

MirenaImu::~MirenaImu()
{
}

//----------------------------------------------------------[Godot RUntime]----------------------------------------------//
void MirenaImu::_ros_ready()
{
    // Create Publishers
    posePub = ros_node->create_publisher<geometry_msgs::msg::PoseStamped>("car_pos", 10);
    speedPub = ros_node->create_publisher<geometry_msgs::msg::TwistStamped>("car_speed", 10);
    accelPub = ros_node->create_publisher<geometry_msgs::msg::AccelStamped>("car_accel", 10);
}

void MirenaImu::_ros_process(double delta)
{
    // Update Linear Measures
    l_pos = get_global_transform().origin;
    // Speed
    l_speed = (l_pos - l_prev_pos) / delta;
    // Accel
    l_accel = (l_speed - l_prev_speed) / delta;
    // Update prevs
    l_prev_pos = l_pos;
    l_prev_speed = l_speed;

    // Update Angular Measures
    a_pos = get_global_transform().basis.get_euler();
    aq_pos = get_global_transform().basis.get_quaternion();
    // Speed
    a_speed = (a_pos - a_prev_pos) / delta;
    // Accel
    a_accel = (a_speed - a_prev_speed) / delta;
    // Update prevs
    a_prev_pos = a_pos;
    a_prev_speed = a_speed;

    // Create msgs to send
    auto p_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    auto p_speed = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto p_accel = std::make_unique<geometry_msgs::msg::AccelStamped>();

    // Populate the frames
    // Pose
    p_pose->header.stamp = ros_node->now();
    p_pose->header.frame_id = "world";
    p_pose->pose.position.x = l_pos.x;
    p_pose->pose.position.y = l_pos.y;
    p_pose->pose.position.z = l_pos.z;
    p_pose->pose.orientation.x = aq_pos.x;
    p_pose->pose.orientation.y = aq_pos.y;
    p_pose->pose.orientation.z = aq_pos.z;
    p_pose->pose.orientation.w = aq_pos.w;

    // Speed
    p_speed->header.stamp = ros_node->now();
    p_speed->header.frame_id = ros_node->get_name();
    p_speed->twist.linear.x = l_speed.x;
    p_speed->twist.linear.y = l_speed.y;
    p_speed->twist.linear.z = l_speed.z;
    p_speed->twist.angular.x = a_speed.x;
    p_speed->twist.angular.y = a_speed.y;
    p_speed->twist.angular.z = a_speed.z;

    // Accel
    p_accel->header.stamp = ros_node->now();
    p_accel->header.frame_id = ros_node->get_name();
    p_accel->accel.linear.x = l_accel.x;
    p_accel->accel.linear.y = l_accel.y;
    p_accel->accel.linear.z = l_accel.z;
    p_accel->accel.angular.x = a_accel.x;
    p_accel->accel.angular.y = a_accel.y;
    p_accel->accel.angular.z = a_accel.z;

    // Publish all data
    posePub->publish(std::move(p_pose));
    speedPub->publish(std::move(p_speed));
    accelPub->publish(std::move(p_accel));
}
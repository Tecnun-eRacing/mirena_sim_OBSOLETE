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
    imuPub = ros_node->create_publisher<sensor_msgs::msg::Imu>("car_imu", 10);
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
    auto p_imu = std::make_unique<sensor_msgs::msg::Imu>();


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

    //IMU Data
    p_imu->header.stamp = ros_node->now();
    p_imu->header.frame_id = ros_node->get_name();

    //Orientation
    p_imu->orientation.x = aq_pos.x;
    p_imu->orientation.y = aq_pos.y;
    p_imu->orientation.z = aq_pos.z;
    p_imu->orientation.w = aq_pos.w;
    p_imu->orientation_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // 3x3 covariance matrix for orientation

    //Angular velocity
    p_imu->angular_velocity.x = a_speed.x;
    p_imu->angular_velocity.y = a_speed.y;
    p_imu->angular_velocity.z = a_speed.z;
    p_imu->angular_velocity_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // 3x3 covariance matrix for angular velocity


    // Linear Acceleration
    p_imu->linear_acceleration.x = l_accel.x;
    p_imu->linear_acceleration.y = l_accel.y;
    p_imu->linear_acceleration.z = l_accel.z;
    p_imu->linear_acceleration_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // 3x3 covariance matrix for linear acceleration

    // Publish all data
    posePub->publish(std::move(p_pose));
    imuPub->publish(std::move(p_imu));

}
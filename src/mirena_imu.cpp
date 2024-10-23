#include "mirena_imu.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MirenaImu::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_publish_rate", "p_rate"), &MirenaImu::set_publish_rate);
    ClassDB::bind_method(D_METHOD("get_publish_rate"), &MirenaImu::get_publish_rate);

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
}

MirenaImu::MirenaImu() : publish_rate(0.0), last_publish_time(0.0)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    node = rclcpp::Node::make_shared("MirenaImu");
    godot::UtilityFunctions::print("MirenaImu Created");
    //Create Publishers
    posePub = node->create_publisher<geometry_msgs::msg::PoseStamped>("car_pose", 10);
    speedPub = node->create_publisher<geometry_msgs::msg::TwistStamped>("car_speed", 10);
    accelPub = node->create_publisher<geometry_msgs::msg::AccelStamped>("car_accel", 10);
}

MirenaImu::~MirenaImu()
{
}

//----------------------------------------------------------[Godot RUntime]----------------------------------------------//
void MirenaImu::_process(double delta)
{
    last_publish_time += delta;
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
    // Speed
    a_speed = (a_pos - a_prev_pos) / delta;
    // Accel
    a_accel = (a_speed - a_prev_speed) / delta;
    // Update prevs
    a_prev_pos = a_pos;
    a_prev_speed = a_speed;

    if (last_publish_time >= 1.0 / publish_rate)
    {
        // Create msgs to send
        auto p_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
        auto p_speed = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto p_accel = std::make_unique<geometry_msgs::msg::AccelStamped>();

        // Populate the frames
        // Pose
        p_pose->header.stamp = node->now();
        p_pose->header.frame_id = "car_pose";
        p_pose->pose.position.x = l_pos.x;
        p_pose->pose.position.y = l_pos.y;
        p_pose->pose.position.z = l_pos.z;
        p_pose->pose.orientation.x = a_pos.x;
        p_pose->pose.orientation.y = a_pos.y;
        p_pose->pose.orientation.z = a_pos.z;

        // Speed
        p_speed->header.stamp = node->now();
        p_speed->header.frame_id = "car_speed";
        p_speed->twist.linear.x = l_speed.x;
        p_speed->twist.linear.y = l_speed.y;
        p_speed->twist.linear.z = l_speed.z;
        p_speed->twist.angular.x = a_speed.x;
        p_speed->twist.angular.y = a_speed.y;
        p_speed->twist.angular.z = a_speed.z;

        // Accel
        p_accel->header.stamp = node->now();
        p_accel->header.frame_id = "car_accel";
        p_accel->accel.linear.x = l_speed.x;
        p_accel->accel.linear.y = l_speed.y;
        p_accel->accel.linear.z = l_speed.z;
        p_accel->accel.angular.x = a_speed.x;
        p_accel->accel.angular.y = a_speed.y;
        p_accel->accel.angular.z = a_speed.z;

        // Publish all data
        posePub->publish(std::move(p_pose));
        speedPub->publish(std::move(p_speed));
        accelPub->publish(std::move(p_accel));

        rclcpp::spin_some(node);
        last_publish_time = 0.0;
    }
}

//----------------------------------------------------------[Godot Getters and setters]----------------------------------------------//

void MirenaImu::set_publish_rate(double p_rate) { publish_rate = p_rate; }
double MirenaImu::get_publish_rate() const { return publish_rate; }
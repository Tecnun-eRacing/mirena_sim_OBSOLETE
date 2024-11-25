// ros_node_3d.cpp
#include "ros_node3d.hpp"
#include <godot_cpp/core/class_db.hpp>

namespace godot
{

    RosNode3D::RosNode3D()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    RosNode3D::~RosNode3D()
    {
        if (ros_node)
        {
            // rclcpp::shutdown();
        }
    }

    void RosNode3D::_ready()
    {
        // Use object name for ROS node name
        String node_name = get_name();
        ros_node = std::make_shared<rclcpp::Node>(node_name.utf8().get_data());
        transform_publisher = ros_node->create_publisher<geometry_msgs::msg::TransformStamped>("/mirena_transforms", 10);
    }

    void RosNode3D::_process(double delta)
    {
        if (ros_node)
        {
            rclcpp::spin_some(ros_node);
        }

        if (publish_tf)
        {
            elapsed_time += delta;
            if (elapsed_time >= publish_rate)
            {
                publish_transform();
                elapsed_time = 0.0;
            }
        }
    }

    RosNode3D *RosNode3D::find_nearest_ros_parent()
    {
        Node *parent = get_parent();
        while (parent)
        {
            RosNode3D *ros_parent = Object::cast_to<RosNode3D>(parent);
            if (ros_parent)
            {
                return ros_parent;
            }
            parent = parent->get_parent();
        }
        return nullptr;
    }

    void RosNode3D::publish_transform()
    {
        if (!transform_publisher)
            return;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = ros_node->get_clock()->now();

        RosNode3D *parent_ros_node = find_nearest_ros_parent();
        if (parent_ros_node)
        {
            std::string parent_name(reinterpret_cast<const char *>(parent_ros_node->get_name().to_utf8_buffer().ptr()), parent_ros_node->get_name().to_utf8_buffer().size());
            transform.header.frame_id = parent_name;
        }
        else
        {
            transform.header.frame_id = "world";
        }
        std::string child_name(reinterpret_cast<const char *>(get_name().to_utf8_buffer().ptr()), get_name().to_utf8_buffer().size());
        transform.child_frame_id = child_name;

        // Get global transform
        Transform3D global_transform = get_global_transform();

        // Set translation
        transform.transform.translation.x = global_transform.origin.x;
        transform.transform.translation.y = global_transform.origin.y;
        transform.transform.translation.z = global_transform.origin.z;

        // Set rotation (as quaternion)
        Basis rotation = global_transform.basis;
        Quaternion quat = rotation.get_quaternion();
        transform.transform.rotation.x = quat.x;
        transform.transform.rotation.y = quat.y;
        transform.transform.rotation.z = quat.z;
        transform.transform.rotation.w = quat.w;

        transform_publisher->publish(transform);
    }

    void RosNode3D::_bind_methods()
    {
        ClassDB::bind_method(D_METHOD("set_publish_transform", "enable"), &RosNode3D::set_publish_transform);
        ClassDB::bind_method(D_METHOD("get_publish_transform"), &RosNode3D::get_publish_transform);

        ClassDB::bind_method(D_METHOD("set_publish_rate", "rate"), &RosNode3D::set_publish_rate);
        ClassDB::bind_method(D_METHOD("get_publish_rate"), &RosNode3D::get_publish_rate);
        // Add properties
        ADD_PROPERTY(PropertyInfo(Variant::BOOL, "publish_transform"), "set_publish_transform", "get_publish_transform");
        ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
    }

    void RosNode3D::set_publish_transform(bool enable)
    {
        publish_tf = enable;
    }

    bool RosNode3D::get_publish_transform() const
    {
        return publish_tf;
    }

    void RosNode3D::set_publish_rate(double rate)
    {
        publish_rate = rate;
    }

    double RosNode3D::get_publish_rate() const
    {
        return publish_rate;
    }

}
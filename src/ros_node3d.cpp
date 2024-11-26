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

    }

    void RosNode3D::_ready()
    {
        if (!Engine::get_singleton()->is_editor_hint())
        { // Comprobamos si estamos en el editor
            // Use object name for ROS node name
            String node_name = get_name();
            ros_node = std::make_shared<rclcpp::Node>(node_name.utf8().get_data());
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*ros_node);
            //Look for parent object

            _ros_ready();
        }
    }

    void RosNode3D::_ros_ready()
    {
    }

    void RosNode3D::_physics_process(double delta)
    {
        if (!Engine::get_singleton()->is_editor_hint())
        { // Comprobamos si estamos en el editor
          // Handle Transform publisher
            elapsed_time += delta;
            if (elapsed_time >= 1 / publish_rate)
            {
                current_time = ros_node->now();
                if (publish_tf)
                    publish_transform();
                _ros_process(delta); // call downstream processes
                elapsed_time = 0.0;
            }
            rclcpp::spin_some(ros_node);
        }
    }

    void RosNode3D::_ros_process(double delta)
    {
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
        if (!tf_broadcaster)
            return;

        geometry_msgs::msg::TransformStamped transform;

        // Get global transform of object
        Transform3D nodeTransform = get_global_transform();

        RosNode3D *parent_ros_node = find_nearest_ros_parent();
        if (parent_ros_node) // if parent ros2 node get corresponding relative transform between them
        {
            std::string parent_name(reinterpret_cast<const char *>(parent_ros_node->get_name().to_utf8_buffer().ptr()), parent_ros_node->get_name().to_utf8_buffer().size());
            transform.header.frame_id = parent_name;
            // Get global transform
            nodeTransform = parent_ros_node->get_global_transform().affine_inverse()*nodeTransform; // Compute relative transform
        }
        else
        {
            transform.header.frame_id = "world";
        }
        std::string child_name(reinterpret_cast<const char *>(get_name().to_utf8_buffer().ptr()), get_name().to_utf8_buffer().size());
        transform.child_frame_id = child_name;

        // Set translation
        transform.transform.translation.x = nodeTransform.origin.x;
        transform.transform.translation.y = nodeTransform.origin.y;
        transform.transform.translation.z = nodeTransform.origin.z;

        // Set rotation (as quaternion)
        Basis rotation = nodeTransform.basis;
        Quaternion quat = rotation.get_quaternion();
        transform.transform.rotation.x = quat.x;
        transform.transform.rotation.y = quat.y;
        transform.transform.rotation.z = quat.z;
        transform.transform.rotation.w = quat.w;

        transform.header.stamp = current_time;
        tf_broadcaster->sendTransform(transform);
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
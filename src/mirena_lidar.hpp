#ifndef MIRENA_LIDAR_H
#define MIRENA_LIDAR_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <omp.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace godot
{

    class MirenaLidar : public Node3D
    {
        GDCLASS(MirenaLidar, Node3D)

    private:
        // ROS publisher
        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;

        double max_range;
        int horizontal_resolution;
        int vertical_resolution;
        double vertical_fov;
        double horizontal_fov;
        uint32_t collision_mask;
        double noise_dev;
        //Rate
        double publish_rate;
        double last_publish_time;


    protected:
        static void _bind_methods();

    public:
        MirenaLidar();
        ~MirenaLidar();

		void _physics_process(double delta) override;



        //Getters and Setters
        void set_publish_rate(double p_rate);
        double get_publish_rate() const;

        void set_max_range(double p_range);
        double get_max_range() const;

        void set_noise_dev(double p_dev);
        double get_noise_dev() const;

        void set_horizontal_resolution(int p_resolution);
        int get_horizontal_resolution() const;

        void set_vertical_resolution(int p_resolution);
        int get_vertical_resolution() const;

        void set_vertical_fov(double p_fov);
        double get_vertical_fov() const;

        void set_horizontal_fov(double p_fov);
        double get_horizontal_fov() const;

        void set_collision_mask(uint32_t p_mask);
        uint32_t get_collision_mask() const;

        void scan();
    };

}

#endif // MIRENA_LIDAR
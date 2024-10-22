#ifndef MIRENACAM_H
#define MIRENACAM_H
// Godot
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/sub_viewport_container.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/core/class_db.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace godot
{

	class MirenaCam : public Node3D
	{
		GDCLASS(MirenaCam, Node3D)
	private:
		rclcpp::Node::SharedPtr node;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
		float publish_rate;
		double last_publish_time;

		SubViewport *viewport;
		Camera3D *camera;
		Vector2i resolution;
		bool use_environment;
		// Create ROS2 message
		std::unique_ptr<sensor_msgs::msg::Image> frame; 
		
		// Camera settings
		float fov;
		float near_clip;
		float far_clip;
		Camera3D::ProjectionType projection_type;
		float size_ortho;
		float h_offset;
		float v_offset;
		bool doppler_tracking;

	protected:
		static void _bind_methods();
		void _cleanup();
		void _update_camera_settings();

	public:
		MirenaCam();
		~MirenaCam();

		void _enter_tree() override;
		void _exit_tree() override;
		void _ready() override;
		void _process(double delta) override;

		// Configuration methods
		void set_publish_rate(float rate);
		float get_publish_rate() const;
		void set_resolution(Vector2i res);
		Vector2i get_resolution() const;
		void set_use_environment(bool enable);
		bool get_use_environment() const;

		// Camera control methods
		void set_camera_position(Vector3 position);
		Vector3 get_camera_position() const;
		void set_camera_rotation(Vector3 rotation);
		Vector3 get_camera_rotation() const;
		// Camera settings methods
		void set_fov(float p_fov);
		float get_fov() const;
		void set_near_clip(float p_near);
		float get_near_clip() const;
		void set_far_clip(float p_far);
		float get_far_clip() const;
		void set_projection_type(Camera3D::ProjectionType p_type);
		Camera3D::ProjectionType get_projection_type() const;
		void set_size_ortho(float p_size);
		float get_size_ortho() const;
		void set_h_offset(float p_offset);
		float get_h_offset() const;
		void set_v_offset(float p_offset);
		float get_v_offset() const;
		void set_doppler_tracking(bool p_enable);
		bool get_doppler_tracking() const;
	};

}

#endif
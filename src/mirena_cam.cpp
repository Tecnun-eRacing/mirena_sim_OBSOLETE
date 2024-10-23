#include "mirena_cam.hpp"
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
using namespace godot;

void MirenaCam::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_publish_rate", "rate"), &MirenaCam::set_publish_rate);
    ClassDB::bind_method(D_METHOD("get_publish_rate"), &MirenaCam::get_publish_rate);
    ClassDB::bind_method(D_METHOD("set_resolution", "resolution"), &MirenaCam::set_resolution);
    ClassDB::bind_method(D_METHOD("get_resolution"), &MirenaCam::get_resolution);
    ClassDB::bind_method(D_METHOD("set_use_environment", "enable"), &MirenaCam::set_use_environment);
    ClassDB::bind_method(D_METHOD("get_use_environment"), &MirenaCam::get_use_environment);
    ClassDB::bind_method(D_METHOD("set_camera_position", "position"), &MirenaCam::set_camera_position);
    ClassDB::bind_method(D_METHOD("get_camera_position"), &MirenaCam::get_camera_position);
    ClassDB::bind_method(D_METHOD("set_camera_rotation", "rotation"), &MirenaCam::set_camera_rotation);
    ClassDB::bind_method(D_METHOD("get_camera_rotation"), &MirenaCam::get_camera_rotation);

    // Camera settings bindings
    ClassDB::bind_method(D_METHOD("set_fov", "fov"), &MirenaCam::set_fov);
    ClassDB::bind_method(D_METHOD("get_fov"), &MirenaCam::get_fov);
    ClassDB::bind_method(D_METHOD("set_near_clip", "near"), &MirenaCam::set_near_clip);
    ClassDB::bind_method(D_METHOD("get_near_clip"), &MirenaCam::get_near_clip);
    ClassDB::bind_method(D_METHOD("set_far_clip", "far"), &MirenaCam::set_far_clip);
    ClassDB::bind_method(D_METHOD("get_far_clip"), &MirenaCam::get_far_clip);
    ClassDB::bind_method(D_METHOD("set_projection_type", "type"), &MirenaCam::set_projection_type);
    ClassDB::bind_method(D_METHOD("get_projection_type"), &MirenaCam::get_projection_type);
    ClassDB::bind_method(D_METHOD("set_size_ortho", "size"), &MirenaCam::set_size_ortho);
    ClassDB::bind_method(D_METHOD("get_size_ortho"), &MirenaCam::get_size_ortho);
    ClassDB::bind_method(D_METHOD("set_h_offset", "offset"), &MirenaCam::set_h_offset);
    ClassDB::bind_method(D_METHOD("get_h_offset"), &MirenaCam::get_h_offset);
    ClassDB::bind_method(D_METHOD("set_v_offset", "offset"), &MirenaCam::set_v_offset);
    ClassDB::bind_method(D_METHOD("get_v_offset"), &MirenaCam::get_v_offset);
    ClassDB::bind_method(D_METHOD("set_doppler_tracking", "enable"), &MirenaCam::set_doppler_tracking);
    ClassDB::bind_method(D_METHOD("get_doppler_tracking"), &MirenaCam::get_doppler_tracking);

    // Camera settings
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fov", PROPERTY_HINT_RANGE, "1,179,0.1"), "set_fov", "get_fov");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "near_clip", PROPERTY_HINT_RANGE, "0.001,10,0.001"), "set_near_clip", "get_near_clip");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "far_clip", PROPERTY_HINT_RANGE, "0.1,10000,0.1"), "set_far_clip", "get_far_clip");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "projection_type", PROPERTY_HINT_ENUM, "Perspective,Orthogonal"), "set_projection_type", "get_projection_type");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "size_ortho", PROPERTY_HINT_RANGE, "0.1,1000,0.1"), "set_size_ortho", "get_size_ortho");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "h_offset", PROPERTY_HINT_RANGE, "-10,10,0.01"), "set_h_offset", "get_h_offset");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "v_offset", PROPERTY_HINT_RANGE, "-10,10,0.01"), "set_v_offset", "get_v_offset");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "doppler_tracking"), "set_doppler_tracking", "get_doppler_tracking");
    // NODE
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2I, "resolution"), "set_resolution", "get_resolution");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_environment"), "set_use_environment", "get_use_environment");
}
MirenaCam::MirenaCam() : publish_rate(0),
                         last_publish_time(0.0),
                         resolution(Vector2i(640, 480)),
                         use_environment(true),
                         viewport(nullptr),
                         camera(nullptr), fov(75.0),
                         near_clip(0.05),
                         far_clip(1000.0),
                         projection_type(Camera3D::PROJECTION_PERSPECTIVE),
                         size_ortho(10.0),
                         h_offset(0.0),
                         v_offset(0.0),
                         doppler_tracking(false)
{

    // Initialize ROS2
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    node = rclcpp::Node::make_shared("godot_camera_node");
    image_publisher = node->create_publisher<sensor_msgs::msg::Image>("camera_frames", 10);
}

MirenaCam::~MirenaCam()
{
    _cleanup();
}

void MirenaCam::_cleanup()
{
    if (camera)
    {
        memdelete(camera);
        camera = nullptr;
    }
    if (viewport)
    {
        memdelete(viewport);
        viewport = nullptr;
    }
}

void MirenaCam::_enter_tree()
{
    Node3D::_enter_tree();
}

void MirenaCam::_exit_tree()
{
    _cleanup();
    Node3D::_exit_tree();
}

void MirenaCam::_ready()
{
    // Create viewport
    viewport = memnew(SubViewport);
    viewport->set_size(resolution);
    viewport->set_use_own_world_3d(false);
    viewport->set_clear_mode(SubViewport::CLEAR_MODE_ALWAYS);
    viewport->set_update_mode(SubViewport::UPDATE_ALWAYS);

    // Create camera
    camera = memnew(Camera3D);
    _update_camera_settings();
    viewport->add_child(camera);

    // If using environment, copy it from the main viewport
    if (use_environment)
    {
        Viewport *main_viewport = get_viewport();
        if (main_viewport && main_viewport->get_world_3d().is_valid())
        {
            viewport->set_world_3d(main_viewport->get_world_3d());
        }
    }

    add_child(viewport);
}

void MirenaCam::_process(double delta)
{
    last_publish_time += delta;
    if(camera)camera->set_global_transform(get_global_transform()); // Set camera position to node position
    if (last_publish_time >= 1.0 / publish_rate && viewport)
    {
        Ref<Image> img = viewport->get_texture()->get_image();
        if (img.is_null())
            return;

        frame = std::make_unique<sensor_msgs::msg::Image>();
        frame->header.stamp = node->now();
        frame->header.frame_id = "camera_frame";
        frame->height = img->get_height();
        frame->width = img->get_width();
        frame->encoding = "rgb8";
        frame->is_bigendian = false;
        frame->step = img->get_width() * 3;

        // Copy image data
        PackedByteArray data = img->get_data();
        frame->data.resize(data.size());
        std::memcpy(frame->data.data(), data.ptrw(), data.size());

        // Publish
        image_publisher->publish(std::move(frame));
        last_publish_time = 0.0;
    }

    // Process ROS2 callbacks
    rclcpp::spin_some(node);
}

void MirenaCam::_update_camera_settings()
{
    if (!camera)
        return;

    camera->set_fov(fov);
    camera->set_near(near_clip);
    camera->set_far(far_clip);
    camera->set_projection(projection_type);
    camera->set_size(size_ortho);
    camera->set_h_offset(h_offset);
    camera->set_v_offset(v_offset);
    camera->set_doppler_tracking(doppler_tracking ? Camera3D::DOPPLER_TRACKING_IDLE_STEP : Camera3D::DOPPLER_TRACKING_DISABLED);
}

void MirenaCam::set_publish_rate(float rate)
{
    publish_rate = rate;
}

float MirenaCam::get_publish_rate() const
{
    return publish_rate;
}

void MirenaCam::set_resolution(Vector2i res)
{
    resolution = res;
    if (viewport)
    {
        viewport->set_size(resolution);
    }
}

Vector2i MirenaCam::get_resolution() const
{
    return resolution;
}

void MirenaCam::set_use_environment(bool enable)
{
    use_environment = enable;
    if (viewport && enable)
    {
        Viewport *main_viewport = get_viewport();
        if (main_viewport && main_viewport->get_world_3d().is_valid())
        {
            viewport->set_world_3d(main_viewport->get_world_3d());
        }
    }
}

bool MirenaCam::get_use_environment() const
{
    return use_environment;
}

void MirenaCam::set_camera_position(Vector3 position)
{
    if (camera)
    {
        camera->set_position(position);
    }
}

Vector3 MirenaCam::get_camera_position() const
{
    return camera ? camera->get_position() : Vector3();
}

void MirenaCam::set_camera_rotation(Vector3 rotation)
{
    if (camera)
    {
        camera->set_rotation(rotation);
    }
}

Vector3 MirenaCam::get_camera_rotation() const
{
    return camera ? camera->get_rotation() : Vector3();
}

// Camera settings methods
void MirenaCam::set_fov(float p_fov)
{
    fov = p_fov;
    if (camera)
        camera->set_fov(fov);
}

float MirenaCam::get_fov() const
{
    return fov;
}

void MirenaCam::set_near_clip(float p_near)
{
    near_clip = p_near;
    if (camera)
        camera->set_near(near_clip);
}

float MirenaCam::get_near_clip() const
{
    return near_clip;
}

void MirenaCam::set_far_clip(float p_far)
{
    far_clip = p_far;
    if (camera)
        camera->set_far(far_clip);
}

float MirenaCam::get_far_clip() const
{
    return far_clip;
}

void MirenaCam::set_projection_type(Camera3D::ProjectionType p_type)
{
    projection_type = p_type;
    if (camera)
        camera->set_projection(projection_type);
}

Camera3D::ProjectionType MirenaCam::get_projection_type() const
{
    return projection_type;
}

void MirenaCam::set_size_ortho(float p_size)
{
    size_ortho = p_size;
    if (camera)
        camera->set_size(size_ortho);
}

float MirenaCam::get_size_ortho() const
{
    return size_ortho;
}

void MirenaCam::set_h_offset(float p_offset)
{
    h_offset = p_offset;
    if (camera)
        camera->set_h_offset(h_offset);
}

float MirenaCam::get_h_offset() const
{
    return h_offset;
}

void MirenaCam::set_v_offset(float p_offset)
{
    v_offset = p_offset;
    if (camera)
        camera->set_v_offset(v_offset);
}

float MirenaCam::get_v_offset() const
{
    return v_offset;
}

void MirenaCam::set_doppler_tracking(bool p_enable)
{
    doppler_tracking = p_enable;
    if (camera)
    {
        camera->set_doppler_tracking(doppler_tracking ? Camera3D::DOPPLER_TRACKING_IDLE_STEP : Camera3D::DOPPLER_TRACKING_DISABLED);
    }
}

bool MirenaCam::get_doppler_tracking() const
{
    return doppler_tracking;
}
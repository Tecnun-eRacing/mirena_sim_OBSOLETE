extends VehicleBody3D

const ENGINE_F = 900
const BRAKE_F = 20
const MAX_STEER = 30*3.1415/180

#Control Mode
enum ControlMode {ROS, STEERING_WHEEL}
var drive_mode = ControlMode.ROS

# Called when the node enters the scene tree for the first time.
func _ready():
	# Doing this ensures that all dumps are calculated righfully
	RenderingServer.frame_post_draw.connect(_on_post_render)

func _on_post_render():
	if Input.is_action_just_pressed("dump_yolo"):
		$MirenaCar/MirenaCam.dump_group_bbox_to_yolo("Cones")
	
	if Input.is_action_just_pressed("dump_keypoints"):
		$MirenaCar/MirenaCam.dump_group_keypoints("Cones")


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	#Handle all inputs
	
	if Input.is_action_just_pressed("driving_mode"):
		drive_mode = (drive_mode + 1) % 2  # Toggle between 3 modes
		match drive_mode:
			ControlMode.ROS:
				$UserCam.current = false		
			ControlMode.STEERING_WHEEL:
				$UserCam.current = true
			_:
				pass
	if Input.is_action_just_pressed("autofollow"):
		follow_path(get_node("/root/SimEnviroment/Track").path)

	match drive_mode:
				ControlMode.ROS:
					ros_drive()
				ControlMode.STEERING_WHEEL:
					manual_drive()
				_:
					pass
					
	$MirenaCar.set_wheels_speed($RL_WHEEL.get_rpm()*PI/30,$RR_WHEEL.get_rpm()*PI/30,$FL_WHEEL.get_rpm()*PI/30,$FL_WHEEL.get_rpm()*PI/30)
	
	
func ros_drive():
	steering = $MirenaCar.steer_angle
	engine_force = $MirenaCar.gas*ENGINE_F/255
	brake = $MirenaCar.brake * BRAKE_F/255

func manual_drive():
	var steering_input = -Input.get_joy_axis(0, 2)
	var accelerator_input = Input.get_action_strength("accel")
	var brake_input = Input.get_action_strength("brake")

	# Move Car
	steering = MAX_STEER * steering_input
	engine_force = ENGINE_F * accelerator_input;
	brake = BRAKE_F * brake_input	
	
	
func follow_path(path: Path3D, speed: float = 10) -> void:
	if not path :
		push_warning("Missing path reference!")
		return
	# Create local PathFollow3D node
	var path_follow = PathFollow3D.new()
	path.add_child(path_follow)
	path_follow.loop = true
	# Stop the vehicle and disable physics temporarily
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	engine_force = 0
	brake = 1.0
	freeze = true # Freeze physics
	
	# Reset progress
	path_follow.progress = 0.0
	# Follow the path until completion
	while not is_equal_approx(path_follow.progress_ratio, 1.0):
		await get_tree().process_frame
		# Move forward along the path
		path_follow.progress += speed * get_process_delta_time()
		path_follow.progress_ratio = clamp(path_follow.progress_ratio, 0.0, 1.0)
		# Update vehicle position and orientation
		global_position = path_follow.global_position
		# Adjust orientation to follow the path's direction
		var path_direction = path_follow.transform.basis.z
		global_transform = global_transform.looking_at(
		global_position + path_direction, Vector3.UP)
	# Re-enable physics once done
	freeze = false
	await get_tree().process_frame
	brake = 0.0
	print("Finished Touring the Path")
	path_follow.queue_free()


func set_pose(pos : Vector3, theta : float) -> void:
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	#Update transform
	set_deferred("global_transform", Transform3D(Basis(Vector3.UP, theta), pos))

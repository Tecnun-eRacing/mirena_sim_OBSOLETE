extends VehicleBody3D

const ENGINE_F = 900
const BRAKE_F = 20
const MAX_STEER = 30*3.1415/180

#Control Mode
enum ControlMode {ROS, STEERING_WHEEL}
var drive_mode = ControlMode.ROS

#Physics
var previous_linear_velocity: Vector3 = Vector3.ZERO
var previous_angular_velocity: Vector3 = Vector3.ZERO

var linear_acceleration: Vector3 = Vector3.ZERO
var angular_acceleration: Vector3 = Vector3.ZERO

# Broadcast
var car_broadcast_accumulator: float = 0
var car_broadcast_period: float = 0.1 # seconds

# Position Wrap
var do_pos_wraping: bool = true;
var x_limits: Vector2 = Vector2i(-20, 20)
var z_limits: Vector2 = Vector2i(-45, 45)

# Called when the node enters the scene tree for the first time.
func _ready():
	# Configure the popup panel
	var screen_size = get_viewport().get_visible_rect().size
	$CarOptionsMenu.size = Vector2i(400, 400)
	$CarOptionsMenu.position = Vector2(
	10, # left edge
	(screen_size.y - $CarOptionsMenu.size.y) / 2 # vertical center
	)
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
		
	if Input.is_action_just_pressed("open_car_menu"):
		$CarOptionsMenu.popup()
		
	########### CAR POSITION WRAPPING #############
	if (do_pos_wraping and is_outside_boundaries()):
		print("IS outside", self.position)
		var new_pos = self.position
		var x_range = x_limits.y - x_limits.x
		var z_range = z_limits.y - z_limits.x

		new_pos.x = wrap_float(new_pos.x, x_limits.x, x_limits.y)
		new_pos.z = wrap_float(new_pos.z, z_limits.x, z_limits.y)
		
		self.set_pose(new_pos, self.rotation.y, false)

	match drive_mode:
				ControlMode.ROS:
					ros_drive()
				ControlMode.STEERING_WHEEL:
					manual_drive()
				_:
					pass
					
	$MirenaCar.set_wheels_speed($RL_WHEEL.get_rpm()*PI/30,$RR_WHEEL.get_rpm()*PI/30,$FL_WHEEL.get_rpm()*PI/30,$FL_WHEEL.get_rpm()*PI/30)

func _physics_process(delta: float) -> void:
	########### ACCEL CALCULATION #############
	# Calculate linear acceleration
	linear_acceleration = (linear_velocity - previous_linear_velocity) / delta
	previous_linear_velocity = linear_velocity

	# Calculate angular acceleration
	angular_acceleration = (angular_velocity - previous_angular_velocity) / delta
	previous_angular_velocity = angular_velocity
	
	########### STATE BROADCASTING #############	
	# If enough time has passed, broadcast the message
	self.car_broadcast_accumulator += delta
	if self.car_broadcast_accumulator >= self.car_broadcast_period:
		self.car_broadcast_accumulator = fmod(self.car_broadcast_accumulator, self.car_broadcast_period)
		$MirenaCar.ros_broadcast_car_state(position, rotation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration)
		
		
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


func set_pose(pos : Vector3, theta : float = 0, reset_vel: bool = false) -> void:
	if reset_vel:
		linear_velocity = Vector3.ZERO
		angular_velocity = Vector3.ZERO
	await get_tree().process_frame #Let the phisics state propagate
	#Update transform
	set_deferred("global_transform", Transform3D(Basis(Vector3.UP, theta), pos))
	set_deferred("global_transform", Transform3D(Basis(Vector3.UP, theta), pos))

func reset_car() -> void:
	set_pose(Vector3(0, 1, 0), 0, true)

func is_outside_boundaries() -> bool:
	return self.position.x < self.x_limits.x or self.position.x > self.x_limits.y or self.position.z < self.z_limits.x or self.position.z > self.z_limits.y

func toggle_do_position_wrapping() -> void:
	do_pos_wraping = !do_pos_wraping;
	
func set_do_position_wrapping(value: bool) -> void:
	do_pos_wraping = value

func get_do_position_wrapping() -> bool:
	return do_pos_wraping

func wrap_float(value: float, min_value: float, max_value: float) -> float:
	var range = max_value - min_value
	var result = fmod(value - min_value, range)
	if result < 0:
		result += range
	return result + min_value

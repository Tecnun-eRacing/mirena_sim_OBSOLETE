extends VehicleBody3D

const ENGINE_F = 900
const BRAKE_F = 20
const MAX_STEER = 30*3.1415/180

#Control Mode
enum ControlMode {ROS, STEERING_WHEEL}
var drive_mode = ControlMode.ROS

# Called when the node enters the scene tree for the first time.
func _ready():
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if Input.is_action_just_pressed("driving_mode"):
		drive_mode = (drive_mode + 1) % 2  # Toggle between 3 modes
		match drive_mode:
			ControlMode.ROS:
				$UserCam.current = false		
			ControlMode.STEERING_WHEEL:
				$UserCam.current = true
			_:
				pass

	if Input.is_action_just_pressed("dump_yolo") or Input.is_joy_button_pressed(0, 0):
		$MirenaCar/MirenaCam.dump_group_bbox_to_yolo("Cones")
		
	if Input.is_action_just_pressed("dump_keypoints") or Input.is_joy_button_pressed(0, 0):
		$MirenaCar/MirenaCam.dump_group_keypoints("Cones")
		follow_path(get_node("/root/SimEnviroment/Track").path,self)

		
	match drive_mode:
				ControlMode.ROS:
					ros_drive()
				ControlMode.STEERING_WHEEL:
					manual_drive()
				_:
					pass
		
		
func ros_drive():
	steering = $MirenaCar.steer_angle
	engine_force = $MirenaCar.gas*ENGINE_F/255
	brake = $MirenaCar.brake * BRAKE_F/255

func manual_drive():
	var steering_input = -Input.get_joy_axis(0, 2)
	var accelerator_input = Input.get_joy_axis(0, 1)
	var brake_input = 0
	if Input.is_joy_button_pressed(0, 1):
		brake_input = 1
	else:
		brake_input = 0
	# Move Car
	steering = MAX_STEER * steering_input
	engine_force = ENGINE_F * accelerator_input;
	brake = BRAKE_F * brake_input	

func follow_path(path: Path3D, car: Node3D, speed: float = 5.0) -> void:
	if not path or not car:
		push_warning("Missing path or car reference!")
		return
		
	# Create local PathFollow3D node
	var path_follow = PathFollow3D.new()
	path.add_child(path_follow)
	path_follow.loop = true
	self.freeze = true # Congelamos las fisicas
	while not is_equal_approx(path_follow.progress_ratio, 1.0):
		path_follow.progress -= speed * get_process_delta_time()
		path_follow.progress_ratio = clamp(path_follow.progress_ratio, 0.0, 1.0)

		car.global_position = path_follow.global_position
		car.global_transform = path_follow.global_transform
		await get_tree().process_frame
	self.freeze = false # DesCongelamos las fisicas
	print("Finished Touring the Path")
	path_follow.queue_free()
	

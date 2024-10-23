extends VehicleBody3D

const ENGINE_F = 900

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	steering = $MirenaCar.steer_angle
	engine_force = $MirenaCar.gas*ENGINE_F/255
	$MirenaLidar.publish_rate = 5.0
	$MirenaCam.publish_rate = 30.0

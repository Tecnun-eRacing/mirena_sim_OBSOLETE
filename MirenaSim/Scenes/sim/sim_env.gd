extends Node

#Eliminates all members of the group cones
func clearCones():
	for cone in get_tree().get_nodes_in_group("Cones"):
		cone.queue_free() # Delete each element

func loadTrack(path : String):
	clearCones() # Clears existing track
	var file = FileAccess.open(path, FileAccess.READ)
	var data = file.get_as_text()
	var json = JSON.new()
	json.parse(data)
	var track = json.data
	for cone in track["cones"]:
		var instance = load("res://Scenes/cones/cone.tscn").instantiate()
		match cone["type"]:
			"blue":
				instance.set_meta("type","blue")
				instance.set_type(instance.color.BLUE)
			"yellow":
				instance.set_meta("type","yellow")
				instance.set_type(instance.color.YELLOW)
		instance.translate(Vector3(cone["x"],0,cone["y"]))
		$Track.add_child(instance)
		instance.add_to_group("Cones")
		# Move car to spawnPos
	print("Spawn Pos",Vector3(track["spawn"]["x"],0,track["spawn"]["y"]))
	#$vehicle.freeze = true
	#await get_tree().process_frame # Hacemos que el freeze tenga efecto
	#$vehicle.translate(Vector3(track["spawn"]["x"],0,track["spawn"]["y"]))
	#$vehicle.freeze = false
	PhysicsServer3D.body_set_state(
	$vehicle.get_rid(),
	PhysicsServer3D.BODY_STATE_TRANSFORM,
	Transform3D.IDENTITY.rotated(Vector3.UP, track["spawn"]["theta"])
	.translated(Vector3(track["spawn"]["x"],0,track["spawn"]["y"]))	
	)
	print("Loaded track")


	
func _on_file_selected(path: String) -> void:
	print("Selected track path: ", path)
	loadTrack(path)
	
	
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	# Check if the "m" key is pressed
	if Input.is_action_just_pressed("load_track"):
		# Show the FileDialog
		$FileDialog.popup_centered()
	

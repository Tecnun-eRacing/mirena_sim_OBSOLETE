extends Node3D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	 # Check if the "m" key is pressed
	if Input.is_action_just_pressed("load_track"):
		# Show the FileDialog
		$FileDialog.popup_centered()
	pass

#Eliminates all members of the group cones
func clearCones():
	for cone in get_tree().get_nodes_in_group("Cones"):
		cone.queue_free() # Delete each element
		
func loadCones(path : String):
	clearCones() # Clears existing track
	var file = FileAccess.open(path, FileAccess.READ)
	var data = file.get_as_text()
	var json = JSON.new()
	json.parse(data)
	var track = json.data
	for cone in track["cones"]:
		var model
		match cone["type"]:
			"blue":
				model = load("res://Models/Cone/bCone.glb").instantiate() 
			"yellow":
				model = load("res://Models/Cone/yCone.glb").instantiate() 
		model.translate(Vector3(cone["x"],0,cone["y"]))
		add_child(model)
		model.add_to_group("Cones")


func _on_file_selected(path: String) -> void:
	print("Selected track path: ", path)
	loadCones(path)
	

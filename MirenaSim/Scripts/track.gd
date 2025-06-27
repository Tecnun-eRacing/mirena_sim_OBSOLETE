extends Node3D
class_name TrackManager

## Exposes the Curve3D for the path
var path_curve : Curve3D = Curve3D.new()
##Exposes Followable Path for the track
var car_path : Path3D = Path3D.new()

## Clears all track contents
func clearTrack():
	for cone in get_tree().get_nodes_in_group("Cones"):
		cone.queue_free() # Delete each element


func showPath():
	#Visualize path
	var line_mesh := ImmediateMesh.new()
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.mesh = line_mesh
	add_child(mesh_instance)
	# Draw the curve as a line strip
	line_mesh.clear_surfaces()
	line_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)

	for i in range(path_curve.get_point_count()):
		line_mesh.surface_add_vertex(path_curve.get_point_position(i))
	line_mesh.surface_end()
	
	
##Loads curve from a json file
func loadTrack(filepath : String):
	#Get json trackfile
	print("Loading Track:" + filepath)
	var file = FileAccess.open(filepath, FileAccess.READ)
	var data = file.get_as_text()
	var json = JSON.new()
	json.parse(data)
	var track = json.data
	#Load the curve into memory
	for point in track["path"]:
		path_curve.add_point(Vector3(point[0], 0, point[1]))
	#Load the path into track
	car_path.curve = path_curve
	add_child(car_path) #Expose path as child
	#Show the path curve
	showPath()
	#Spawn the cones
	genGates(path_curve,4,3)


## Generates the cones along a path with given separation and spacing
func genGates(path : Curve3D, spacing : float = 4, width : float = 3):
	# Preload Cone Scene
	var coneScene = load("res://Scenes/cones/cone.tscn") 
	#Calc Dimensions
	var length = path.get_baked_length() #Gets length
	var num_gates = int(length / spacing) # Calculates number of gates
	# Generate Startline and place car
	var start_pos = path.sample_baked(0)
	var first_pos = path.sample_baked(0 + spacing)
	var start_tan = (first_pos-start_pos).normalized()
	var start_normal = Vector3.UP.cross(start_tan).normalized()
	
	#Generate the two orange cones per side
	for i in range(4):
		var start_cone = coneScene.instantiate()
		start_cone.set_meta("type","blue")
		start_cone.set_type(start_cone.color.ORANGE)
		match i:
			0:
				start_cone.translate(start_pos + start_normal * (width / 2) + 0.3*start_tan)
			1: 
				start_cone.translate(start_pos + start_normal * (width / 2)- 0.3*start_tan)
			2:
				start_cone.translate(start_pos - start_normal * (width / 2)+ 0.3*start_tan)
			3:
				start_cone.translate(start_pos - start_normal * (width / 2)- 0.3*start_tan)
		start_cone.rotate_y(randf_range(0,PI/4))
		start_cone.add_to_group("Cones") # Add to group for easy reference
		add_child(start_cone)
	# Set car position
	# SIM.get_vehicle().set_pose(start_pos, start_theta)
	
	#Generate each gate
	for i in range(1,num_gates + 1):
		var  d = (i * spacing) # Obtiene la distancia de la puerta
		var pos = path.sample_baked(d)
		var nextPos = path.sample_baked(d + spacing)
		var tangent = (nextPos-pos).normalized()
		var normal = Vector3.UP.cross(tangent).normalized() # Get perpendicular Vector

		#Blue 
		var cone = coneScene.instantiate()
		cone.name = "G" + str(i) + "B"
		cone.set_meta("type","blue")
		cone.set_type(cone.color.BLUE)
		cone.translate(pos + normal * (width / 2))
		cone.rotate_y(randf_range(0,PI/4))
		cone.add_to_group("Cones") # Add to group for easy reference
		add_child(cone)

		#Yellow
		cone = coneScene.instantiate()
		cone.name = "G" + str(i) + "Y"
		cone.set_meta("type","yellow")
		cone.set_type(cone.color.YELLOW)
		cone.translate(pos - normal * (width / 2))
		cone.rotate_y(randf_range(0,PI/4))
		cone.add_to_group("Cones") # Add to group for easy reference
		add_child(cone)

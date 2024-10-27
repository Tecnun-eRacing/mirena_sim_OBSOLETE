extends Node3D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	spawnCones()
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func spawnCones():
	for i in range(50):
		# Create a line of cones
		var dCenter = Vector3(i,0,0)
		var bCone : Node3D = load("res://Models/Cone/bCone.glb").instantiate() 
		var yCone : Node3D = load("res://Models/Cone/yCone.glb").instantiate()
		bCone.translate(dCenter + Vector3(0,0.1,-1.5))
		yCone.translate(dCenter + Vector3(0,0.1,+1.5))
		add_child(bCone)
		add_child(yCone)

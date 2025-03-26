extends Node3D



enum color {BIG_ORANGE ,ORANGE, YELLOW, BLUE}
var type = color.BLUE
var moved = false # Holds if cone is intact

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func set_type(_type : color ):
		type = _type
		match type:
			color.BLUE:
				$Model.mesh = load("res://Models/Cone/Meshes/BCone.res")
			color.YELLOW:
				$Model.mesh = load("res://Models/Cone/Meshes/YCone.res")
			color.ORANGE:
				$Model.mesh = load("res://Models/Cone/Meshes/OCone.res")
	


func _on_body_entered(body: Node) -> void:
	if body.name == "vehicle" and not moved:
		moved = true # Cone already moved
		Debug.fallen_cones+=1
		#print("Cone Pushed")

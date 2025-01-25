extends Node3D

enum c_type {BIG_ORANGE ,ORANGE, YELLOW, BLUE}
var type = c_type.BLUE


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass



func set_type(_type : c_type ):
		type = _type
		match type:
			c_type.BLUE:
				$Model.mesh = load("res://Models/Cone/Meshes/BCone.res")
			c_type.YELLOW:
				$Model.mesh = load("res://Models/Cone/Meshes/YCone.res")
			c_type.ORANGE:
				$Model.mesh = load("res://Models/Cone/Meshes/YCone.res")
	

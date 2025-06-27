extends Node

var _current_scene: SimEnviroment

var _vehicle: MirenaCar
var _hud: MirenaHud

var _stats: Dictionary = {
	"cones_fallen": 0
}

func _ready() -> void:
	var vehicle_scene = preload("res://Scenes/vehicle/mirena_car.tscn")
	self._vehicle = vehicle_scene.instantiate()
	var hud_scene = preload("res://UserInterface/hud/mirena_hud.tscn")
	self._hud = hud_scene.instantiate()
	
	self._start_sim()

func _start_sim() -> void:
	self._current_scene = get_tree().current_scene
	self._current_scene.add_child(self._vehicle)
	self._current_scene.add_child(self._hud)

# -------------------------------------------------
# Interface
# -------------------------------------------------

func get_vehicle() -> MirenaCar:
	return self._vehicle

func get_hud() -> MirenaHud:
	return self._hud

func get_stats() -> Dictionary:
	return self._stats

func get_env() -> SimEnviroment:
	return self._current_scene

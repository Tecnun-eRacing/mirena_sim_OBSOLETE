extends CanvasLayer

var fps_label: Label
var fallen_cones_label: Label
var timer_label: Label

func _ready():
	# Get the Label nodes
	fps_label = $FpsLabel
	fallen_cones_label = $FallenConesLabel
	timer_label = $TimerLabel
	

func _process(delta):
	# Update FPS label
	fps_label.text = "FPS: %d" % Engine.get_frames_per_second()
	# Update Timer label
	Debug.timer += delta
	timer_label.text = "Timer: %.2f" % Debug.timer

	# Update Fallen Cones label
	fallen_cones_label.text = "Fallen Cones: %d" % Debug.fallen_cones

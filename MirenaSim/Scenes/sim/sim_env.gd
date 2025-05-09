extends Node

var arguments = {}
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	#Process user messages
	for argument in OS.get_cmdline_user_args():
		if argument.contains(" "):
			var key_value = argument.split(" s")
			arguments[key_value[0].trim_prefix("--")] = key_value[1]
		else:
			# Options without an argument will be present in the dictionary,
			# with the value set to an empty string.
			arguments[argument.trim_prefix("--")] = ""
	# Handle custom track load
	if arguments.has("track"):
		$Track.loadTrack(arguments["track"])
	if arguments.has("follow"):
		$vehicle.follow_path($Track.path)
	

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
	

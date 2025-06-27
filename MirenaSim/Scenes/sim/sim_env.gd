extends Node
class_name SimEnviroment

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	self._parse_arguments()

func get_track_manager() -> TrackManager:
	return $TrackManager

func _parse_arguments() -> void:
	var arguments = {}
	#Process user messages
	for argument in OS.get_cmdline_user_args():
		if argument.contains("="):
			var key_value = argument.split("=")
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

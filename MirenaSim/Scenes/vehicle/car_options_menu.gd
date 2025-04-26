extends PopupPanel

var margin = MarginContainer.new()
var main_panel = PanelContainer.new()

# Basic frame
var basic_controls = VBoxContainer.new()

# Basic frame / Compoents
var butt_reset_car: Button = Button.new()
var togg_wrap_car: CheckBox = CheckBox.new()

func _ready() -> void:
	self.connect("about_to_popup", self.on_popup)
	
	margin.add_theme_constant_override("margin_left", 20)
	margin.add_theme_constant_override("margin_top", 20)
	margin.add_theme_constant_override("margin_right", 20)
	margin.add_theme_constant_override("margin_bottom", 20)
	self.add_child(margin)
	
	margin.add_child(main_panel)
	
	basic_controls.add_theme_constant_override("separation", 20)
	main_panel.add_child(basic_controls)
	
	butt_reset_car.connect("pressed", self.on_butt_reset_car)
	butt_reset_car.text = "Reset Car"
	basic_controls.add_child(butt_reset_car)

	togg_wrap_car = CheckBox.new()
	togg_wrap_car.text = "Do Position Wrapping"
	togg_wrap_car.connect("toggled", self.on_togg_wrap_car)
	basic_controls.add_child(togg_wrap_car)

func on_popup() -> void:
	# Update the state
	togg_wrap_car.set_pressed_no_signal(self.get_owner_car().get_do_position_wrapping())

func on_butt_reset_car():
	self.get_owner_car().reset_car()
	
func on_togg_wrap_car(pressed):
	self.get_owner_car().set_do_position_wrapping(pressed)
	
func get_owner_car() -> VehicleBody3D:
	return self.get_parent() as VehicleBody3D
	

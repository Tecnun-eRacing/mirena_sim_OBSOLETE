extends CheckButton

func _ready() -> void:
	self.toggled.connect(SIM.get_vehicle().set_do_position_wrapping)
	self.toggled.connect(func(_x): self.update_state())

func _notification(what: int) -> void:
	if what == NOTIFICATION_VISIBILITY_CHANGED:
		self.update_state()

func update_state() -> void:
	self.set_pressed_no_signal(SIM.get_vehicle().get_do_position_wrapping())

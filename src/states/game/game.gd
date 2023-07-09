extends Node


enum State {WARMUP, PLAY, FINISH}

var state: State
var ticks: int

var num_players := 1
var players: Array[Dictionary] = []

var time_limit_ticks: int


func _ready() -> void:
	var level_scene := load(Global.level_path) as PackedScene
	var level := level_scene.instantiate() as Level
	
	time_limit_ticks = level.time_limit * Engine.physics_ticks_per_second

	var start_pad: Node3D
	start_pad = level.find_child("StartPad")

	for i in range(num_players):
		var viewport_texture: TextureRect = preload("res://src/states/game/game_viewport.tscn").instantiate()
		$Viewports.add_child(viewport_texture)
		var viewport: SubViewport = viewport_texture.get_node("SubViewport")
		if i == 0:
			viewport.add_child(level)
		else:
			viewport.world_3d = players[0].viewport.world_3d

		var marble: RigidBody3D = preload("res://src/objects/marble.tscn").instantiate()
		marble.position = start_pad.position + Vector3.UP * 10
		if num_players > 1:
			var offset_inc := TAU / num_players
			var offset := Vector3(1.25 * cos(i * offset_inc), 0, 1.25 * sin(i * offset_inc))
			offset = offset.rotated(Vector3.UP, -PI + start_pad.rotation.y)
			marble.position += offset
		marble.ballcam_yaw = start_pad.rotation.y + PI
		marble.desired_yaw = start_pad.rotation.y + PI
		marble.connect("level_finished", self._on_marble_level_finished)
		marble.SizeThresholds = level.SizeThresholds
		#marble.freeze = true
		level.add_child(marble)

		if num_players > 1:
			var controller := PlayerController.new()
			controller.uses_all_devices = false
			controller.which_device = i - 1

			marble.set_player_controller(controller)

		var camera: Camera3D = viewport.get_node("Camera3D")
		marble.find_child("CameraRemoteTransform").remote_path = camera.get_path()
		camera.fov = 50
		camera.near = 1
		camera.far = 10000
		#camera.fov += 20 * (num_players - 1)

		var gui: Gui = viewport_texture.get_node("Gui")
		gui.update_ball_goal(level.SizeThresholds[level.VictoryThreshold])
		marble.VictoryThreshold = level.VictoryThreshold

		var player_dict := {
			"marble": marble,
			"viewport": viewport,
			"camera": camera,
			"gui": gui,
		}
		players.append(player_dict)

	ticks = 0
	for player in players:
		player.marble.gui = player.gui
		player.gui.update_time_display(ticks, time_limit_ticks)

	state = State.PLAY
	if level.has_node("Camera3D"):
		var newCam = level.get_node("Camera3D") as Camera3D
		newCam.make_current()


func _physics_process(_delta: float) -> void:
	if state == State.PLAY:
		ticks += 1
		for player in players:
			player.gui.update_time_display(ticks, time_limit_ticks)


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("pause"):
		get_viewport().set_input_as_handled()
		%PauseMenu.show_pause_menu()
	elif event.is_action_pressed("restart") or event.is_action_pressed("restart_controller"):
		get_viewport().set_input_as_handled()
		get_tree().reload_current_scene()


func _on_marble_level_finished() -> void:
	state = State.FINISH

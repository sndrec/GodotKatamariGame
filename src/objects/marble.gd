extends RigidBody3D


signal gem_collected
signal level_finished

const VOLUMESCALE := 0.66

var player_controller: PlayerController = null

var ballcam_pitch := deg_to_rad(-10.0)
var ballcam_yaw := 0.0
var old_ballcam_yaw := 0.0
var desired_yaw := 0.0
var desired_arm_dist := 3.0
var ratio_to_next_transition := 0.0

var just_transitioned := false

var forward_timer := 0
var recent_forward_vector := Vector2.ZERO

var gui

var ballvolume := 1.0
var startingvolume := 1.0

var CurThreshold := 0.0
var NextThreshold := 0.0
var FinalThreshold := false

var jumping := false

var is_level_finished := false

var oldVel := Vector3.ZERO
var oldAngularVel := Vector3.ZERO
var oldPos := Vector3(0, 2, 0)
var oldBasis := Basis.IDENTITY

var desiredVel := 0.0

var LastGroundNormal := Vector3.UP
var LastGroundContact := Vector3(-5, 4.015605, 16.34)
var LastGroundVelocity := Vector3.ZERO
var AirTime := 0.0

var anyPoint := false
var furthestCollidingPoint := 0.0
var furthestDist := 0.0

var LastEffectiveVelocity := Vector3(0,0,0)

var OnStick := false

var climbTimer := 0.0
var noClimbTimer := 0.0

var gachaCount := 0
var gachaTimer := 0
var gachaLastDir := 0
var oldSteering := 0.0

var boostPower := 240.0
var boostHuffTime := -8000
var boostHuffing := false

var KatamariHullPointData = []

var cameraPos := Vector3(0, 0, 0)

var CollectSounds := [preload("res://src/sound/pickup/game33.wav"), preload("res://src/sound/pickup/game34.wav"), preload("res://src/sound/pickup/game35.wav")]
var VaultSound := preload("res://src/sound/game15.wav")

var ChargeSound := preload("res://src/sound/game11.wav")
var BoostSound := preload("res://src/sound/game12.wav")

var QuickSwitchSound := preload("res://src/sound/game16.wav")

var leftStick := false
var rightStick := false
var quickSwitchTime := -8000

var charState := "Wait"


var SoftCollideSound = preload("res://src/sound/game14.wav")

@onready var center_node: Node3D = $CenterNode
@onready var spring_arm: SpringArm3D = $CenterNode/SpringArm3D


func _ready() -> void:
	center_node.top_level = true
	center_node.position = position + Vector3(0, 0.4, 0)
	cameraPos = center_node.global_position
	center_node.rotation = Vector3.ZERO
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	if player_controller == null:
		set_player_controller(PlayerController.new())
	
	await get_tree().create_timer(0.1).timeout
	set_katamari_diameter(1.0)
	recalculate_katamari_size()
	#instantiate_katamari_hull()
	desired_arm_dist = get_katamari_diameter() * 3.0
	


static func angle_to_angle(from: float, to: float) -> float:
	return fposmod(to-from + PI, PI*2) - PI
	
func volume_to_radius(volume: float) -> float:
	return pow(6.0 * volume / PI, 1.0 / 3.0) * 0.5
	
func radius_to_volume(radius: float) -> float:
	return (4.0/3.0) * PI * pow(radius, 3)

func set_katamari_diameter(diameter: float):
	ballvolume = radius_to_volume(diameter * 0.5)
	startingvolume = radius_to_volume(diameter * 0.5)
	recalculate_katamari_size()
	
func get_katamari_diameter() -> float:
	return volume_to_radius(ballvolume) * 2.0

func get_katamari_radius() -> float:
	return volume_to_radius(ballvolume)

func recalculate_katamari_size() -> void:
	var newRadius := volume_to_radius(ballvolume)
	var Thresholds := get_meta("SizeThresholds") as PackedFloat32Array
	var OldThreshold := CurThreshold
	for i in range(0, Thresholds.size()): 
		if get_katamari_diameter() >= Thresholds[i]:
			if i == Thresholds.size():
				CurThreshold = Thresholds[i]
				FinalThreshold = true
			else:
				CurThreshold = Thresholds[i]
				NextThreshold = Thresholds[i + 1]
		else:
			break
	
	if OldThreshold != CurThreshold:
		just_transitioned = true
	
	ratio_to_next_transition = (get_katamari_diameter() - CurThreshold) / (NextThreshold - CurThreshold)
	if just_transitioned:
		desired_arm_dist = get_katamari_diameter() * 3.0
		just_transitioned = false
	else:
		desired_arm_dist = CurThreshold * 3.0 + (get_katamari_diameter() - CurThreshold)
	
	var BallCollision := get_node("KatamariColliderShape") as CollisionShape3D
	var BallCollisionShape := BallCollision.shape as SphereShape3D
	BallCollisionShape.radius = newRadius
	var BallCollisionBackup := get_node("CollisionBackup") as ShapeCast3D
	BallCollisionBackup.shape = BallCollision.shape
	var NocolliderCollision := get_node("CollectibleNocollider").get_node("NocolliderShape") as CollisionShape3D
	var NocolliderCollisionShape := NocolliderCollision.shape as SphereShape3D
	NocolliderCollisionShape.radius = newRadius * 3
	var GrabberCollision := get_node("CollectibleGrabber").get_node("KatamariCollectorShapeMiddle") as CollisionShape3D
	var GrabberCollisionShape := GrabberCollision.shape as SphereShape3D
	GrabberCollisionShape.radius = newRadius * 1
	var BottomGrabberCollision := get_node("CollectibleGrabber").get_node("KatamariCollectorShapeBottom") as CollisionShape3D
	BottomGrabberCollision.position = Vector3(0, -newRadius * 0.8, 0)
	var BottomGrabberCollisionShape := BottomGrabberCollision.shape as SphereShape3D
	BottomGrabberCollisionShape.radius = newRadius * 0.2
	var KatamariModel := get_node("KatamariModel") as Node3D
	var visualRadius := volume_to_radius(startingvolume + ((ballvolume - startingvolume) * 0.1))
	var NewScale := visualRadius * 2.7
	KatamariModel.scale = Vector3(NewScale, NewScale, NewScale)
	if gui:
		gui.update_ball_size(newRadius * 2)
	
func recalculate_katamari_model_size():
	print("placeholder")
	
func can_collect_object_of_size(InSize: float) -> float:
	return volume_to_radius(InSize) <= volume_to_radius(ballvolume) * 0.5
	
func play_char_animation(InAnimator: AnimationPlayer, InAnim: String):
	if charState != InAnim:
		InAnimator.play(InAnim)
		charState = InAnim
	
	
func manage_character_animations() -> void:
	var forward = Vector3(0, 0, -1).rotated(Vector3.UP, ballcam_yaw)
	var right = Vector3(1, 0, 0).rotated(Vector3.UP, ballcam_yaw)
	var char = get_node("character") as Node3D
	char.global_position = position + forward * get_katamari_radius() * -2 + Vector3.UP * -get_katamari_radius()
	char.global_rotation = Vector3(0, ballcam_yaw + PI, 0)
	char.scale = Vector3(0.075, 0.075, 0.075)
	var animator = char.get_node("AnimationPlayer") as AnimationPlayer
	if linear_velocity.length() <= 1:
		play_char_animation(animator, "Wait")
	else: if linear_velocity.length() <= 6:
		play_char_animation(animator, "WalkSlow")
	else:
		play_char_animation(animator, "Run")

func _process(delta: float) -> void:
	if is_level_finished:
		spring_arm.rotation.x = lerp(spring_arm.rotation.x, deg_to_rad(-25), 3 * delta)
		return
		
	var leftStickNew := player_controller.get_action("stick_click_left")
	var rightStickNew := player_controller.get_action("stick_click_right")
	
	manage_character_animations()
	
	if leftStick == true and rightStickNew == true and rightStick == false or rightStick == true and leftStickNew == true and leftStick == false or leftStick == false and rightStick == false and leftStickNew == true and rightStickNew == true:
		if quickSwitchTime + 333 <= Time.get_ticks_msec():
			quickSwitchTime = Time.get_ticks_msec()
			$BoostSound.stream = QuickSwitchSound
			$BoostSound.play()
	
	var quickSwitchRatio := float(Time.get_ticks_msec() - quickSwitchTime) / 333.0
	
	var desiredPitch := deg_to_rad(-10.0 - (ratio_to_next_transition * 35))
	
	leftStick = leftStickNew
	rightStick = rightStickNew
	
	if quickSwitchTime + 333 > Time.get_ticks_msec():
		desired_yaw += delta * PI * 3
		ballcam_yaw += delta * PI * 3
		ballcam_pitch += sin(quickSwitchRatio * PI * 1.54) * -delta * 10
	
	center_node.position = position + Vector3(0, 0.4, 0)
	var rstickdir := player_controller.get_camera_direction()
	var lstickdir := player_controller.get_move_direction()
	if lstickdir.length() > 1:
		lstickdir = lstickdir.normalized()
	if rstickdir.length() > 1:
		rstickdir = rstickdir.normalized()
	
	var stickdiff := lstickdir.y - rstickdir.y
	stickdiff = move_toward(stickdiff, 0, 0.1)
	desired_yaw += stickdiff * delta * 1.25
	
	var Anchor := get_node("CameraAnchor") as RayCast3D
	var distDown := 1
	Anchor.target_position = to_local(position - Vector3(0, get_katamari_diameter() * distDown, 0))
	var desiredPos = Anchor.get_collision_point() + Vector3(0, get_katamari_diameter() * distDown, 0)
	if !Anchor.is_colliding():
		desiredPos = position
		
	#cameraPos = Vector3(desiredPos.x, lerp(cameraPos.y, desiredPos.y, delta * 10), desiredPos.z)
	cameraPos = lerp(cameraPos, desiredPos, delta * 12)
	
	center_node.global_position = cameraPos
	
	#ballcam_pitch = deg_to_rad(-20.0)
	ballcam_pitch = lerp(ballcam_pitch, desiredPitch, delta * 3)
	ballcam_yaw = lerp_angle(ballcam_yaw, desired_yaw, delta * 10)
	
	spring_arm.rotation = Vector3(ballcam_pitch, ballcam_yaw, 0)
	spring_arm.spring_length = move_toward(spring_arm.spring_length, desired_arm_dist, delta * 4)
	
	var NoCollider = get_node("CollectibleNocollider") as Area3D
	for overlap in NoCollider.get_overlapping_bodies():
		if overlap.get_class() == "RigidBody3D":
			var body = overlap as RigidBody3D
			if can_collect_object_of_size(body.mass):
				body.set_collision_layer_value(1, false)
			else: 
				body.set_collision_layer_value(1, true)
				
	var Collector = get_node("CollectibleGrabber") as Area3D
	#var newRadius = volume_to_radius(ballvolume)
	Collector.global_rotation = Vector3.ZERO
	for overlap in Collector.get_overlapping_bodies():
		if overlap.get_class() == "RigidBody3D":
			var body := overlap as RigidBody3D
			if body and can_collect_object_of_size(body.mass):
				if body.get_child_count() < 2:
					return
				if body.has_node("CollectibleShape"):
					print("Collected!")
					var ModelMesh := body.get_node("CollectibleModel") as MeshInstance3D
					ModelMesh.set_instance_shader_parameter("collected", true)
					var ModelShape := body.get_node("CollectibleShape") as CollisionShape3D
					var OldPos := body.global_position
					var OldRot := body.global_rotation
					if body.has_node("AnimationPlayer"):
						body.animated = false
						var Animator := body.get_node("AnimationPlayer") as AnimationPlayer
						Animator.queue_free()
					body.set_collision_layer_value(3, false)
					body.get_parent().remove_child(body)
					add_child(body)
					body.position = to_local(OldPos)
					body.global_rotation = OldRot
					ballvolume += body.mass * VOLUMESCALE
					body.mass = 0.001
					#body.visible = false
					recalculate_katamari_size()
					#add_katamari_hull_point(body.position)
					var modifiedPoints = PackedVector3Array()
					add_katamari_hull_point(body.position, false, null)
					if body.has_meta("col_points"):
						print("Vault points detected!")
						var ColPoints := body.get_meta("col_points") as PackedVector3Array
						for point in ColPoints:
							var newPoint := body.transform * point
							add_katamari_hull_point(newPoint, false, null)
					ModelShape.queue_free()
					var rng = RandomNumberGenerator.new()
					print("We parented it now.")
					$CollectSound.stream = CollectSounds[rng.randi_range(0, 2)]
					$CollectSound.play()

func get_last_local_contact_position(pos: Vector3):
	return LastGroundContact - pos
	
func get_collision_impulse(inVel: Vector3, normal: Vector3, velAtPoint: Vector3):
		var restitution = 1
		if inVel.length() > 5 and AirTime > 0.05:
			restitution = 1.5
		var dot = -normal.dot(inVel - velAtPoint)
		
		if dot >= 5:
			$CollisionSound.stream = SoftCollideSound
			$CollisionSound.play()
		
		dot *= restitution
		if dot > 0:
			return normal * dot
		else:
			return Vector3.ZERO

func get_effective_radius():
	if anyPoint:
		return KatamariHullPointData[furthestCollidingPoint][0].target_position.length()
	else:
		return get_katamari_radius()

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	
	state.linear_velocity = oldVel
	state.angular_velocity = oldAngularVel
	var cooltransform = state.transform
	cooltransform.basis = oldBasis
	
	var forward = Vector3(0, 0, -1).rotated(Vector3.UP, ballcam_yaw)
	var right = Vector3(1, 0, 0).rotated(Vector3.UP, ballcam_yaw)
	
	var forwardDir = -spring_arm.basis.z
	forwardDir.y = 0
	forwardDir = forwardDir.normalized()
	var rstickdir := player_controller.get_camera_direction()
	var lstickdir := player_controller.get_move_direction()
	if lstickdir.length() > 1:
		lstickdir = lstickdir.normalized()
	if rstickdir.length() > 1:
		rstickdir = rstickdir.normalized()
		
	var avgDir = (lstickdir + rstickdir).normalized()
	
	var stickmagnitude = ((rstickdir + lstickdir) * 0.5).length()
	var truestickdir = avgDir * stickmagnitude
	var stickdot = rstickdir.dot(lstickdir)
	var stickdotcapped = clampf(rstickdir.dot(lstickdir) * 4, 0, 1)
	truestickdir = truestickdir * stickdotcapped
	
	var steering = (lstickdir.y - rstickdir.y) * 0.6
	steering = clampf(move_toward(steering, 0, 0.2) * 1.2, -1, 1)
	
	var slope = (state.linear_velocity * Vector3(1, 0, 1)).normalized().dot((LastGroundNormal * Vector3(1, 0, 1)).normalized())
	#-1 uphill, 1 downhill
	var gravSlope = clampf(remap(slope, -1.0, 1.0, 1.5, -0.5), 0.0, 1.0)
	var gravityMult = 1 - minf(maxf(steering, truestickdir.length()), gravSlope)
	
	if absf(steering) > 0.25 and absf(oldSteering) < 0.25 and boostPower > 0 and !boostHuffing:
		if gachaLastDir == 0 or steering > 0 and gachaLastDir == -1 or steering < 0 and gachaLastDir == 1:
			gachaLastDir = sign(steering)
			gachaCount = gachaCount + 1
			gachaTimer = Time.get_ticks_msec()
			if gachaCount == 3:
				$BoostSound.stream = ChargeSound
				$BoostSound.play()
			if gachaCount == 5:
				state.linear_velocity += forward * 15
				$BoostSound.stream = BoostSound
				$BoostSound.play()
	
	if Time.get_ticks_msec() > gachaTimer + 400:
		gachaCount = 0
		gachaLastDir = 0
	
	if gachaCount > 2:
		boostPower -= 30 * state.step
	else:
		boostPower += 5.4 * state.step
	
	boostPower = clampf(boostPower, 0, 240)
		
	if boostPower == 0:
		boostHuffing = true
		boostHuffTime = Time.get_ticks_msec()
	
	if boostHuffing and Time.get_ticks_msec() > boostHuffTime + 8000:
		boostPower = 240
		boostHuffing = false
	
	oldSteering = steering
	
	
	if climbTimer <= 0.5:
		if AirTime <= 0.05:
			state.linear_velocity += Vector3(0, -30 * gravityMult, 0) * state.step
		else:
			state.linear_velocity += Vector3(0, -30, 0) * state.step
	
	
	if anyPoint and gachaCount < 3 and climbTimer < 0.5:
		var point = KatamariHullPointData[furthestCollidingPoint]
		var ray = point[0] as RayCast3D
		var contactPoint = ray.get_collision_point()
		var debugaxis = get_node("debug_axis") as Sprite3D
		var offset = to_global(ray.target_position) - contactPoint
		var depth = -offset.dot(ray.get_collision_normal())
		var length = ray.target_position.length()
		var dir = (to_global(ray.target_position) - cooltransform.origin).normalized() * -1
		var collider = ray.get_collider() as RigidBody3D
		var velAtPoint = collider.get_velocity_at_point(contactPoint)
		debugaxis.global_position = position + velAtPoint
		debugaxis.global_rotation = Vector3.ZERO
		var colImpulse = get_collision_impulse(state.linear_velocity, ray.get_collision_normal(), velAtPoint)
		var velDot = ray.get_collision_normal().dot(state.linear_velocity)
		LastGroundNormal = ray.get_collision_normal()
		
		if velDot > 1:
			state.linear_velocity += colImpulse
			anyPoint = false
			KatamariHullPointData[furthestCollidingPoint][3] = false
			return
		
		if !point[3]:
			cooltransform.origin += ray.get_collision_normal() * depth
			KatamariHullPointData[furthestCollidingPoint][4] = ray.get_collider() as RigidBody3D
			KatamariHullPointData[furthestCollidingPoint][3] = true
			KatamariHullPointData[furthestCollidingPoint][2] = point[4].to_local(cooltransform.origin + dir * -length)
			if length > get_katamari_diameter():
				$CollisionSound.stream = VaultSound
				$CollisionSound.play()
		
		AirTime = 0
		cooltransform.origin = point[4].to_global(point[2]) + dir * length * 0.99
		state.linear_velocity += colImpulse
	
	
	
	position = cooltransform.origin
	
	force_update_transform()
	
	if anyPoint:
		var Ray = get_node("CollisionBackup") as ShapeCast3D
		Ray.position = Vector3(0,0,0)
		Ray.target_position = Vector3(0,0,0)
		#Ray.force_update_transform()
		Ray.force_shapecast_update()
		
		
		if Ray.is_colliding():
			for result in Ray.get_collision_count():
				var colPoint = Ray.get_collision_point(result)
				var colNormal = Ray.get_collision_normal(result)
				var dist = colPoint.distance_to(cooltransform.origin)
				if dist >= get_katamari_radius():
					continue
				var desiredPos = colPoint + colNormal * get_katamari_radius() * 0.99
				cooltransform.origin = desiredPos
				position = cooltransform.origin
				#state.linear_velocity += normal
				#state.linear_velocity *= -1
	
	anyPoint = false
	furthestCollidingPoint = 0
	furthestDist = 0
	
	for i in range(KatamariHullPointData.size()):
		if climbTimer > 0.5 or gachaCount >= 3:
			KatamariHullPointData[i][3] = false
			continue
		var point = KatamariHullPointData[i]
		point[0].force_raycast_update()
		var ray = point[0] as RayCast3D
		if ray.is_colliding() and ray.get_collision_normal().y > 0.2:
			anyPoint = true
			var offset = to_global(ray.target_position) - ray.get_collision_point()
			var depth = -offset.dot(ray.get_collision_normal())
			if depth > furthestDist:
				furthestCollidingPoint = i
				furthestDist = depth
		else:
			KatamariHullPointData[i][3] = false
	
	var touchingWall = false
	
	var curNormal = Vector3(0, 1, 0)
	
	var tempOldPos = position
	for contact in state.get_contact_count():
		var normal = state.get_contact_local_normal(contact)
		curNormal = normal
		LastGroundNormal = normal
		var pos = state.get_contact_local_position(contact)
		if cooltransform.origin.distance_to(pos) < get_katamari_radius():
			anyPoint = false
		
		var velAtPoint = state.get_contact_collider_object(contact).get_velocity_at_point(pos)
		#var velAtPoint = collider.get_velocity_at_point(contactPoint)
		var debugaxis = get_node("debug_axis") as Sprite3D
		debugaxis.global_position = position + velAtPoint
		debugaxis.global_rotation = Vector3.ZERO
		state.linear_velocity += get_collision_impulse(state.linear_velocity, normal, velAtPoint)
		if normal.y <= 0.2 and normal.y >= 0:
			touchingWall = true
			
	
	if is_on_floor(state):
		AirTime = 0
	else:
		AirTime += state.step
	
	if touchingWall:
		climbTimer += state.step
	
	if climbTimer >= 0.5:
		state.linear_velocity += Vector3(0, get_katamari_diameter() * state.step * 4, 0) - curNormal * state.step * 10
		state.linear_velocity += -state.linear_velocity * state.step * 4
	
	if !touchingWall and climbTimer >= 0.5:
		noClimbTimer += state.step
	
	if noClimbTimer >= 0.2:
		print("NO CLIMB!")
		climbTimer = 0
		noClimbTimer = 0
	
	var accel_vector = Vector3(truestickdir.x, 0, truestickdir.y).rotated(Vector3.UP, ballcam_yaw)
	var real_travel_axis = state.linear_velocity.cross(Vector3.UP + LastGroundNormal).normalized()
	if !real_travel_axis.is_normalized():
		real_travel_axis = Vector3(0, 0, -1)
	
	if climbTimer > 1 and accel_vector.dot(LastGroundNormal) > -0.5:
		climbTimer = 0
	
	
	if AirTime <= 0.05:
		state.linear_velocity += accel_vector * 24 * (get_katamari_radius() + 0.5) * state.step * (1 - absf(steering))
		state.linear_velocity += -state.linear_velocity * 3 * state.step * (1 - absf(steering))
	
	cooltransform.basis = cooltransform.basis.rotated(real_travel_axis, (state.linear_velocity.length() * PI * -0.25 * state.step) / get_effective_radius())
	
	if climbTimer > 0.5:
		cooltransform.basis = cooltransform.basis.rotated(LastGroundNormal.cross(Vector3.UP).normalized(), state.step * 12 / get_katamari_diameter())
	
	if gachaCount > 2 and gachaCount < 5:
		cooltransform.basis = cooltransform.basis.rotated(right, state.step * 24)
				
	LastEffectiveVelocity = cooltransform.origin - oldPos
	if quickSwitchTime + 333 <= Time.get_ticks_msec():
		var camDiff = ballcam_yaw - old_ballcam_yaw
		state.linear_velocity = state.linear_velocity.rotated(Vector3(0, 1, 0), camDiff)
	cooltransform.basis = cooltransform.basis.orthonormalized()
	oldAngularVel = state.angular_velocity
	oldVel = state.linear_velocity
	oldPos = cooltransform.origin
	oldBasis = cooltransform.basis
	old_ballcam_yaw = ballcam_yaw
	
	state.transform = cooltransform

func is_on_floor(state: PhysicsDirectBodyState3D) -> bool:
	for contact in state.get_contact_count():
		var contact_normal := state.get_contact_local_normal(contact)
		#apply_impulse (contact_normal * contact_normal.dot(linear_velocity))
		if contact_normal.dot(Vector3.UP) > 0.2:
			return true
	return false
	
func add_katamari_hull_point(local_point: Vector3, permanent: bool, body: RigidBody3D):
	var RayCaster = RayCast3D.new()
	add_child(RayCaster)
	RayCaster.target_position = local_point
	RayCaster.set_collision_mask_value(1, true)
	var tempTable = [RayCaster, permanent, Vector3(0,0,0), false, self]
	KatamariHullPointData.append(tempTable)

func do_finish_effect(finish_pos: Vector3):
	is_level_finished = true
	gravity_scale = 0
	linear_damp = 3


func set_player_controller(new_controller: PlayerController) -> void:
	if player_controller != null:
		player_controller.queue_free()
	player_controller = new_controller
	# new_controller.connect("mouse_moved", self._on_mouse_moved)

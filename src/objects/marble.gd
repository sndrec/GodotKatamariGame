extends RigidBody3D


signal gem_collected
signal level_finished

const VOLUMESCALE = 0.66

var player_controller: PlayerController = null

var ballcam_pitch := deg_to_rad(-10.0)
var ballcam_yaw := 0.0
var desired_yaw := 0.0
var desired_arm_dist := 3.0
var ratio_to_next_transition := 0.0

var just_transitioned := false

var forward_timer := 0
var recent_forward_vector := Vector2.ZERO

var gui

var ballvolume = 1
var startingvolume = 1

var CurThreshold = 0
var NextThreshold = 0
var FinalThreshold = false

var jumping := false

var is_level_finished := false
var finish_point: Vector3

var oldVel = Vector3.ZERO
var oldAngularVel = Vector3.ZERO
var oldPos = Vector3(0, 2, 0)
var oldBasis = Basis.IDENTITY

var desiredVel = 0.0

var LastGroundNormal = Vector3.UP
var LastGroundContact = Vector3(-5, 4.015605, 16.34)
var LastGroundVelocity = Vector3.ZERO
var GroundTime = 0

var anyPoint = false
var furthestCollidingPoint = 0
var furthestDist = 0

var LastEffectiveVelocity = Vector3(0,0,0)

var OnStick = false

var climbTimer = 0

var KatamariHullPointData = []

var cameraPos = Vector3(0, 0, 0)

@onready var center_node: Node3D = $CenterNode
@onready var spring_arm: SpringArm3D = $CenterNode/SpringArm3D


func _ready() -> void:
	center_node.top_level = true
	center_node.position = position + Vector3(0, 0.5, 0)
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
	


static func angle_to_angle(from, to):
	return fposmod(to-from + PI, PI*2) - PI
	
func volume_to_radius(volume):
	return pow(6.0 * volume / PI, 1.0 / 3.0) * 0.5
	
func radius_to_volume(radius):
	return (4.0/3.0) * PI * pow(radius, 3)

func set_katamari_diameter(diameter):
	ballvolume = radius_to_volume(diameter * 0.5)
	startingvolume = radius_to_volume(diameter * 0.5)
	recalculate_katamari_size()
	
func get_katamari_diameter():
	return volume_to_radius(ballvolume) * 2.0

func get_katamari_radius():
	return volume_to_radius(ballvolume)

func recalculate_katamari_size():
	var newRadius = volume_to_radius(ballvolume)
	var Thresholds = get_meta("SizeThresholds") as PackedFloat32Array
	var OldThreshold = CurThreshold
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
	
	var BallCollision = get_node("KatamariColliderShape") as CollisionShape3D
	var BallCollisionShape = BallCollision.shape as SphereShape3D
	BallCollisionShape.radius = newRadius
	var NocolliderCollision = get_node("CollectibleNocollider").get_node("NocolliderShape") as CollisionShape3D
	var NocolliderCollisionShape = NocolliderCollision.shape as SphereShape3D
	NocolliderCollisionShape.radius = newRadius * 2
	var GrabberCollision = get_node("CollectibleGrabber").get_node("KatamariCollectorShapeMiddle") as CollisionShape3D
	var GrabberCollisionShape = GrabberCollision.shape as SphereShape3D
	GrabberCollisionShape.radius = newRadius * 1
	var BottomGrabberCollision = get_node("CollectibleGrabber").get_node("KatamariCollectorShapeBottom") as CollisionShape3D
	BottomGrabberCollision.position = Vector3(0, -newRadius * 0.8, 0)
	var BottomGrabberCollisionShape = BottomGrabberCollision.shape as SphereShape3D
	BottomGrabberCollisionShape.radius = newRadius * 0.2
	var KatamariModel = get_node("KatamariModel") as Node3D
	var visualRadius = volume_to_radius(startingvolume + ((ballvolume - startingvolume) * 0.1))
	var NewScale = visualRadius * 2.7
	#print(NewScale)
	KatamariModel.scale = Vector3(NewScale, NewScale, NewScale)
	if gui:
		gui.update_ball_size(newRadius * 2)
	
func recalculate_katamari_model_size():
	print("placeholder")
	
func can_collect_object_of_size(InSize):
	return volume_to_radius(InSize) <= volume_to_radius(ballvolume) * 0.5

func _process(delta: float) -> void:
	if is_level_finished:
		center_node.position = lerp(center_node.position, finish_point, delta)
		spring_arm.rotation.x = lerp(spring_arm.rotation.x, deg_to_rad(-25), 3 * delta)
		return

	center_node.position = position + Vector3(0, 0.75, 0)
	#quaternion = Quaternion(oldBasis)
	#print(LastGroundContact.length())
	var rstickdir := player_controller.get_camera_direction()
	var lstickdir := player_controller.get_move_direction()
	if lstickdir.length() > 1:
		lstickdir = lstickdir.normalized()
	if rstickdir.length() > 1:
		rstickdir = rstickdir.normalized()
	
	var stickdiff = lstickdir.y - rstickdir.y
	desired_yaw += stickdiff * delta * 1.25
	
	var Anchor = get_node("CameraAnchor") as RayCast3D
	var distDown = 1.2
	Anchor.target_position = to_local(position - Vector3(0, get_katamari_diameter() * distDown, 0))
	var desiredPos = Anchor.get_collision_point() + Vector3(0, get_katamari_diameter() * distDown, 0)
	if !Anchor.is_colliding():
		desiredPos = position
		
	#cameraPos = Vector3(desiredPos.x, lerp(cameraPos.y, desiredPos.y, delta * 10), desiredPos.z)
	cameraPos = lerp(cameraPos, desiredPos, delta * 12)
	
	center_node.global_position = cameraPos
	
	#ballcam_pitch = deg_to_rad(-20.0)
	ballcam_pitch = lerp(ballcam_pitch, deg_to_rad(-10.0 - (ratio_to_next_transition * 35)), delta * 3)
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
			var body = overlap as RigidBody3D
			if body and can_collect_object_of_size(body.mass):
				print("Collected!")
				var ModelShape = body.get_node("CollectibleShape") as CollisionShape3D
				if ModelShape:
					var OldPos = body.position
					var OldRot = body.global_rotation
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
					add_katamari_hull_point(body.position, false)
					ModelShape.queue_free()

func get_last_local_contact_position(pos: Vector3):
	return LastGroundContact - pos
	
func get_collision_impulse(inVel: Vector3, normal: Vector3):
		var restitution = 1
		if inVel.length() > 5:
			restitution = 1.2
		var dot = normal.dot(inVel) * -restitution
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
	
	var forwardDir = -spring_arm.basis.z
	forwardDir.y = 0
	forwardDir = forwardDir.normalized()
	var rstickdir := player_controller.get_camera_direction()
	var lstickdir := player_controller.get_move_direction()
	if lstickdir.length() > 1:
		lstickdir = lstickdir.normalized()
	if rstickdir.length() > 1:
		rstickdir = rstickdir.normalized()
	
	var truestickdir = (rstickdir + lstickdir) * 0.5
	#var stickdot = rstickdir.dot(lstickdir)
	#var stickdotcapped = maxf(rstickdir.dot(lstickdir), 0)
	#truestickdir = truestickdir * stickdotcapped
	
	var steering = lstickdir.y - rstickdir.y
	steering = move_toward(steering, 0, 0.1)
	
	if truestickdir.length() > 0 and absf(steering) <= 0.75:
		forward_timer = Time.get_ticks_msec()
		if truestickdir.length() > recent_forward_vector.length():
			recent_forward_vector = truestickdir
		else:
			recent_forward_vector = truestickdir.normalized() * recent_forward_vector.length()
	
	if absf(steering) <= 0.75 and Time.get_ticks_msec() > forward_timer + 50:
		recent_forward_vector = Vector2.ZERO
	else:if absf(steering) > 0.75:
		truestickdir = recent_forward_vector
	
	if climbTimer <= 0.2:
		if GroundTime >= 0.02:		
			state.linear_velocity += Vector3(0, -38 + (truestickdir.length() * 32), 0) * state.step
		else:
			state.linear_velocity += Vector3(0, -38, 0) * state.step
	
	var oldGroundTime = GroundTime
	
	if is_on_floor(state):
		GroundTime += state.step
	else:
		GroundTime = 0
		
	
	if anyPoint:
		print("Any point!")
		GroundTime = oldGroundTime + state.step
		var point = KatamariHullPointData[furthestCollidingPoint]
		var ray = point[0] as RayCast3D
		var contactPoint = ray.get_collision_point()
		var endPoint = to_global(point[2])
		var offset = to_global(ray.target_position) - ray.get_collision_point()
		var depth = -offset.dot(ray.get_collision_normal())
		if !point[3]:
			cooltransform.origin += ray.get_collision_normal() * depth
			KatamariHullPointData[furthestCollidingPoint][3] = true
			KatamariHullPointData[furthestCollidingPoint][2] = contactPoint
		
		var length = ray.target_position.length()
		var dir = (to_global(ray.target_position) - cooltransform.origin).normalized() * -1
		cooltransform.origin = point[2] + dir * length * 0.99
		state.linear_velocity += get_collision_impulse(state.linear_velocity, ray.get_collision_normal())
		#KatamariHullPointData[furthestCollidingPoint][0].target_position = to_local((contactPoint))
	
	position = cooltransform.origin
	anyPoint = false
	furthestCollidingPoint = 0
	furthestDist = 0
	
	for i in range(KatamariHullPointData.size()):
		var point = KatamariHullPointData[i]
		point[0].force_raycast_update()
		var ray = point[0] as RayCast3D
		if ray.is_colliding() and ray.get_collision_normal().y > 0.2 and (to_global(ray.target_position) - cooltransform.origin).normalized().y < -0.05:
			anyPoint = true
			var offset = to_global(ray.target_position) - ray.get_collision_point()
			var depth = -offset.dot(ray.get_collision_normal())
			#print(depth)
			if depth > furthestDist:
				furthestCollidingPoint = i
				furthestDist = depth
		else:
			KatamariHullPointData[i][3] = false
	
	var touchingWall = false
	
	var curNormal = Vector3(0, 1, 0)
	
	for contact in state.get_contact_count():
		var normal = state.get_contact_local_normal(contact)
		curNormal = normal
		var pos = state.get_contact_local_position(contact)
		if cooltransform.origin.distance_to(pos) < get_katamari_radius():
			anyPoint = false
		state.linear_velocity += get_collision_impulse(state.linear_velocity, normal)
		if normal.y <= 0.2 and normal.y >= 0:
			touchingWall = true
			climbTimer += state.step
		
	if !touchingWall:
		climbTimer = 0
		
	
	if climbTimer >= 0.2:
		state.linear_velocity += Vector3(0, get_katamari_diameter() * state.step * 4, 0) - curNormal * state.step * 10
		state.linear_velocity += -state.linear_velocity * state.step * 4
	
	var gravityDir = Vector3(0, -1, 0)
		
	
	var accel_vector = Vector3(truestickdir.x, 0, truestickdir.y).rotated(Vector3.UP, ballcam_yaw)
	var real_travel_axis = state.linear_velocity.cross(curNormal).normalized()
	if !real_travel_axis.is_normalized():
		real_travel_axis = Vector3(0, 0, -1)
	
	if accel_vector.dot(curNormal) > -0.5:
		climbTimer = 0
	
	if GroundTime >= 0.02:
		
		state.linear_velocity += accel_vector * 24 * (get_katamari_radius() + 0.5) * state.step
		state.linear_velocity += -state.linear_velocity * 3 * state.step
		if OnStick:
			cooltransform.basis = cooltransform.basis.rotated(real_travel_axis, (state.linear_velocity.length() * PI * -0.25 * state.step) / get_effective_radius())
			cooltransform.basis = cooltransform.basis.orthonormalized()
		else:
			cooltransform.basis = cooltransform.basis.rotated(real_travel_axis, (state.linear_velocity.length() * PI * -0.25 * state.step) / get_effective_radius())
			cooltransform.basis = cooltransform.basis.orthonormalized()
	else:
		cooltransform.basis = cooltransform.basis.rotated(real_travel_axis, (state.linear_velocity.length() * PI * -0.25 * state.step) / get_effective_radius())
		cooltransform.basis = cooltransform.basis.orthonormalized()
	
	LastEffectiveVelocity = cooltransform.origin - oldPos
	
	oldAngularVel = state.angular_velocity
	oldVel = state.linear_velocity
	oldPos = cooltransform.origin
	oldBasis = cooltransform.basis
	
	state.transform = cooltransform

func is_on_floor(state: PhysicsDirectBodyState3D) -> bool:
	for contact in state.get_contact_count():
		var contact_normal := state.get_contact_local_normal(contact)
		#apply_impulse (contact_normal * contact_normal.dot(linear_velocity))
		if contact_normal.dot(Vector3.UP) > 0.2:
			return true
	return false
	
func add_katamari_hull_point(local_point: Vector3, permanent: bool):
	var RayCaster = RayCast3D.new()
	add_child(RayCaster)
	RayCaster.target_position = local_point
	RayCaster.set_collision_mask_value(1, true)
	var tempTable = [RayCaster, permanent, Vector3(0,0,0), false]
	KatamariHullPointData.append(tempTable)
	print("Added raycaster!")
	print(RayCaster)
	
func instantiate_katamari_hull():
	var t = (1.0 + sqrt(5.0)) / 2.0

	var NewPointArray = [Vector3(-1,  t,  0), Vector3( 1,  t,  0), Vector3(-1, -t,  0), Vector3( 1, -t,  0),	Vector3( 0, -1,  t), Vector3( 0,  1,  t), Vector3( 0, -1, -t), Vector3( 0,  1, -t), Vector3( t,  0, -1), Vector3( t,  0,  1), Vector3(-t,  0, -1), Vector3(-t,  0,  1)]
	for i in range(0, NewPointArray.size()):
		NewPointArray[i] *= 0.25
		add_katamari_hull_point(NewPointArray[i], true)

func do_finish_effect(finish_pos: Vector3):
	is_level_finished = true
	finish_point = finish_pos
	gravity_scale = 0
	linear_damp = 3


func set_player_controller(new_controller: PlayerController) -> void:
	if player_controller != null:
		player_controller.queue_free()
	player_controller = new_controller
	# new_controller.connect("mouse_moved", self._on_mouse_moved)

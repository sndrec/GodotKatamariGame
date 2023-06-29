@tool
extends RigidBody3D

var curTransform := Transform3D.IDENTITY
var oldTransform := Transform3D.IDENTITY
var transformDiff := Transform3D.IDENTITY
var animated := false
var latestDelta := 1.0

var bigDot = preload("res://assets/debug/bigdot.png")

@export var PickupMultiplier: float = 1.0:
	set(new_pickupmult):
		PickupMultiplier = new_pickupmult

@export var GiveMultiplier: float = 1.0:
	set(new_givemult):
		GiveMultiplier = new_givemult

@export var VaultPoints = PackedVector3Array():
	set(new_array):
		VaultPoints = new_array
		if Engine.is_editor_hint():
			for child in get_children():
				if child.get_class() == "Sprite3D":
					child.queue_free()
			for point in VaultPoints:
				var Sprite = Sprite3D.new()
				add_child(Sprite)
				Sprite.position = point
				Sprite.texture = bigDot
				Sprite.pixel_size = 0.005
				Sprite.no_depth_test = true
				Sprite.fixed_size = true
				Sprite.billboard = true
				print(point)


# Called when the node enters the scene tree for the first time.
func _ready():
	if Engine.is_editor_hint():
		return
	instantiate()

func calc_mass():
	var ModelShape := get_node("CollectibleShape") as CollisionShape3D
	var ConvexHull := ModelShape.shape as ConvexPolygonShape3D
	if ConvexHull:
		var Bounds := AABB()
		var Mins := Vector3(0, 0, 0)
		var Maxs := Vector3(0, 0, 0)
		for point in ConvexHull.points:
			if point.x < Mins.x:
				Mins.x = point.x
			if point.y < Mins.y:
				Mins.y = point.y
			if point.z < Mins.z:
				Mins.z = point.z
			if point.x > Maxs.x:
				Maxs.x = point.x
			if point.y > Maxs.y:
				Maxs.y = point.y
			if point.z > Maxs.z:
				Maxs.z = point.z
		Bounds.size = Maxs - Mins
		Bounds.size *= ModelShape.scale
		mass = Bounds.get_volume()
	else:
		var TriMesh = ModelShape.shape as ConcavePolygonShape3D
		if TriMesh:
			var Bounds := AABB()
			var Mins := Vector3(0, 0, 0)
			var Maxs := Vector3(0, 0, 0)
			for point in TriMesh.get_faces():
				if point.x < Mins.x:
					Mins.x = point.x
				if point.y < Mins.y:
					Mins.y = point.y
				if point.z < Mins.z:
					Mins.z = point.z
				if point.x > Maxs.x:
					Maxs.x = point.x
				if point.y > Maxs.y:
					Maxs.y = point.y
				if point.z > Maxs.z:
					Maxs.z = point.z
			Bounds.size = Maxs - Mins
			Bounds.size *= ModelShape.scale
			mass = Bounds.get_volume()
		else:
			var VisMesh = get_node("CollectibleModel") as MeshInstance3D
			if VisMesh:
				print("yay!")

func instantiate():
	
	oldTransform = global_transform
	curTransform = global_transform
	if has_node("AnimationPlayer"):
		var Animator := get_node("AnimationPlayer") as AnimationPlayer
		if Animator.has_animation("track"):
			Animator.set_assigned_animation("track")
			animated = true
		else:
			Animator.queue_free()
	
	if has_meta("volume"):
		mass = get_meta("volume")
	else:
		calc_mass()

func get_velocity_at_point(inPoint: Vector3) -> Vector3:
	var oldPoint = oldTransform.inverse() * inPoint
	var newPoint = curTransform * oldPoint
	var diff = newPoint - inPoint
	
	#print(latestDelta)
	return diff / latestDelta

func _process(delta: float) -> void:
	
	if Engine.is_editor_hint():
		return
	oldTransform = global_transform
	if animated:
		$AnimationPlayer.advance(delta)
		force_update_transform()
	curTransform = global_transform
	latestDelta = delta

@tool
extends RigidBody3D

var curTransform := Transform3D.IDENTITY
var oldTransform := Transform3D.IDENTITY
var transformDiff := Transform3D.IDENTITY
var animated := false
var latestDelta := 1.0
var pickedUp = false

var bigDot = preload("res://assets/debug/bigdot.png")
var pickupSoundNode = AudioStreamPlayer.new()
var collectibleReady = false

@export var ItemName: String = "Default":
	set(new_name):
		ItemName = new_name

@export var ItemDesc: String = "This is the default\nitem description.":
	set(new_desc):
		ItemDesc = new_desc
		
@export var PickupSound: AudioStreamWAV:
	set(new_sound):
		PickupSound = new_sound

@export var AllowSkeletonAnimation: bool = false:
	set(new_allow):
		AllowSkeletonAnimation = new_allow

@export var PickupMultiplier: float = 1.0:
	set(new_pickupmult):
		PickupMultiplier = new_pickupmult
		if Engine.is_editor_hint() and collectibleReady:
			print_katamari_debug_hint()

@export var GiveMultiplier: float = 1.0:
	set(new_givemult):
		GiveMultiplier = new_givemult
		if Engine.is_editor_hint() and collectibleReady:
			print_katamari_debug_hint()

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
				Sprite.pixel_size = 0.002
				Sprite.no_depth_test = true
				Sprite.fixed_size = true
				Sprite.billboard = true


# Called when the node enters the scene tree for the first time.
func _ready():
	if Engine.is_editor_hint():
		collectibleReady = true
		calc_mass()
		
		print_katamari_debug_hint()
		return
	instantiate()

func print_katamari_debug_hint():
	calc_mass()
	var neededDiameter = volume_to_radius(mass * pow(PickupMultiplier, 3)) * 4
	print("Wow! It's a \"" + ItemName + "\"!")
	print("The katamari must be " + prettify_size(neededDiameter) + " in diameter to collect it.")
	var nextDiameter = radius_to_volume(neededDiameter * 0.5) + mass * GiveMultiplier * 0.5
	nextDiameter = volume_to_radius(nextDiameter) * 2
	var diff = nextDiameter - neededDiameter
	print("A katamari just big enough would gain " + str(snapped(diff, 0.001)) + "cm in diameter,")
	print("which is a " + str(snapped(diff / neededDiameter, 0.001)) + "% increase.")

func prettify_size(inSize: float) -> String:
	var mm = int(floor(inSize * 10)) % 10
	var cm = int(floor(inSize)) % 100
	var m = int(floor(inSize * 0.01)) % 1000
	var km = int(floor(inSize * 0.00001))
	
	if inSize < 1:
		return str(mm) + "mm"
	if inSize < 100:
		return str(cm) + "cm " + str(mm) + "mm"
	if inSize < 100000:
		return str(m) + "m " + str(cm) + "cm " + str(mm) + "mm"
	return str(km) + "km " + str(m) + "m " + str(cm) + "cm " + str(mm) + "mm"

func volume_to_radius(volume: float) -> float:
	return pow(6.0 * volume / PI, 1.0 / 3.0) * 0.5

func radius_to_volume(radius: float) -> float:
	return (4.0/3.0) * PI * pow(radius, 3)

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

func manage_item_skeletal_animations(delta: float) -> void:
	var itemModel = get_node("CollectibleModel") as Node3D
	var animator = itemModel.get_node("AnimationPlayer") as AnimationPlayer
	if animator.has_animation("Struggle") and pickedUp:
		animator.play("Struggle")
		return
	if animator.has_animation("Walk"):
		if get_velocity_at_point(position).length() > 0.1:
			animator.play("Walk")
		else:
			animator.play("Idle")
	else:
		animator.play("Idle")

func _process(delta: float) -> void:
	
	if Engine.is_editor_hint():
		return
	
	if AllowSkeletonAnimation and get_node("CollectibleModel").get_class() == "Node3D":
		manage_item_skeletal_animations(delta)
	
	oldTransform = global_transform
	if animated:
		$AnimationPlayer.advance(delta)
		force_update_transform()
	curTransform = global_transform
	latestDelta = delta
	
func _on_pickup_sound_finished():
	pickupSoundNode.queue_free()
	
func pickup():
	pickedUp = true
	print(PickupSound)
	if PickupSound:
		pickupSoundNode = AudioStreamPlayer.new()
		pickupSoundNode.stream = PickupSound
		add_child(pickupSoundNode)
		pickupSoundNode.volume_db = -12
		pickupSoundNode.play()
		pickupSoundNode.finished.connect(_on_pickup_sound_finished)

extends RigidBody3D


# Called when the node enters the scene tree for the first time.
func _ready():
	var ModelShape = get_node("CollectibleShape") as CollisionShape3D
	var ConvexHull = ModelShape.shape as ConvexPolygonShape3D
	if ConvexHull:
		var Bounds = AABB()
		var Mins = Vector3(0, 0, 0)
		var Maxs = Vector3(0, 0, 0)
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
			var Bounds = AABB()
			var Mins = Vector3(0, 0, 0)
			var Maxs = Vector3(0, 0, 0)
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
		
	


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass

@tool
extends EditorScript

var curRoot = Node.new()
var trimesh = false
var collectibleMat = preload("res://addons/flexible_toon_shader/collectible.gdshader")
var hasShape = false

func _run():
	hasShape = false
	curRoot.queue_free()
	curRoot = get_scene()
	hasShape = curRoot.has_node("CollectibleShape")
	for child in get_scene().get_children():
		if child.get_class() == "Node3D":
			child.position = Vector3(0, 0, 0)
			child.rotation = Vector3(0, 0, 0)
			child.name = "CollectibleModel"
			iterate(child)
			break
		if child.get_class() == "MeshInstance3D":
			child.name = "CollectibleModel"
			set_material_on_mesh_instance(child)
			if !hasShape:
				add_collision_from_mesh_instance(child)

func set_material_on_mesh_instance(inst: MeshInstance3D):
	var MeshNodeMesh = inst.mesh as Mesh
	for surface in MeshNodeMesh.get_surface_count():
		var mat = MeshNodeMesh.surface_get_material(surface) as BaseMaterial3D
		if mat:
			var newMat = ShaderMaterial.new()
			newMat.shader = load("res://addons/flexible_toon_shader/collectible.gdshader")
			var tex = mat.albedo_texture
			newMat.set_shader_parameter("albedo_texture", tex)
			newMat.set_shader_parameter("albedo", mat.albedo_color)
			var rimCol = mat.albedo_color * 0.5
			rimCol.a = 0.25
			newMat.set_shader_parameter("rim_color", rimCol)
			newMat.set_shader_parameter("rim_width", 2.0)
			MeshNodeMesh.surface_set_material(surface, newMat)

func add_collision_from_mesh_instance(inst: MeshInstance3D):
	var MeshNodeMesh = inst.mesh as Mesh
	var collider = CollisionShape3D.new()
	if trimesh:
		collider.shape = MeshNodeMesh.create_trimesh_shape()
	else:
		collider.shape = MeshNodeMesh.create_convex_shape(true)
	collider.name = "CollectibleShape"
	collider.visible = true
	collider.scale = inst.scale
	collider.position = inst.position
	collider.rotation = inst.rotation
	curRoot.add_child(collider)
	collider.set_owner(curRoot)
	hasShape = true
	

func iterate(node: Node):
	if node != null:
		if node.get_class() == "MeshInstance3D":
			set_material_on_mesh_instance(node)
			if !hasShape:
				add_collision_from_mesh_instance(node)
		for child in node.get_children():
			iterate(child)

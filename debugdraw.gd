
extends ImmediateMesh

func _ready():
	pass

func draw_line(startPos, endPos):
	clear_surfaces()
	surface_begin(PrimitiveMesh.PRIMITIVE_LINES)
	surface_set_color(Color(1,1,1))
	surface_add_vertex(startPos) 
	surface_add_vertex(endPos)	
	surface_end()

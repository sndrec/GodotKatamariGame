shader_type spatial;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_nearest;

void fragment() {
	vec4 screen = texture(screen_texture, UV);
	ALBEDO = screen.rgb * 0.5;
}
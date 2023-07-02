shader_type spatial;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_nearest;
uniform float strength = 0;

void vertex() {
	POSITION = vec4(VERTEX, 1.0);
}

void fragment() {
	vec4 add = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.05 * strength))) + 0.5);
	vec4 add2 = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.1 * strength))) + 0.5);
	vec4 add3 = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.15 * strength))) + 0.5);
	vec4 add4 = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.2 * strength))) + 0.5);
	vec4 add5 = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.25 * strength))) + 0.5);
	vec4 add6 = texture(screen_texture, ((SCREEN_UV - 0.5) * (1.0 - (0.3 * strength))) + 0.5);
	add *= 0.4 * strength;
	add2 *= 0.2 * strength;
	add3 *= 0.1 * strength;
	add4 *= 0.05 * strength;
	add5 *= 0.025 * strength;
	add6 *= 0.0125 * strength;
	vec4 screen = texture(screen_texture, SCREEN_UV) + add + add2 + add3 + add4 + add5 + add6;
	ALBEDO = screen.rgb;
}
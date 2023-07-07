shader_type spatial;
render_mode unshaded;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_nearest;
uniform sampler2D depth_texture : source_color, hint_depth_texture;
uniform float strength = 0;
uniform float diameter = 5;

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
	
	vec4 dof1 = texture(screen_texture, SCREEN_UV + vec2(0.0025, 0.0036));
	vec4 dof2 = texture(screen_texture, SCREEN_UV + vec2(0.0025, -0.0036));
	vec4 dof3 = texture(screen_texture, SCREEN_UV + vec2(-0.0025, 0.0036));
	vec4 dof4 = texture(screen_texture, SCREEN_UV + vec2(-0.0025, -0.0036));
	
	vec4 finaldof = (dof1 + dof2 + dof3 + dof4) * 0.5;
	
	float depth = texture(depth_texture, SCREEN_UV).x;
	vec3 ndc = vec3(SCREEN_UV * 2.0 - 1.0, depth);
	vec4 view = INV_PROJECTION_MATRIX * vec4(ndc, 1.0);
	view.xyz /= view.w;
	float linear_depth = (-view.z * 0.01) / diameter;
	linear_depth = clamp(linear_depth - 0.1, 0, 1);
	
	ALBEDO = (screen.rgb * (1.0 - linear_depth)) + (finaldof.rgb * linear_depth);
}
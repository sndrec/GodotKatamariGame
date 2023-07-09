shader_type spatial;
render_mode cull_disabled;

//render_mode ambient_light_disabled;


uniform vec4 albedo : source_color = vec4(1.0f);
uniform sampler2D albedo_texture : source_color, filter_nearest;

uniform bool use_attenuation = true;

uniform bool use_rim = true;
uniform float rim_width : hint_range(0.0f, 16.0f) = 8.0f;
uniform vec4 rim_color : source_color = vec4(1.0f);

instance uniform bool collected = false;

varying vec3 vertex_pos;
varying vec3 normal;

float split_specular(float specular) {
	return step(0.5f, specular);
}

void vertex() {
	vertex_pos = VERTEX;
	normal = NORMAL;
}

vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void fragment() {
	vec3 newAlbedo = albedo.rgb * texture(albedo_texture, UV).rgb * COLOR.rgb;
	vec3 alt = rgb2hsv(newAlbedo);
	float saturation = alt.y * 0.6 + 0.1;
	float value = alt.z * 0.9;
	if (collected) {
		saturation *= 1.4;
		value *= 1.2;
		value += 0.1;
	}
	alt.y = min(saturation, 1.0);
	alt.z = min(value, 1.0);
	ALBEDO = hsv2rgb(alt);
}

void light() {
	// Attenuation.
	float attenuation = 1.0f;
	if (use_attenuation) {
		attenuation = ATTENUATION;
	}
	
	// Diffuse lighting.
	float NdotL = dot(NORMAL, LIGHT);
	float diffuse_amount = NdotL + (attenuation - 1.0);
	float SUVY = FRAGCOORD.y / VIEWPORT_SIZE.y;
	SUVY = ((sin(SUVY * 1000.0) / PI) + 1.0) * 0.5;
	float lerpRatio = clamp(SUVY + diffuse_amount, 0, 1);
	diffuse_amount = diffuse_amount * lerpRatio - (1.0 - lerpRatio);
	diffuse_amount = clamp(diffuse_amount * 2.0, 0.0, 1.0);
	
	
	// Apply diffuse result to different styles.
	vec3 diffuse = ALBEDO.rgb * LIGHT_COLOR / PI;
	vec3 diffusefinal = diffuse * (diffuse_amount * 0.5 + 0.5);
	
	
	DIFFUSE_LIGHT = diffusefinal;
	
	// Simple rim lighting.
	if (use_rim) {
		float NdotV = dot(NORMAL, VIEW);
		float rim_light = pow(1.0 - NdotV, rim_width);
		if (!collected){
			rim_light *= 0.25;
		}
		SPECULAR_LIGHT += rim_light * rim_color.rgb * rim_color.a * LIGHT_COLOR / PI; //Changed DIFFUSE_LIGHT to SPECULAR_LIGHT while migrating for similar result
	}
}

shader_type spatial;
render_mode cull_disabled, depth_draw_always;

//render_mode ambient_light_disabled;


uniform vec4 albedo : source_color = vec4(1.0f);
uniform sampler2D albedo_texture : source_color, filter_nearest;

uniform sampler2D depth_texture : source_color, hint_depth_texture;

uniform bool use_attenuation = true;

uniform bool use_rim = true;
uniform float rim_width : hint_range(0.0f, 16.0f) = 8.0f;
uniform vec4 rim_color : source_color = vec4(1.0f);

instance uniform bool collected = false;
global uniform float katamariSize;
global uniform vec3 katamariPos;

uniform float alpha_mult: hint_range(0,1) = 1.0;
uniform float alpha_clip: hint_range(0,1) = 0.0;

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

float linear_depth(in float depth, in vec2 screen_uv, in mat4 inv_projection_matrix){
	// get raw depth, this is not a linear value in godot 4.0 vulkan rendering
    
	vec3 normalized_device_coordinates = vec3(screen_uv * 2.0 - 1.0, depth);
	
	//convert NDC to view space via the inverse projection matrix
    vec4 view_space = inv_projection_matrix * vec4(normalized_device_coordinates, 1.0);	
    
	//linearize the depth
	view_space.xyz /= view_space.w;	
	
	// camera view points in the negative Z direction, so all depths are negative
	// we invert the sign here to get positive depth values
	return -view_space.z;
}

void fragment() {
	vec4 newAlbedo = albedo * texture(albedo_texture, UV) * vec4(COLOR.rgb, 1);
	
	
    float opacity = newAlbedo.a * alpha_mult;
    int x = int(FRAGCOORD.x) % 4;
    int y = int(FRAGCOORD.y) % 4;
    int index = x + y * 4;
    float limit = 0.0;
 
    if (x < 8) {
        if (index == 0) limit = 0.0625;
        if (index == 1) limit = 0.5625;
        if (index == 2) limit = 0.1875;
        if (index == 3) limit = 0.6875;
        if (index == 4) limit = 0.8125;
        if (index == 5) limit = 0.3125;
        if (index == 6) limit = 0.9375;
        if (index == 7) limit = 0.4375;
        if (index == 8) limit = 0.25;
        if (index == 9) limit = 0.75;
        if (index == 10) limit = 0.125;
        if (index == 11) limit = 0.625;
        if (index == 12) limit = 1.0;
        if (index == 13) limit = 0.5;
        if (index == 14) limit = 0.875;
        if (index == 15) limit = 0.375;
    }
    // Is this pixel below the opacity limit? Skip drawing it
    if (opacity < limit || opacity < alpha_clip)
        discard;
	
	vec3 alt = rgb2hsv(newAlbedo.rgb);
	float saturation = alt.y * 0.6 + 0.1;
	float value = alt.z * 0.9;
	if (collected) {
		saturation *= 1.4;
		value *= 1.2;
		value += 0.1;
	}
	alt.y = min(saturation, 1.0);
	alt.z = min(value, 1.0);
	
	float distanceToClip = length(CAMERA_POSITION_WORLD - katamariPos);
	float vLength = length(VERTEX);
	vLength += limit * katamariSize;
	bool test1 = !collected && vLength <= distanceToClip;
	vec4 katamariScreenSpacePos = vec4(katamariPos, 1.0);
	katamariScreenSpacePos = VIEW_MATRIX * katamariScreenSpacePos;
	katamariScreenSpacePos = PROJECTION_MATRIX * katamariScreenSpacePos;
	katamariScreenSpacePos = vec4(katamariScreenSpacePos.xyz / katamariScreenSpacePos.w, 0);
	katamariScreenSpacePos += vec4(1, 1, 0, 0);
	katamariScreenSpacePos *= 0.5;
	float correctedDist = length((SCREEN_UV - katamariScreenSpacePos.xy) * (VIEWPORT_SIZE / VIEWPORT_SIZE.x));
	correctedDist -= limit * 0.025;
	bool test2 = correctedDist < 0.2;
	vec4 worldSpacePixelPos = INV_VIEW_MATRIX * vec4(VERTEX, 1.0);
	bool test3 = worldSpacePixelPos.y >= katamariPos.y - katamariSize * 0.9;
	if (test1 && test2 && test3){
		discard;
	}
	if (vLength < katamariSize){
		discard;
	}
	
	//ALBEDO = vec3(correctedDist, correctedDist, correctedDist);
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

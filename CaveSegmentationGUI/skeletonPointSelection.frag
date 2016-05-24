#version 420 compatibility

layout(location=0) out vec4 result;

void main(void)
{
	vec2 coord = gl_PointCoord - vec2(0.5);
	float l = length(coord);
	if(l > 0.5 || l < 0.4)
		discard;
	result = (1 - pow(abs(0.45 - l) * 20, 2)) * vec4(0.9, 0.9, 0.6, 1.0);
}
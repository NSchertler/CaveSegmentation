#version 420 compatibility

uniform vec4 color;

out vec4 result;

void main(void)
{
	vec2 coord = gl_PointCoord - vec2(0.5);
	float l = length(coord);
	if(l > 0.5)
		discard;
	result = (1 - pow(abs(0.5 - l) * 2, 2)) * color;
}
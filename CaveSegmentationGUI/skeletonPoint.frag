#version 420 compatibility

flat in int primitiveId;
in vec4 color;

layout(location=0) out vec4 result;
layout(location=1) out int id;

void main(void)
{
	vec2 coord = gl_PointCoord - vec2(0.5);
	float l = length(coord);
	if(l > 0.5)
		discard;
	result = vec4((1.0 - l) * color.rgb, color.a);

	id = primitiveId;
}
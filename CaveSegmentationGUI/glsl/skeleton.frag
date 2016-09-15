#version 420 compatibility

layout(location=0) out vec4 result;

in vec2 tex;

void main(void)
{	
	result = vec4(0.5, 0.8, 0.5, 1.0) * (1.0 - abs(0.5 - tex.y));
	result.a = 1;
}
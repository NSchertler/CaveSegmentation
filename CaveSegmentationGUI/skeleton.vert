#version 420 compatibility

layout(location=0) in vec4 in_position;

uniform mat4 mvp;

void main(void)
{
	gl_Position = mvp * in_position;
}
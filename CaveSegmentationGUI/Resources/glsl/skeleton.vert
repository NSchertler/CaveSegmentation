#version 420 compatibility

layout(location=0) in vec4 in_position;

uniform mat4 mvp;

out vec4 position;

void main(void)
{
	position = mvp * in_position;
}
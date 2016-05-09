#version 420 compatibility

in vec3 in_position;

out vec4 position;

void main(void)
{
	position = vec4(in_position, 1.0);
}
#version 330 compatibility

layout(location=0) out vec4 color;
layout(location=1) out int id;

void main()
{
	color = vec4(0, 0.1f, 0.3f, 1.00);
	id = 0;
}
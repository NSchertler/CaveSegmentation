#version 420 compatibility

layout(location=0) out vec4 result;
layout(location=1) out int id;

void main(void)
{	
	result = vec4(0.5, 0.5, 0.5, 1.0);	
	id = 0;
}
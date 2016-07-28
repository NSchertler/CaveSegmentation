#version 420 compatibility

in vec3 color;
uniform bool transparent;

out vec4 c;

void main(void)
{
	if(transparent)
		c = vec4(color, 0.5);
	else
		c = vec4(color, 1.0);	
}
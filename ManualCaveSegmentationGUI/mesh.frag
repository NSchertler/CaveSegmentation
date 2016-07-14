#version 420 compatibility

in vec3 color;
uniform bool transparent;

void main(void)
{
	if(transparent)
		gl_FragColor = vec4(color, 0.5);
	else
		gl_FragColor = vec4(color, 1.0);	
}
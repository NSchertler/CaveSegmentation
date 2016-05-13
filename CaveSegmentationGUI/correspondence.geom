#version 420 compatibility
layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vec4 first[];
in vec4 second[];
in uint discarded[];

void main(void)
{
	if(discarded[0] != 0)
		return;

	gl_Position = first[0];
	EmitVertex();
	gl_Position = second[0];
	EmitVertex();
}
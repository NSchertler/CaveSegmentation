#version 420 compatibility
layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vec4 first[];
in vec4 second[];
in uint discarded[];

out vec4 color;

void main(void)
{
	if(discarded[0] != 0)
		return;

	color = vec4(0.6, 0.6, 0.6, 1);
	gl_Position = first[0];
	EmitVertex();

	color = vec4(0.9, 0.9, 0.9, 1);
	gl_Position = second[0];
	EmitVertex();
}
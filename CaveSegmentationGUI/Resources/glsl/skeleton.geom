#version 330 compatibility

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

in vec4 position[];
out vec2 tex;

void main()
{
	vec4 direction = position[1] - position[0];
	
	vec4 normal = normalize(vec4(-direction.y, direction.x, 0, 0));
	
	gl_Position = position[0] + 0.2 * normal; tex = vec2(0.0, 0.0); EmitVertex();
	gl_Position = position[0] - 0.2 * normal; tex = vec2(0.0, 1.0); EmitVertex();
	gl_Position = position[1] + 0.2 * normal; tex = vec2(1.0, 0.0); EmitVertex();
	gl_Position = position[1] - 0.2 * normal; tex = vec2(1.0, 1.0); EmitVertex();
}
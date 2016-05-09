#version 420 compatibility
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec4 position[];

out vec3 color;

uniform mat4 mvp;

void main(void)
{
	vec3 n = normalize(cross(position[1].xyz - position[0].xyz, position[2].xyz - position[0].xyz));
	color = (abs(dot(n, normalize(vec3(1,1,1)))) + 0.15) * vec3(1, 1, 1);
	
	for(int i = 0; i < 3; ++i)
	{
		gl_Position = mvp * position[i];
		EmitVertex();
	}	
}
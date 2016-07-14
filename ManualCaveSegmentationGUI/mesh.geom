#version 420 compatibility
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

uniform mat4 mvp;

in vec4 position[];
in vec3 segColor[];

out vec3 color;

void main(void)
{
	vec3 n = normalize(cross(position[1].xyz - position[0].xyz, position[2].xyz - position[0].xyz));
	float c = abs(dot(n, normalize(vec3(1,1,1)))) + 0.15;
	
	for(int i = 0; i < 3; ++i)
	{
		color = c * segColor[i];
		gl_Position = mvp * position[i];
		EmitVertex();
	}	
}
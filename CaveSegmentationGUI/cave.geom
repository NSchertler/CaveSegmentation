#version 420 compatibility
layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec4 position[];
in vec4 v_color[];

out vec4 color;

uniform mat4 mv;
uniform mat4 mvp;

void main(void)
{
	vec4 n = mv * vec4(normalize(cross(position[1].xyz - position[0].xyz, position[2].xyz - position[0].xyz)), 0);
	float diffuseFactor = 0.9 * max(0.1, abs(normalize(n.xyz).z));
	
	for(int i = 0; i < 3; ++i)
	{
		color = vec4(diffuseFactor * v_color[i].rgb, v_color[i].a);
		gl_Position = mvp * position[i];
		EmitVertex();
	}	
}
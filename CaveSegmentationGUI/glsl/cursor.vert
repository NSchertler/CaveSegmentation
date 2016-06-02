#version 330 core

const vec2 texCoords[4] = vec2[]( vec2(0.0, 1.0), vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0));

uniform vec4 posSize;
uniform float depth;

out vec2 tex;

void main()
{
	switch(gl_VertexID)
	{
		case 0: gl_Position = vec4(posSize.x, posSize.y, depth, 1.0); break;
		case 1: gl_Position = vec4(posSize.x, posSize.y - posSize.w, depth, 1.0); break;
		case 2: gl_Position = vec4(posSize.x + posSize.z, posSize.y, depth, 1.0); break;
		case 3: gl_Position = vec4(posSize.x + posSize.z, posSize.y - posSize.w, depth, 1.0); break;
	}
    tex = texCoords[gl_VertexID];
}
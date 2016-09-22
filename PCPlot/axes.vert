#version 330 compatibility

uniform mat4 mvp;

void main(void)
{
	vec4 worldPos;
	worldPos.x = float(gl_VertexID >> 1);
	worldPos.y = float(gl_VertexID & 1);
	worldPos.z = 0;
	worldPos.w = 1;
	gl_Position = mvp * worldPos;
}
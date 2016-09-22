#version 330 compatibility

out vec2 texCoord;

void main()
{
	gl_Position.x = float((gl_VertexID & 1) << 2) - 1;
	gl_Position.y = float((gl_VertexID & 2) << 1) - 1;
	gl_Position.z = 0;
	gl_Position.w = 1;

	texCoord.x = float((gl_VertexID & 1) << 1);
	texCoord.y = float((gl_VertexID & 2));
}
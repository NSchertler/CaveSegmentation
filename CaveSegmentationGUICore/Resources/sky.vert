#version 330 core

uniform mat4 mvp;

out vec4 clipPos;

void main()
{
    clipPos = vec4(		float((gl_VertexID << 1) & 2) - 1.0,
                        float((gl_VertexID +  1) & 2) - 1.0,
                        float( gl_VertexID       & 2) - 1.0,
                        1.0);

	gl_Position = mvp * clipPos;
	gl_Position.z = 0.5 * gl_Position.w;
}
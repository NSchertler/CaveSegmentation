#version 420 compatibility

layout(location=0) in vec4 in_position;
layout(location=1) in vec4 colorLayer;

uniform mat4 mvp;
uniform float viewportDiagonal;
uniform float sizeMultiplier;

flat out int primitiveId;
out vec4 color;

void main(void)
{
	gl_Position = mvp * in_position;
	gl_PointSize = sizeMultiplier * viewportDiagonal / gl_Position.w;

	color = colorLayer * vec4(0.5, 0.5, 0.5, 1.0);

	primitiveId = gl_VertexID + 1;		
}
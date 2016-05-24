#version 420 compatibility

uniform mat4 mvp;
uniform float viewportDiagonal;
uniform float sizeMultiplier;

uniform vec4 position;

void main(void)
{
	gl_Position = mvp * position;
	gl_PointSize = sizeMultiplier * viewportDiagonal / gl_Position.w;
}
#version 330 compatibility

out float color;

in float texCoord;
in float quality;

void main()
{
	float t = -0.5 * cos(texCoord * 6.283) + 0.5;

	color = t * quality;
}
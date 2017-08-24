#version 330 compatibility

uniform sampler2D sampler;

out vec4 color;

in vec2 tex;

void main()
{
	color = texture(sampler, tex);
}
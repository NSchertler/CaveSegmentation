#version 420 compatibility

layout(location=0) in vec4 in_position;
in vec4 segColor;

out vec4 position;
out vec4 v_color;

uniform bool hasSegmentation;

void main(void)
{
	position = in_position;
	v_color = hasSegmentation ? segColor : vec4(1, 1, 1, 1);
}
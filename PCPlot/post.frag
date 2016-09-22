#version 330

out vec4 color;

in vec2 texCoord;

uniform sampler2D tex;

void main()
{
	float d = texture2D(tex, texCoord).r;	
	//float d = 0.85 + 0.10 * texCoord.y;	
	//d = pow((d - 0.85) / 0.10, 5);
	color = vec4(1 - d, 1 - d, 1 - d, d);
}
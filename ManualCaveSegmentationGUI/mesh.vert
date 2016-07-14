#version 420 compatibility

in vec3 in_position;
in unsigned int segmentation;

out vec4 position;
out vec3 segColor;

void main(void)
{
	position = vec4(in_position, 1.0);

	switch(segmentation)
	{
		case 0: 
			segColor = vec3(0.5, 0.5, 0.5);
			break;
		case 1:
			segColor = vec3(0.7, 0.0, 0.0);
			break;
		case 2:
			segColor = vec3(0.0, 0.7, 0.0);
			break;
		default: 
			segColor = vec3(0.0, 0.0, 0.0);
			break;
	}
	//segColor = vec3(float(segmentation & 0xff000000u) / float(0xff000000u), float(segmentation & 0xff0000u) / float(0xff00u), float(segmentation & 0xff00u) / float(0xff00u));
}
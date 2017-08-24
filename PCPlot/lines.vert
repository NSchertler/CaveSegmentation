#version 430

struct AxisInfo
{	
	int offset;
	float min, max;
};

layout (std430, binding=0) buffer axesBuffer
{ 
  AxisInfo axes[];
};

layout (std430, binding=1) buffer samples
{ 
  float data[];
};

uniform mat4 mvp;
uniform int stride;
uniform int qualityOffset;
uniform int instanceOffset;

out float texCoord;
out float quality;

void main()
{
	int sampleId = gl_InstanceID + instanceOffset;
	texCoord = float(gl_VertexID & 1);
	float offset = texCoord * (-0.02) + 0.01;
	int axisId = gl_VertexID >> 1;	
	quality = data[stride * sampleId + qualityOffset];	
	//The mapping function from quality to color is hard-coded for simplicity
	quality = pow((quality - 0.99) / 0.01, 10); //mapping used in paper: pow((quality - 0.85) / 0.10, 5);
	float y = (data[stride * sampleId + axes[axisId].offset] - axes[axisId].min) / (axes[axisId].max - axes[axisId].min);
	gl_Position = mvp * vec4(float(axisId), y + offset, 0, 1);
}

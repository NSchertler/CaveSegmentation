#version 430 compatibility

layout(location=0) in vec4 in_position;
in uint in_correspondence;

layout (std430, binding=0) buffer skeleton_data
{ 
  vec4 positions[];
};

out vec4 first;
out vec4 second;
out uint discarded;

uniform mat4 mvp;
uniform int specificSkeletonVertex;

void main(void)
{
	discarded = (in_correspondence != specificSkeletonVertex && specificSkeletonVertex >= 0 ? 1 : 0);
		
	first = mvp * in_position;
	second = mvp * positions[in_correspondence];
}
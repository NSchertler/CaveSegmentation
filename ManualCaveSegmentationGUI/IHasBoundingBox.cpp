#include "IHasBoundingBox.h"


IHasBoundingBox::IHasBoundingBox(void)
{
	reset_bounding_box();
}

void IHasBoundingBox::reset_bounding_box()
{
	max = glm::vec3(-std::numeric_limits<float>::infinity());
	min = glm::vec3(std::numeric_limits<float>::infinity());
}


IHasBoundingBox::~IHasBoundingBox(void)
{
}

glm::vec3 IHasBoundingBox::getMin()
{
	return min;
}

glm::vec3 IHasBoundingBox::getMax()
{
	return max;
}

void IHasBoundingBox::add_point(glm::vec3 p) 
{
	if(p.x < min.x)
		min.x = p.x;
	if(p.x > max.x)
		max.x = p.x;
	if(p.y < min.y)
		min.y = p.y;
	if(p.y > max.y)
		max.y = p.y;
	if(p.z < min.z)
		min.z = p.z;
	if(p.z > max.z)
		max.z = p.z;
}
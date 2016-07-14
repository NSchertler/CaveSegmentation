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

void IHasBoundingBox::add_point(float x, float y, float z)
{
	if (x < min.x)
		min.x = x;
	if (x > max.x)
		max.x = x;
	if (y < min.y)
		min.y = y;
	if (y > max.y)
		max.y = y;
	if (z < min.z)
		min.z = z;
	if (z > max.z)
		max.z = z;
}

void IHasBoundingBox::add_point(glm::vec3 p)
{
	add_point(p.x, p.y, p.z);
}
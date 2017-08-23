#include "IHasBoundingBox.h"


IHasBoundingBox::IHasBoundingBox(void)
{
	reset_bounding_box();
}

void IHasBoundingBox::reset_bounding_box()
{
	max = Eigen::Vector3f(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
	min = Eigen::Vector3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
}


IHasBoundingBox::~IHasBoundingBox(void)
{
}

Eigen::Vector3f IHasBoundingBox::getMin()
{
	return min;
}

Eigen::Vector3f IHasBoundingBox::getMax()
{
	return max;
}

void IHasBoundingBox::add_point(float x, float y, float z)
{
	if (x < min.x())
		min.x() = x;
	if (x > max.x())
		max.x() = x;
	if (y < min.y())
		min.y() = y;
	if (y > max.y())
		max.y() = y;
	if (z < min.z())
		min.z() = z;
	if (z > max.z())
		max.z() = z;
}

void IHasBoundingBox::add_point(const Eigen::Vector3f& p)
{
	add_point(p.x(), p.y(), p.z());
}
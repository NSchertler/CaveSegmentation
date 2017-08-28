#include "BoundingBoxAccumulator.h"


BoundingBoxAccumulator::BoundingBoxAccumulator(void)
{
	ResetBoundingBox();
}

void BoundingBoxAccumulator::ResetBoundingBox()
{
	max = Eigen::Vector3f(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
	min = Eigen::Vector3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
}

const Eigen::Vector3f& BoundingBoxAccumulator::GetMin() const
{
	return min;
}

const Eigen::Vector3f& BoundingBoxAccumulator::GetMax() const
{
	return max;
}

void BoundingBoxAccumulator::AddPoint(float x, float y, float z)
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

void BoundingBoxAccumulator::AddPoint(const Eigen::Vector3f& p)
{
	AddPoint(p.x(), p.y(), p.z());
}
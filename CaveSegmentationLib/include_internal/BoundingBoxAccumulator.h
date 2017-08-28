#pragma once

#include <Eigen/Dense>

#include "IHasBoundingBox.h"

class BoundingBoxAccumulator : public virtual IHasBoundingBox
{
public:
	BoundingBoxAccumulator();

	const Eigen::Vector3f& GetMin() const;
	const Eigen::Vector3f& GetMax() const;

	void AddPoint(const Eigen::Vector3f& position);
	void AddPoint(float x, float y, float z);
	void ResetBoundingBox();

private:
	Eigen::Vector3f min, max;
};
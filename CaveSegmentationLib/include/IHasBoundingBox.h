#pragma once

#include <Eigen/Dense>
#include <CaveSegmentationLib.h>

class CAVESEGMENTATIONLIB_API IHasBoundingBox
{
public:
	virtual const Eigen::Vector3f& GetMin() const = 0;
	virtual const Eigen::Vector3f& GetMax() const = 0;
};


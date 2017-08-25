#pragma once

#include <Eigen/Dense>
#include <CaveSegmentationLib.h>

class CAVESEGMENTATIONLIB_API IHasBoundingBox
{
public:
	IHasBoundingBox(void);
	~IHasBoundingBox(void);

	Eigen::Vector3f getMin();
	Eigen::Vector3f getMax();

private:
	Eigen::Vector3f min, max;

protected:
	void add_point(const Eigen::Vector3f& position);
	void add_point(float x, float y, float z);
	void reset_bounding_box();
};


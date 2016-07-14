#pragma once

#include "cavesegmentationguicore_global.h"

#include <glm/glm.hpp>

class CAVESEGMENTATIONGUICORE_EXPORT IHasBoundingBox
{
public:
	IHasBoundingBox(void);
	~IHasBoundingBox(void);

	glm::vec3 getMin();
	glm::vec3 getMax();

private:
	glm::vec3 min, max;

protected:
	void add_point(glm::vec3 position);
	void add_point(float x, float y, float z);
	void reset_bounding_box();
};


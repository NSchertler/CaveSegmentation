#pragma once

#include <vector>
#include <Eigen/Dense>
#include <IndexedTriangle.h>

class IMesh
{
public:
	virtual const std::vector<Eigen::Vector3f>& MeshVertices() const = 0;
	virtual const std::vector<IndexedTriangle>& MeshTriIndices() const = 0;
};
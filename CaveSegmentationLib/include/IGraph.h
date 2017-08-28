#pragma once

#include "CaveSegmentationLib.h"

#include <vector>

class CAVESEGMENTATIONLIB_API IGraph
{
public:

	//Returns the skeleton vertices that are adjacent to the given vertex.
	virtual const std::vector<int>& AdjacentNodes(size_t skeletonVertex) const = 0;

	//Returns the id of the edge between two vertices.
	virtual size_t EdgeIdFromVertexPair(size_t v1, size_t v2) const = 0;

	virtual void IncidentVertices(size_t edgeId, size_t& v1, size_t& v2) const = 0;

	virtual const Eigen::Vector3f& VertexPosition(size_t vertexId) const = 0;

	//Returns the average half distance of a node to its neighbors.
	virtual double NodeRadius(size_t vertexId) const = 0;

	virtual size_t NumberOfVertices() const = 0;
	virtual size_t NumberOfEdges() const = 0;
};
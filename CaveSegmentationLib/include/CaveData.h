#pragma once

#include <vector>
#include <CurveSkeleton.h>

struct CaveData
{
	CaveData(CurveSkeleton* skeleton, unsigned int meshVertexCount)
		: skeleton(skeleton),
		meanDistances (skeleton->vertices.size()),
		maxDistances (skeleton->vertices.size()),
		minDistances (skeleton->vertices.size()),
		caveSizes (skeleton->vertices.size()),
		nodeRadii (skeleton->vertices.size()),
		meshVertexCorrespondsTo (meshVertexCount),
		parents (skeleton->vertices.size()),
		children (skeleton->vertices.size()),
		caveSizeDerivatives (skeleton->vertices.size()),
		caveSizeCurvatures (skeleton->vertices.size()),
		caveScale(skeleton->vertices.size()),
		caveSizeDerivativesPerEdge (skeleton->edges.size()),
		caveSizeCurvaturesPerEdge (skeleton->edges.size()),
		adjacency (skeleton->vertices.size())
	{

	}
	
	CurveSkeleton* skeleton;
	std::vector<double> meanDistances;
	std::vector<double> maxDistances;
	std::vector<double> minDistances;
	std::vector<double> caveSizes;
	std::vector<double> nodeRadii;
	std::vector<unsigned int> meshVertexCorrespondsTo;
	std::vector<int> parents;
	std::vector<std::vector<int>> children;
	std::vector<double> caveSizeDerivatives;
	std::vector<double> caveSizeCurvatures;
	std::vector<double> caveScale; //average cave size in the area surrounded by the vertex
	std::vector<double> caveSizeDerivativesPerEdge;
	std::vector<double> caveSizeCurvaturesPerEdge;
	std::vector<std::vector<int>> adjacency;
	std::map<std::pair<int, int>, int> vertexPairToEdge;
};
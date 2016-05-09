#pragma once

#include <vector>
#include <CurveSkeleton.h>

#include "CGALCommon.h"
#include "MeshProc.h"

#include "SizeCalculation.h"
#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"

struct CaveData
{
	CaveData();

	virtual void LoadMesh(const std::string& offFile);
	void SetSkeleton(CurveSkeleton* skeleton);

	void CalculateDistances();
	void LoadDistances(const std::string& file);
	void SaveDistances(const std::string& file) const;
	void SmoothAndDeriveDistances();

	void WriteBranchStatistics(const std::string& directory) const;

	void ResizeMeshAttributes(size_t vertexCount);
	void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount);
	
	RegularUniformSphereSampling sphereSampling;
	std::unique_ptr<CaveSizeCalculator> caveSizeCalculator;

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
	int rootVertex;	

	SphereVisualizer sphereVisualizer;

	const std::vector<Eigen::Vector3f>& meshVertices() { return _meshVertices; }
	const TriangleList& meshTriangles(){ return _meshTriangles; }
	const std::vector<IndexedTriangle>& meshTriIndices(){ return _meshTriIndices; }
	const Tree& meshAABBTree(){ return _meshAABBTree; }

protected:
	//Calculates basic derived data from the stored skeleton, such as adjacency, node radii, etc.
	void CalculateBasicSkeletonData();

	std::vector<Eigen::Vector3f> _meshVertices;
	TriangleList _meshTriangles;
	std::vector<IndexedTriangle> _meshTriIndices;
	Tree _meshAABBTree;
};
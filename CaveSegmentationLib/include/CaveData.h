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

	template <typename TSphereVisualizer = VoidSphereVisualizer>
	bool CalculateDistancesSingleVertex(int iVert, float exponent, std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient);
	template <typename TSphereVisualizer = VoidSphereVisualizer>
	bool CalculateDistancesSingleVertex(int iVert, float exponent = 1.0f);
	bool CalculateDistances(float exponent = 1.0f);
	void LoadDistances(const std::string& file);
	void SaveDistances(const std::string& file) const;
	void SmoothAndDeriveDistances();

	void SetOutputDirectory(const std::wstring& outputDirectory);

	void WriteBranchStatistics(const std::string& directory) const;

	void ResizeMeshAttributes(size_t vertexCount);
	void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount);

	void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation);
	
	RegularUniformSphereSampling sphereSampling;
	typedef CaveSizeCalculatorLineFlow CaveSizeCalculator;
	std::vector<CaveSizeCalculator::TCustomData> caveSizeCalculatorCustomData;

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

	std::vector<double> caveSizeUnsmoothed;

	const std::vector<Eigen::Vector3f>& meshVertices() const { return _meshVertices; }
	const TriangleList& meshTriangles(){ return _meshTriangles; }
	const std::vector<IndexedTriangle>& meshTriIndices(){ return _meshTriIndices; }
	const Tree& meshAABBTree(){ return _meshAABBTree; }

	//Returns a 3-component RGB array
	const int* GetSegmentColor(int segmentIndex);

	std::vector<int> invalidVertices; //a list of vertices that did not have valid distances before reconstruction

	enum Algorithm
	{
		Max,
		Smooth,
		Advect
	} CAVE_SCALE_ALGORITHM;
	double CAVE_SCALE_KERNEL_FACTOR; //kernel deviation for calculating cave scale (multiplied by local cave size)
	double CAVE_SIZE_KERNEL_FACTOR; //kernel deviation for smoothing cave size (multiplied by cave scale)
	double CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR; //kernel deviation for smoothing cave size derivative (multiplied by cave scale)	

protected:
	//Calculates basic derived data from the stored skeleton, such as adjacency, node radii, etc.
	void CalculateBasicSkeletonData();

	std::vector<Eigen::Vector3f> _meshVertices;
	TriangleList _meshTriangles;
	std::vector<IndexedTriangle> _meshTriIndices;
	Tree _meshAABBTree;

	std::wstring outputDirectoryW;
};
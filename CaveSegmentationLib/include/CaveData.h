#pragma once

#include <vector>
#include <CurveSkeleton.h>

#include "CGALCommon.h"
#include "MeshProc.h"
#include "IHasBoundingBox.h"

#include "SizeCalculation.h"
#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"

//Agglomeration of all cave-related data.
struct CaveData : public IHasBoundingBox
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

	void ResizeMeshAttributes(size_t vertexCount);
	void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount);

	void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation);
	
	RegularUniformSphereSampling sphereSampling;
	typedef CaveSizeCalculatorLineFlow CaveSizeCalculator;
	std::vector<CaveSizeCalculator::TCustomData> caveSizeCalculatorCustomData;

	CurveSkeleton* skeleton;
	//Mean radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> meanDistances;
	//Maximum radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> maxDistances;
	//Minimum radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> minDistances;

	//Cave size for a given skeleton vertex
	std::vector<double> caveSizes;

	//Radius of a skeleton vertex based on the distance to its neighbor vertices.
	std::vector<double> nodeRadii;

	//Correspondence of the mesh vertices to skeleton vertices
	std::vector<unsigned int> meshVertexCorrespondsTo;

	//Unsmoothed cave size per skeleton vertex as calculated by the size calculator
	std::vector<double> caveSizeUnsmoothed;

	//Calculated skeleton properties	
	std::vector<double> caveScale;
	std::vector<double> caveSizeDerivativesPerEdge;
	std::vector<double> caveSizeCurvaturesPerEdge;

	//Adjacency list of the skeleton
	std::vector<std::vector<int>> adjacency;
	//Maps a pair of vertices to an edge index for the ...PerEdge vectors
	std::map<std::pair<int, int>, int> vertexPairToEdge;
	
	const std::vector<Eigen::Vector3f>& meshVertices() const { return _meshVertices; }
	const TriangleList& meshTriangles() const { return _meshTriangles; }
	const std::vector<IndexedTriangle>& meshTriIndices() const { return _meshTriIndices; }
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
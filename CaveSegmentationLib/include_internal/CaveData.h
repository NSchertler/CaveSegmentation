#pragma once

#include <vector>

#include "ICaveData.h"

#include "CGALCommon.h"
#include "IndexedTriangle.h"
#include "SizeCalculation.h"
#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"
#include "MeshProc.h"
#include "BoundingBoxAccumulator.h"

#pragma warning(disable: 4250) //inherits via dominance

//Agglomeration of all cave-related data.
struct CaveData : public virtual ICaveData, public BoundingBoxAccumulator
{
	CaveData();

	void LoadMesh(const std::string& offFile);
	void WriteMesh(const std::string& offFile, std::function<void(int i, int& r, int& g, int& b)> colorFunc) const;
	void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation) const;
	void SetSkeleton(CurveSkeleton* skeleton);

	bool CalculateDistances(float exponent = 1.0f);
	bool CalculateDistancesSingleVertexWithDebugOutput(int iVert, float exponent = 1.0f);
	void LoadDistances(const std::string& file);
	void SaveDistances(const std::string& file) const;
	void SmoothAndDeriveDistances();
	bool HasUnsmoothedCaveSizes() const { return caveSizeUnsmoothed.size() != 0; };
	bool HasCaveSizes() const { return caveSizes.size() != 0; }

	void SetOutputDirectory(const std::wstring& outputDirectory);

	void ResizeMeshAttributes(size_t vertexCount);
	void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount);	

	const std::vector<int>& AdjacentNodes(size_t skeletonVertex) const { return adjacency[skeletonVertex]; }

	const CurveSkeleton* Skeleton() const { return skeleton; }

	size_t EdgeIdFromVertexPair(size_t v1, size_t v2) const { return vertexPairToEdge.at(std::make_pair(v1, v2)); }

	void IncidentVertices(size_t edgeId, size_t& v1, size_t& v2) const
	{
		v1 = skeleton->edges[edgeId].first;
		v2 = skeleton->edges[edgeId].second;
	}

	double CaveSize(size_t iVertex) const { return caveSizes.at(iVertex); }
	double CaveSizeUnsmoothed(size_t iVertex) const { return caveSizeUnsmoothed.at(iVertex); }
	double CaveScale(size_t iVertex) const { return caveScale.at(iVertex); }
	double CaveSizeDerivative(size_t iEdge) const { return caveSizeDerivativesPerEdge.at(iEdge); }
	double CaveSizeCurvature(size_t iEdge) const { return caveSizeCurvaturesPerEdge.at(iEdge); }

	const Eigen::Vector3f& VertexPosition(size_t vertexId) const { return skeleton->vertices[vertexId].position; };

	double NodeRadius(size_t vertexId) const { return nodeRadii.at(vertexId); }

	size_t NumberOfVertices() const { return skeleton->vertices.size(); };
	size_t NumberOfEdges() const { return skeleton->edges.size(); };

	size_t MeshVertexCorrespondsTo(size_t meshVertex) const { return meshVertexCorrespondsTo.at(meshVertex); };

	const std::vector<Eigen::Vector3f>& MeshVertices() const { return _meshVertices; }
	const std::vector<IndexedTriangle>& MeshTriIndices() const { return _meshTriIndices; }

	ICaveData::Algorithm& CaveScaleAlgorithm() { return CAVE_SCALE_ALGORITHM; }
	double& CaveScaleKernelFactor() { return CAVE_SCALE_KERNEL_FACTOR; }
	double& CaveSizeKernelFactor() { return CAVE_SIZE_KERNEL_FACTOR; }
	double& CaveSizeDerivativeKernelFactor() { return CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR; }

	const std::vector<size_t>& VerticesWithInvalidSize() const { return invalidVertices; }
protected:

	template <typename TSphereVisualizer = VoidSphereVisualizer>
	bool CalculateDistancesSingleVertex(int iVert, float exponent = 1.0f);

	template <typename TSphereVisualizer = VoidSphereVisualizer>
	bool CalculateDistancesSingleVertex(int iVert, float exponent, std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient);

	//Calculates basic derived data from the stored skeleton, such as adjacency, node radii, etc.
	void CalculateBasicSkeletonData();

	RegularUniformSphereSampling sphereSampling;
	typedef CaveSizeCalculatorLineFlow CaveSizeCalculator;
	std::vector<CaveSizeCalculator::TCustomData> caveSizeCalculatorCustomData;

	//Cave size for a given skeleton vertex
	std::vector<double> caveSizes;
	//Unsmoothed cave size per skeleton vertex as calculated by the size calculator
	std::vector<double> caveSizeUnsmoothed;

	//Calculated skeleton properties	
	std::vector<double> caveScale;
	std::vector<double> caveSizeDerivativesPerEdge;
	std::vector<double> caveSizeCurvaturesPerEdge;

	CurveSkeleton* skeleton;
	//Mean radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> meanDistances;
	//Maximum radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> maxDistances;
	//Minimum radius of the visible sphere around a skeleton vertex; not used anymore.
	std::vector<double> minDistances;

	//Radius of a skeleton vertex based on the distance to its neighbor vertices.
	std::vector<double> nodeRadii;

	//Correspondence of the mesh vertices to skeleton vertices
	std::vector<unsigned int> meshVertexCorrespondsTo;

	//Adjacency list of the skeleton
	std::vector<std::vector<int>> adjacency;
	//Maps a pair of vertices to an edge index for the ...PerEdge vectors
	std::map<std::pair<size_t, size_t>, size_t> vertexPairToEdge;
	
	const TriangleList& meshTriangles() const { return _meshTriangles; }	
	const Tree& meshAABBTree() { return _meshAABBTree; }	

	std::vector<size_t> invalidVertices; //a list of vertices that did not have valid distances before reconstruction

	ICaveData::Algorithm CAVE_SCALE_ALGORITHM;
	double CAVE_SCALE_KERNEL_FACTOR; //kernel deviation for calculating cave scale (multiplied by local cave size)
	double CAVE_SIZE_KERNEL_FACTOR; //kernel deviation for smoothing cave size (multiplied by cave scale)
	double CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR; //kernel deviation for smoothing cave size derivative (multiplied by cave scale)	


	std::vector<Eigen::Vector3f> _meshVertices;
	TriangleList _meshTriangles;
	std::vector<IndexedTriangle> _meshTriIndices;
	Tree _meshAABBTree;

	std::wstring outputDirectoryW;
};
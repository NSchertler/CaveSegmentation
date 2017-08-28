#pragma once

#include "CaveSegmentationLib.h"
#include "IHasBoundingBox.h"
#include "IGraph.h"
#include "IMesh.h"

#include <CurveSkeleton.h>
#include <functional>
#include <memory>

class CAVESEGMENTATIONLIB_API ICaveData : public virtual IHasBoundingBox, public virtual IGraph, public virtual IMesh
{
public:

	enum Algorithm
	{
		Max,
		Smooth,
		Advect
	};

	virtual void LoadMesh(const std::string& offFile) = 0;
	virtual void WriteMesh(const std::string& offFile, std::function<void(int i, int& r, int& g, int& b)> colorFunc) const = 0;
	virtual void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation) const = 0;
	virtual void SetSkeleton(CurveSkeleton* skeleton) = 0;
	
	//Calculates the cave distances and sizes for every vertex and returns if there are any vertices with invalid sizes.
	virtual bool CalculateDistances(float exponent = 1.0f) = 0;

	//Calculates the cave distances and size for a single vertex and writes debug output to disk. Returns if
	//size calculation has been successful.
	virtual bool CalculateDistancesSingleVertexWithDebugOutput(int iVert, float exponent = 1.0f) = 0;

	
	virtual void LoadDistances(const std::string& file) = 0;
	virtual void SaveDistances(const std::string& file) const = 0;
	virtual void SmoothAndDeriveDistances() = 0;

	virtual bool HasUnsmoothedCaveSizes() const = 0;
	virtual bool HasCaveSizes() const = 0;
	virtual const std::vector<size_t>& VerticesWithInvalidSize() const = 0;

	//Specifies the output directory into which all results will be written.
	virtual void SetOutputDirectory(const std::wstring& outputDirectory) = 0;

	virtual void ResizeMeshAttributes(size_t vertexCount) = 0;
	virtual void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount) = 0;	

	//Returns the id of the skeleton vertex that corresponds to a surface vertex.
	virtual size_t MeshVertexCorrespondsTo(size_t meshVertex) const = 0;

	//Returns the underlying curve skeleton.
	virtual const CurveSkeleton* Skeleton() const = 0;

	virtual Algorithm& CaveScaleAlgorithm() = 0;
	virtual double& CaveScaleKernelFactor() = 0; //kernel deviation for calculating cave scale (multiplied by local cave size)
	virtual double& CaveSizeKernelFactor() = 0; //kernel deviation for smoothing cave size (multiplied by cave scale)
	virtual double& CaveSizeDerivativeKernelFactor() = 0; //kernel deviation for smoothing cave size derivative (multiplied by cave scale)	

	virtual double CaveSize(size_t iVertex) const = 0;
	virtual double CaveSizeUnsmoothed(size_t iVertex) const = 0;
	virtual double CaveScale(size_t iVertex) const = 0;
	virtual double CaveSizeDerivative(size_t iEdge) const = 0;
	virtual double CaveSizeCurvature(size_t iEdge) const = 0;

	//Returns the default color for a segment as a 3-component RGB array.
	static const int* GetSegmentColor(int segmentIndex);

	//Initialize the cave segmentation system
	static void StartCaveSeg();

	//De-initialize the cave segmentation system
	static void StopCaveSeg();
};

extern CAVESEGMENTATIONLIB_API std::shared_ptr<ICaveData> CreateCaveData();
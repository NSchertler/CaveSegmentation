#pragma once

#include "CaveSegmentationLib.h"
#include "IHasBoundingBox.h"
#include "IGraph.h"

#include <CurveSkeleton.h>

class CAVESEGMENTATIONLIB_API ICaveData : public IHasBoundingBox, public IGraph
{
public:
	virtual void LoadMesh(const std::string& offFile) = 0;
	virtual void SetSkeleton(CurveSkeleton* skeleton) = 0;
	
	//Calculates the cave distances and sizes for every vertex and returns if there are any vertices with invalid sizes.
	virtual bool CalculateDistances(float exponent = 1.0f) = 0;

	//Calculates the cave distances and size for a single vertex and writes debug output to disk. Returns if
	//size calculation has been successful.
	virtual bool CalculateDistancesSingleVertexWithDebugOutput(int iVert, float exponent = 1.0f) = 0;

	
	virtual void LoadDistances(const std::string& file) = 0;
	virtual void SaveDistances(const std::string& file) const = 0;
	virtual void SmoothAndDeriveDistances() = 0;

	//Specifies the output directory into which all results will be written.
	virtual void SetOutputDirectory(const std::wstring& outputDirectory) = 0;

	virtual void ResizeMeshAttributes(size_t vertexCount) = 0;
	virtual void ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount) = 0;

	virtual void WriteSegmentationColoredOff(const std::string& path, const std::vector<int32_t>& segmentation) = 0;	

	double CaveSize(size_t iVertex) const { return caveSizes.at(iVertex); }
	double CaveSizeUnsmoothed(size_t iVertex) const { return caveSizeUnsmoothed.at(iVertex); }
	double CaveScale(size_t iVertex) const { return caveScale.at(iVertex); }
	double CaveSizeDerivative(size_t iEdge) const { return caveSizeDerivativesPerEdge.at(iEdge); }
	double CaveSizeCurvature(size_t iEdge) const { return caveSizeCurvaturesPerEdge.at(iEdge); }

	//Returns the underlying curve skeleton.
	virtual const CurveSkeleton* Skeleton() const = 0;

protected:
	//Cave size for a given skeleton vertex
	std::vector<double> caveSizes;
	//Unsmoothed cave size per skeleton vertex as calculated by the size calculator
	std::vector<double> caveSizeUnsmoothed;

	//Calculated skeleton properties	
	std::vector<double> caveScale;
	std::vector<double> caveSizeDerivativesPerEdge;
	std::vector<double> caveSizeCurvaturesPerEdge;
};

extern CAVESEGMENTATIONLIB_API ICaveData* CreateCaveData();
extern CAVESEGMENTATIONLIB_API void DestroyCaveData(ICaveData*);
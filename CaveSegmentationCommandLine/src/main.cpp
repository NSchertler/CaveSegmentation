#include "Options.h"

#include "ImageProc.h"
#include "CGALCommon.h"

#include "RegularUniformSphereSampling.h"
#include "FileInputOutput.h"
#include "MeshProc.h"
#include "LineProc.h"
#include "ChamberAnalyzation/CurvatureBasedAStar.h"
#include "ChamberAnalyzation/CurvatureBasedQPBO.h"
#include "ChamberAnalyzation/Utils.h"
#include "SizeCalculation.h"
#include "GraphProc.h"
#include "CaveData.h"

#include <iostream>
#include <CurveSkeleton.h>
#include <Eigen/Dense>
#include <map>
#include <stack>
#include <unordered_set>
#include <queue>
#include <boost/filesystem.hpp>

#include <fstream>

int main(int argc, char* argv[])
{	
	if (argc < 2)
	{
		std::cout << "Usage: CaveSegmentationCommandLine [dataDirectory]" << std::endl;
		std::cout << "The data directory must contain a \"model.off\"." << std::endl;
#ifdef CALC_SKELETON
		std::cout << "The skeleton will be saved in the data directory under \"model.skel\"." << std::endl;
#else
		std::cout << "The data directory must contain a \"model.skel\"." << std::endl;
#endif
#ifdef CALC_DISTANCES
		std::cout << "Distance data will be saved in the data directory under \"distances.bin\"." << std::endl;
#else
		std::cout << "The data directory must contain a \"distances.bin\"." << std::endl;
#endif
		std::cout << "All output will be saved in \"[dataDirectory]\\output\"." << std::endl;
	}

	const std::string dataDirectory = argv[1];

	const std::string outputDirectory = dataDirectory + "/output";

	std::wstring outputDirectoryW(outputDirectory.length(), L' ');
	std::copy(outputDirectory.begin(), outputDirectory.end(), outputDirectoryW.begin());
	
	boost::filesystem::path outputDir(outputDirectory);
	boost::filesystem::create_directory(outputDir);

	const std::string offFile = dataDirectory + "/model.off";
	const std::string skeletonFile = dataDirectory + "/model.skel";
	const std::string distancesFile = dataDirectory + "/distances.bin";

	const std::string calculatedSkeletonFile = outputDirectory + "/skeleton.obj";
	const std::string calculatedSkeletonCorrFile = outputDirectory + "/skeletonCorr.obj";
	const std::string segmentationFile = outputDirectory + "/segmentation.seg";

	CaveData data;
	data.LoadMesh(offFile);	
	data.SetOutputDirectory(outputDirectoryW);

	StartImageProc();	

#ifdef CALC_SKELETON
	std::cout << "Calculating skeleton..." << std::endl;
	//CurveSkeleton* skeleton = ComputeCurveSkeleton(offFile, 1.0, 0.8, 1.5);
	CurveSkeleton* skeleton = ComputeCurveSkeleton(offFile, 1.0, 1.0, 20, 1.0);
	skeleton->Save(skeletonFile.c_str());
	skeleton->SaveToObj(calculatedSkeletonFile.c_str());
	skeleton->SaveToObjWithCorrespondences(calculatedSkeletonCorrFile.c_str(), offFile);
	data.SetSkeleton(skeleton);
#else
	std::cout << "Loading skeleton..." << std::endl;
	CurveSkeleton* skeleton = LoadCurveSkeleton(skeletonFile.c_str());	
	data.SetSkeleton(skeleton);
#endif
	
	
#ifdef CALC_DISTANCES
	data.CalculateDistances();

	data.SaveDistances(distancesFile);
#else
	data.LoadDistances(distancesFile);	
#endif
	data.SmoothAndDeriveDistances();
	


#ifdef WRITE_BRANCH_STATISTICS
	data.WriteBranchStatistics(outputDirectory);
#endif
	
	std::cout << "Finding chambers..." << std::endl;

	std::vector<int> segmentation;
	CurvatureBasedQPBO::FindChambers(data, segmentation);

	AssignUniqueChamberIndices(data, segmentation);
	
	WriteSegmentation(segmentationFile, segmentation);

	int colors[10][3] = 
	{
		{166, 206, 227},
		{31,120,180},
		{251,154,153},
		{227,26,28},
		{253,191,111},
		{255,127,0},
		{202,178,214},
		{106,61,154},
		{255,255,153},
		{177,89,40}
	};

	auto colorFunc = [&](int i, int& r, int& g, int& b)
	{
		if (segmentation[i] < 0)
		{
			r = 0; g = 175; b = 0;
		}
		else
		{
			int* color = colors[segmentation[i] % 10];
			r = color[0];
			g = color[1];
			b = color[2];
		}
		/* curvature visualization
		r = g = b = 100;	
		if (data.caveSizeCurvatures[i] > 0)
		{
			r += (int)(data.caveSizeCurvatures[i] * 155 / 0.2);
			if (r > 255)
				r = 255;
		}*/
		/* size visualization 
		r = g = b = (int)(data.caveSizes[i] * 255 / 30); */
		/* entrance prob visualization 
		r = g = b = 0;
		double prob = entranceProbability(data.caveSizeCurvatures[i]);
		if (prob > 0.5)
			r = 128 + 255 * (prob - 0.5);
		else
			b = 128 + 255 * prob;*/
		
	};
	auto colorVertexFunc = [&](const CurveSkeleton::Vertex& v, int i, int& r, int& g, int& b){ colorFunc(i, r, g, b); };

	std::string corrsFile = outputDirectory + "/Correspondences.obj";
	std::string skeletonObjFile = outputDirectory + "/Skeleton.obj";
	std::string skeletonHeightCodedFile = outputDirectory + "/SkeletonWithHeightCodedSize.obj";
	std::string segmentedMeshFile = outputDirectory + "/segmentedMesh.off";

	skeleton->SaveToObjWithCorrespondences(corrsFile.c_str(), offFile, colorVertexFunc);

	skeleton->SaveToObj(skeletonObjFile.c_str(), colorVertexFunc);
	skeleton->SaveToObj(skeletonHeightCodedFile.c_str(), colorVertexFunc, [&](const CurveSkeleton::Vertex& v, int i, float& newX, float& newY, float& newZ) { newX = v.position.x(), newY = v.position.y(), newZ = (float)data.caveSizes.at(i); });

	WriteOff(segmentedMeshFile.c_str(), data.meshVertices(), data.meshTriIndices(), [&](int i, int& r, int& g, int& b) {colorFunc(data.meshVertexCorrespondsTo[i], r, g, b); } );

	DestroySkeleton(skeleton);

	StopImageProc();

	return 0;
}
#include "Options.h"

#include "ImageProc.h"
#include "CGALCommon.h"

#include "CaveData.h"
#include "ChamberAnalyzation/CurvatureBasedQPBO.h"
#include "ChamberAnalyzation/Utils.h"
#include "FileInputOutput.h"

#include <boost/filesystem.hpp>

#include <iostream>
#include <CurveSkeleton.h>

void PrintHelp()
{
	std::cout << "Usage: CaveSegmentationCommandLine [options] [dataDirectory]" << std::endl;
	std::cout << "Options: " << std::endl;
	std::cout << "\t-d [dataDirectory]    The data directory must contain a \"model.off\"." << std::endl;
	std::cout << "\t                      The skeleton location is \"[dataDirectory]/model.skel\"." << std::endl;
	std::cout << "\t                      The distance data location is \"[dataDirectory]/distances.bin\"." << std::endl;
	std::cout << "\t--calcSkel            Specify this to calculate the skeleton and save it to file. Otherwise, it will be loaded from a file." << std::endl;
	std::cout << "\t--calcDist            Specify this to calculate distance data and save them to file. Otherwise, they will be loaded from a file." << std::endl;
	std::cout << "\t--edgeLength [float]  Specify the edge collapse threshold for skeleton calculation in percent of the model's bounding box diagonal." << std::endl;
	std::cout << "\t--wsmooth [float]    Specify the smoothing weight for skeleton calculation." << std::endl;
	std::cout << "\t--wvelocity [float]  Specify the velocity weight for skeleton calculation." << std::endl;
	std::cout << "\t--wmedial [float]    Specify the medial weight for skeleton calculation." << std::endl;
	std::cout << "All output will be saved in \"[dataDirectory]/output\"." << std::endl;
}

int main(int argc, char* argv[])
{	
	bool calculateSkeleton = false;
	bool calculateDistances = false;
	std::string dataDirectory;

	float edgeCollapseThresholdPercent = 1.0f;
	float w_smooth = 1.0f;
	float w_velocity = 20.0f;
	float w_medial = 1.0f;

	if (argc < 2)
	{
		PrintHelp();

		return -1;
	}
	else
	{
		for (int i = 1; i < argc; ++i)
		{
			if (strcmp(argv[i], "-d") == 0)
			{
				dataDirectory = std::string(argv[i + 1]);
				++i;
			}
			else if (strcmp(argv[i], "--calcSkel") == 0)
				calculateSkeleton = true;
			else if (strcmp(argv[i], "--calcDist") == 0)
				calculateDistances = true;
			else if (strcmp(argv[i], "--edgeLength") == 0)
			{
				edgeCollapseThresholdPercent = std::stof(argv[i + 1]);
				++i;
			}
			else if (strcmp(argv[i], "--wsmooth") == 0)
			{
				w_smooth = std::stof(argv[i + 1]);
				++i;
			}
			else if (strcmp(argv[i], "--wvelocity") == 0)
			{
				w_velocity = std::stof(argv[i + 1]);
				++i;
			}
			else if (strcmp(argv[i], "--wmedial") == 0)
			{
				w_medial = std::stof(argv[i + 1]);
				++i;
			}
		}
	}

	if (dataDirectory.empty())
	{
		std::cout << "You did not specify a data directory." << std::endl;
		PrintHelp();
		return -2;
	}

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

	std::cout << "Model file: " << offFile << std::endl;

	CaveData data;
	data.LoadMesh(offFile);	
	data.SetOutputDirectory(outputDirectoryW);

	StartImageProc();	

	CurveSkeleton* skeleton;
	if (calculateSkeleton)
	{
		std::cout << "Calculating skeleton..." << std::endl;
		AbortHandle abort;
		float edgeCollapseThreshold = edgeCollapseThresholdPercent * (data.getMax() - data.getMin()).norm() / 100.0f;
		std::cout << "Edge collapse threshold: " << edgeCollapseThreshold << std::endl;
		std::cout << "w_smooth: " << w_smooth << std::endl;
		std::cout << "w_velocity: " << w_velocity << std::endl;
		std::cout << "w_medial: " << w_medial << std::endl;
		skeleton = ComputeCurveSkeleton(offFile, &abort, edgeCollapseThreshold, w_smooth, w_velocity, w_medial);
		std::cout << "Saving skeleton to " << skeletonFile << std::endl;
		skeleton->Save(skeletonFile.c_str());
		std::cout << "Saving skeleton OBJ to " << calculatedSkeletonFile << std::endl;
		skeleton->SaveToObj(calculatedSkeletonFile.c_str());
		std::cout << "Saving skeleton OBJ with correspondences to " << calculatedSkeletonCorrFile << std::endl;
		skeleton->SaveToObjWithCorrespondences(calculatedSkeletonCorrFile.c_str(), offFile);
	}
	else
	{
		std::cout << "Loading skeleton from " << skeletonFile << std::endl;
		skeleton = LoadCurveSkeleton(skeletonFile.c_str());		
	}
	data.SetSkeleton(skeleton);
	
	
	if (calculateDistances)
	{
		std::cout << "Calculating distances..." << std::endl;
		data.CalculateDistances();

		std::cout << "Saving distances to " << distancesFile << std::endl;
		data.SaveDistances(distancesFile);
	}
	else
	{
		std::cout << "Loading distances from " << distancesFile << std::endl;
		data.LoadDistances(distancesFile);
	}
	std::cout << "Smoothing and deriving distances..." << std::endl;
	data.SmoothAndDeriveDistances();
	


#ifdef WRITE_BRANCH_STATISTICS
	data.WriteBranchStatistics(outputDirectory);
#endif
	
	std::cout << "Finding chambers..." << std::endl;

	std::vector<int> segmentation;
	CurvatureBasedQPBO::FindChambers(data, segmentation);

	AssignUniqueChamberIndices(data, segmentation);
	
	std::cout << "Writing segmentation to " << segmentationFile << std::endl;
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

	std::cout << "Saving skeleton with colored correspondences to " << corrsFile << std::endl;
	skeleton->SaveToObjWithCorrespondences(corrsFile.c_str(), offFile, colorVertexFunc);

	std::cout << "Saving colored skeleton to " << skeletonObjFile << std::endl;
	skeleton->SaveToObj(skeletonObjFile.c_str(), colorVertexFunc);
	std::cout << "Saving skeleton with cave size encoded as z-coordinates to " << skeletonHeightCodedFile << std::endl;
	skeleton->SaveToObj(skeletonHeightCodedFile.c_str(), colorVertexFunc, [&](const CurveSkeleton::Vertex& v, int i, float& newX, float& newY, float& newZ) { newX = v.position.x(), newY = v.position.y(), newZ = (float)data.caveSizes.at(i); });

	std::cout << "Saving segmented model to " << segmentedMeshFile << std::endl;
	WriteOff(segmentedMeshFile.c_str(), data.meshVertices(), data.meshTriIndices(), [&](int i, int& r, int& g, int& b) {colorFunc(data.meshVertexCorrespondsTo[i], r, g, b); } );

	DestroySkeleton(skeleton);

	StopImageProc();

	return 0;
}
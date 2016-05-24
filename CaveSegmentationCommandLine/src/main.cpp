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


/*#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/triangulate_polyhedron.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Union_find.h>*/


int main()
{	
	//const std::string dataDirectory = "data/SmallCaveDownsampled";
	const std::string dataDirectory = "data/SimudPuteh";

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
	data.sphereVisualizer.SetOutputDirectory(outputDirectoryW);

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
	// Reposition skeleton vertices in center of correspondences
	/*for (auto& vertex : skeleton->vertices)
	{
		Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
		Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
		for (int corr : vertex.correspondingOriginalVertices)
		{
			centroid += vertices.at(corr);
		}
		centroid *= 1.0f / vertex.correspondingOriginalVertices.size();

		for (int corr : vertex.correspondingOriginalVertices)
		{
			Eigen::Vector3f corrDir = vertices.at(corr) - centroid;
			covariance += corrDir * corrDir.transpose();
		}
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullV);
		Eigen::Matrix3f toLocal = svd.matrixV();
		for (int i = 0; i < 3; ++i)
			toLocal.row(0).normalize();

		Eigen::Vector3f localMin(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
		Eigen::Vector3f localMax(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
		for (int corr : vertex.correspondingOriginalVertices)
		{
			Eigen::Vector3f local = toLocal * (vertices.at(corr) - centroid);
			for (int i = 0; i < 3; ++i)
			{
				if (local(i) < localMin(i))
					localMin(i) = local(i);
				if (local(i) > localMax(i))
					localMax(i) = local(i);
			}
		}
		Eigen::Vector3f localCenter = (localMin + localMax) * 0.5f;
		Eigen::Vector3f globalCenter = toLocal.transpose() * localCenter + centroid;
		vertex.position = globalCenter;
	}
	skeleton->SaveToObj(calculatedSkeletonFile.c_str());
	skeleton->SaveToObjWithCorrespondences(calculatedSkeletonCorrFile.c_str(), offFile);
	return 0;*/	
	
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

	/*std::cout << "Extracting halls..." << std::endl;

	
	typedef CGAL::Polyhedron_3<K> Polyhedron; //CGAL::Exact_predicates_inexact_constructions_kernel
	Polyhedron poly;		
	poly.delegate(BuildCavePolyhedron<Polyhedron::HalfedgeDS>(argmin, meshVertexCorrespondsTo, vertices, triIndices));
	std::cout << "Valid poly: " << poly.is_valid() << std::endl;

	std::cout << "Filling holes..." << std::endl;

	poly.normalize_border();
	while (poly.size_of_border_edges() > 0) {
		// get the first hole we can find, and close it
		poly.fill_hole(poly.border_halfedges_begin()->opposite());

		// renormalize mesh so that halfedge iterators are again valid
		poly.normalize_border();
	}

	CGAL::triangulate_polyhedron<Polyhedron>(poly);
	std::cout << "Border edges: " << poly.size_of_border_edges() << std::endl;

	std::cout << "Writing OFF..." << std::endl;
	std::ofstream off("CavesWithFilledHoles.off");
	off << poly;
	off.close();

	std::cout << "Finished." << std::endl;

	return 0;*/
}
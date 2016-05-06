#include "Options.h"

#include "ImageProc.h"
#include "CGALCommon.h"

#include "RegularUniformSphereSampling.h"
#include "FileInputOutput.h"
#include "MeshProc.h"
#include "LineProc.h"
#include "ChamberAnalyzation/CurvatureBasedAStar.h"
#include "ChamberAnalyzation/CurvatureBasedQPBO.h"
#include "SphereVisualizer.h"
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


void CalculateGradient(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient)
{
	for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
	{
		double currentDistance = sphereSampling.AccessContainerData(sphereDistances, it);
		Vector currentPos = *it;

		//parameters of the linear system
		// | a  b  c | | x |   | g |
		// | b  d  e | | y | = | h |
		// | c  e  f | | z |   | i |
		double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0, h = 0, i = 0;

		for (auto neighbor : sphereSampling.Neighbors(it))
		{
			Vector neighborPos = *neighbor;
			double neighborDistance = sphereSampling.AccessContainerData(sphereDistances, neighbor);
			Vector direction = neighborPos - currentPos;
			double dirLength = sqrt(direction.squared_length());
			direction = direction * (1.0 / dirLength);
			double derivative = (neighborDistance - currentDistance) / dirLength;

			a += direction.x() * direction.x();
			b += direction.x() * direction.y();
			c += direction.x() * direction.z();
			d += direction.y() * direction.y();
			e += direction.y() * direction.z();
			f += direction.z() * direction.z();
			g += direction.x() * derivative;
			h += direction.y() * derivative;
			i += direction.z() * derivative;
		}

		double denom = c * c * d - 2 * b * c * e + a * e * e + b * b * f - a * d * f;
		Vector gradient(
			(e * e * g - d * f * g - c * e * h + b * f * h + c * d * i - b * e * i) / denom,
			(b * f * g - c * e * g + c * c * h - a * f * h - b * c * i + a * e * i) / denom,
			(c * d * g - b * e * g - b * c * h + a * e * h + b * b * i - a * d * i) / denom);

		//project gradient on sphere surface
		gradient = gradient - (gradient * currentPos) * currentPos;

		if (isnan(gradient.x()))
		{
			std::cout << "g is nan." << std::endl;
			system("PAUSE");
		}

		sphereSampling.AccessContainerData(distanceGradient, it) = gradient;
	}
}

void FindStrongLocalExtrema(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<double>>& sphereDistances, double extremumSearchRadius, std::vector<PositionValue>& maxima, std::vector<PositionValue>& minima, SphereVisualizer& visualizer)
{
	for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
	{
		double dist = sphereSampling.AccessContainerData(sphereDistances, it);
			
		bool isLocalMaximum = true;
		bool isLocalMinimum = true;
		if (isLocalMaximum || isLocalMinimum)
			for (auto neighbor : sphereSampling.Neighbors(*it, extremumSearchRadius))
			{
				double d = sphereSampling.AccessContainerData(sphereDistances, neighbor);
				if (dist < d)
					isLocalMaximum = false;
				if (dist > d)
					isLocalMinimum = false;
				if (!isLocalMaximum && !isLocalMinimum)
					break;
			}

		if (isLocalMaximum)
			maxima.push_back({ *it, dist });
		if (isLocalMinimum)
			minima.push_back({ *it, dist });


#ifdef DRAW_DEBUG_IMAGES
		double x, y, w, h;
		BYTE c = (BYTE)(std::min(255.0, dist * 255 / 20));
		BYTE r = c, g = c, b = c;
		if (isLocalMaximum)
		{
			g >>= 1;
			b >>= 1;
		}
		if (isLocalMinimum)
		{
			r >>= 1;
			g >>= 1;
		}
		it.GetParameterSpaceRect(x, y, w, h);
		x *= imWidth / (2 * M_PI);
		w *= imWidth / (2 * M_PI);
		y *= imHeight / M_PI;
		h *= imHeight / M_PI;

		double myX = x + w / 2;
		double myY = y + h / 2;

		visualizer.FillRect((int)x, (int)y, (int)ceil(x + w - (int)x), (int)ceil(y + h - (int)y), Gdiplus::Color(r, g, b));
		if (x < 0)
			visualizer.FillRect((int)(x + imWidth), (int)y, (int)ceil(w), (int)ceil(h), Gdiplus::Color(r, g, b));
		//visualizer.FillCircle((int)myX, (int)myY, 2, SphereVisualizer::SAMPLE_COLOR);		
#endif 			
	}
}

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

	typedef std::vector<Triangle> TriangleList;	
	std::vector<Eigen::Vector3f> vertices;
	TriangleList triangles;
	std::vector<IndexedTriangle> triIndices;

	std::cout << "Reading file..." << std::endl;
	ReadOff(offFile, vertices, triangles, triIndices);

	StartImageProc();

	// constructs AABB tree 
	Tree tree(triangles.begin(),triangles.end());
		
	RegularUniformSphereSampling sphereSampling(SPHERE_SAMPLING_RESOLUTION);	
	std::unique_ptr<CaveSizeCalculator> caveSizeCalculator = std::unique_ptr<CaveSizeCalculator>(new CaveSizeCalculatorLineFlow());

#ifdef CALC_SKELETON
	std::cout << "Calculating skeleton..." << std::endl;
	//CurveSkeleton* skeleton = ComputeCurveSkeleton(offFile, 1.0, 0.8, 1.5);
	CurveSkeleton* skeleton = ComputeCurveSkeleton(offFile, 1.0, 1.0, 20, 1.0);
	skeleton->Save(skeletonFile.c_str());
	skeleton->SaveToObj(calculatedSkeletonFile.c_str());
	skeleton->SaveToObjWithCorrespondences(calculatedSkeletonCorrFile.c_str(), offFile);
	return 0;
#else
	std::cout << "Loading skeleton..." << std::endl;
	CurveSkeleton* skeleton = LoadCurveSkeleton(skeletonFile.c_str());	
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

	CaveData data(skeleton, vertices.size());

	std::cout << "Calculating adjacency list..." << std::endl;
	
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		auto& edge = skeleton->edges.at(iEdge);
		data.adjacency[edge.first].push_back(edge.second);
		data.adjacency[edge.second].push_back(edge.first);
		data.vertexPairToEdge[edge] = iEdge;
		data.vertexPairToEdge[std::pair<int, int>(edge.second, edge.first)] = iEdge;
	}	

	//find root
	int rootVertex = -1;
	for (int v = 0; v < skeleton->vertices.size(); ++v)
		if (data.adjacency[v].size() == 1)
		{
			rootVertex = v;
			break;
		}
	
	buildTree(rootVertex, skeleton->vertices.size(), data.adjacency, data.parents, data.children);

	//calculate correspondences and node radii
	int iVert = -1;	
	for (auto& vert : skeleton->vertices)
	{
		++iVert;

		auto& adj = data.adjacency.at(iVert);
		for (auto c : vert.correspondingOriginalVertices)
		{
			data.meshVertexCorrespondsTo[c] = iVert;
		}

		double nodeRadius = 0.0;
		std::vector <int> correspondences;
		correspondences.insert(correspondences.end(), vert.correspondingOriginalVertices.begin(), vert.correspondingOriginalVertices.end());
		for (auto adjV : adj)
		{
			auto& v = skeleton->vertices.at(adjV);
			correspondences.insert(correspondences.end(), v.correspondingOriginalVertices.begin(), v.correspondingOriginalVertices.end());
			nodeRadius += (vert.position - v.position).norm() / 2.0;
		}
		data.nodeRadii.at(iVert) = nodeRadius / adj.size();
	}

#ifdef CALC_DISTANCES
	std::vector<std::vector<double>> sphereDistances;
	std::vector<std::vector<Vector>> distanceGradient;
	sphereSampling.PrepareDataContainer(sphereDistances);
	sphereSampling.PrepareDataContainer(distanceGradient);	
	
	std::cout << "Calculating distances..." << std::endl;

	iVert = -1;
	for (auto& vert : skeleton->vertices)
	{
		++iVert;	
		//if (iVert != 69) continue;
		std::cout << "\rProcessing skeleton vertex " << iVert;
		
		SphereVisualizer sphereVisualizer;

#ifdef WRITE_SPHERE_VIS
		auto sphereVisFilename = outputDirectory + "/sphereVis" + std::to_string(iVert) + ".obj";
		std::ofstream sphereVis(sphereVisFilename.c_str());
#endif

		int n = 0;
		double meanSphereDistance = 0.0;
		double maxSphereDistance = 0.0;
		double minSphereDistance = std::numeric_limits<double>::infinity();
		double M2 = 0.0;

		const double MAXIMUM_SEARCH_RADIUS = 0.5;

		//Record distances
		for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
		{
			double visDist = sqrt(GetSqrDistanceToMesh(Point(vert.position.x(), vert.position.y(), vert.position.z()), *it, tree));
			if (isinf(visDist))
			{
				std::cout << "Skeleton lies outside of mesh!" << std::endl;
				system("PAUSE");
			}
			sphereSampling.AccessContainerData(sphereDistances, it) = visDist;

			++n;
			double delta = visDist - meanSphereDistance;
			meanSphereDistance += delta / n;
			M2 += delta * (visDist - meanSphereDistance);
			if (visDist > maxSphereDistance)
				maxSphereDistance = visDist;
			if (visDist < minSphereDistance)
				minSphereDistance = visDist;

#ifdef WRITE_SPHERE_VIS
			auto p = *it;
			sphereVis << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
#endif
		}

		double variance = n < 2 ? 0 : M2 / (n - 1);
		data.maxDistances.at(iVert) = maxSphereDistance;
		data.minDistances.at(iVert) = minSphereDistance;
		data.meanDistances.at(iVert) = meanSphereDistance;

	#ifdef WRITE_SPHERE_STATS
		auto sphereStatsFilename = outputDirectory + "/sphereStats" + std::to_string(iVert) + ".csv";
		std::ofstream sphereStats(sphereStatsFilename.c_str());
		sphereStats.imbue(std::locale("de-DE"));
	#endif

		
	#ifdef WRITE_SPHERE_STATS
		sphereStats << vert.position.x() << ";" << vert.position.y() << ";" << vert.position.z() << std::endl;		
	#endif		

	#ifdef WRITE_HEIGHTFIELD
		auto heightFieldName = outputDirectory + "/heightField" + std::to_string(iVert) + ".obj";
		std::ofstream heightField(heightFieldName.c_str());
		heightField << "mtllib ./heightfield.mtl" << std::endl;
		heightField << "usemtl Default_Smoothing" << std::endl;

		auto heightFieldSphereName = outputDirectory + "/heightFieldSphere" + std::to_string(iVert) + ".obj";
		std::ofstream heightFieldSphere(heightFieldSphereName.c_str());
		heightFieldSphere << "mtllib ./heightfield.mtl" << std::endl;
		heightFieldSphere << "usemtl Default_Smoothing" << std::endl;

		int sample_resolution_x = sphereSampling.MaxNTheta();
		for (int y = 0; y < SPHERE_SAMPLING_RESOLUTION; ++y)
		{
			double phi = y * M_PI / (SPHERE_SAMPLING_RESOLUTION - 1);
			for (int x = 0; x < sample_resolution_x; ++x)
			{
				double theta = x * 2 * M_PI / sample_resolution_x;
				double height = sphereSampling.AccessInterpolatedContainerData(sphereDistances, phi, theta) / 10;
				heightField << "v " << theta << " " << phi << " " << height << std::endl;
				heightField << "vt " << theta / 2 / M_PI << " " << 1 - phi / M_PI << std::endl;
				Vector p = sphereSampling.Point(phi, theta) * height;
				heightFieldSphere << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
				heightFieldSphere << "vt " << theta / 2 / M_PI << " " << 1 - phi / M_PI << std::endl;
				if (y > 0)
				{
					int tl = 1 + (y - 1) * sample_resolution_x + (x);
					int tr = 1 + (y - 1) * sample_resolution_x + (x + 1) % sample_resolution_x;
					int bl = 1 + (y)* sample_resolution_x + (x);
					int br = 1 + (y)* sample_resolution_x + (x + 1) % sample_resolution_x;
					if (x < sample_resolution_x - 1)
					{
						heightField << "f " << tl << "/" << tl << " " << tr << "/" << tr << " " << bl << "/" << bl << std::endl;
						heightField << "f " << bl << "/" << bl << " " << tr << "/" << tr << " " << br << "/" << br << std::endl;
					}
					heightFieldSphere << "f " << tl << "/" << tl << " " << tr << "/" << tr << " " << bl << "/" << bl << std::endl;
					heightFieldSphere << "f " << bl << "/" << bl << " " << tr << "/" << tr << " " << br << "/" << br << std::endl;
				}
			}
		}
		heightField.close();
		heightFieldSphere.close();
	#endif				

		//calculate gradient
		CalculateGradient(sphereSampling, sphereDistances, distanceGradient);
		
		std::vector<PositionValue> sphereDistanceMaxima, sphereDistanceMinima;
		FindStrongLocalExtrema(sphereSampling, sphereDistances, MAXIMUM_SEARCH_RADIUS, sphereDistanceMaxima, sphereDistanceMinima, sphereVisualizer);
		
		sphereVisualizer.Save(outputDirectoryW + L"/distanceField" + std::to_wstring(iVert) + L".png");

		//sphereVisualizer.DrawGradientField(sphereSampling, distanceGradient);		

		data.caveSizes.at(iVert) = caveSizeCalculator->CalculateDistance(sphereSampling, sphereDistances, distanceGradient, sphereDistanceMaxima, sphereDistanceMinima, sphereVisualizer, iVert, outputDirectoryW);

		sphereVisualizer.Save(outputDirectoryW + L"/sphereVis" + std::to_wstring(iVert) + L".png");

	#ifdef WRITE_SPHERE_STATS
		sphereStats.close();
	#endif

	#ifdef WRITE_SPHERE_VIS
		sphereVis.close();
	#endif									
	}

	std::cout << std::endl;

	std::ofstream distanceFile(distancesFile.c_str(), std::ios::binary);
	distanceFile.write(reinterpret_cast<const char*>(&data.maxDistances[0]), sizeof(double) * data.maxDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&data.minDistances[0]), sizeof(double) * data.minDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&data.meanDistances[0]), sizeof(double) * data.meanDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&data.caveSizes[0]), sizeof(double) * data.caveSizes.size());
	distanceFile.close();
#else
	std::cout << "Loading distances from file..." << std::endl;

	std::ifstream distanceFile(distancesFile.c_str(), std::ios::binary);
	distanceFile.read(reinterpret_cast<char*>(&data.maxDistances[0]), sizeof(double) * data.maxDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&data.minDistances[0]), sizeof(double) * data.minDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&data.meanDistances[0]), sizeof(double) * data.meanDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&data.caveSizes[0]), sizeof(double) * data.caveSizes.size());
	distanceFile.close();
#endif


	std::cout << "Smoothing distances..." << std::endl;

	std::vector<double> smoothWorkDouble(std::max(skeleton->vertices.size(), skeleton->edges.size()));

	//Calculate cave scale by smoothing with a very large window
	smooth(skeleton->vertices, data.adjacency, [&data](int iVert) { return 10.0 * data.caveSizes.at(iVert); }, data.caveSizes, data.caveScale);

	//Derive per vertex
	smooth(skeleton->vertices, data.adjacency, [&data](int iVert) { return 0.2 * data.caveScale.at(iVert); }, data.caveSizes, smoothWorkDouble);
	memcpy(&data.caveSizes[0], &smoothWorkDouble[0], data.caveSizes.size() * sizeof(double));

	derive(rootVertex, skeleton->vertices, data.children, data.caveSizes, smoothWorkDouble);
	smooth(skeleton->vertices, data.adjacency, [&data](int iVert) { return 0.2 * data.caveScale.at(iVert); }, smoothWorkDouble, data.caveSizeDerivatives);
		
	derive(rootVertex, skeleton->vertices, data.children, data.caveSizeDerivatives, data.caveSizeCurvatures);

	//Derive per edge
	derivePerEdgeFromVertices(skeleton, data.caveSizes, smoothWorkDouble);
	smoothPerEdge<double, true>(skeleton, data.adjacency, data.vertexPairToEdge, [&data](int iEdge)
		{
			auto edge = data.skeleton->edges.at(iEdge);
			return 0.2 * 0.5 * (data.caveScale.at(edge.first) + data.caveScale.at(edge.second));
		}, smoothWorkDouble, data.caveSizeDerivativesPerEdge);

	derivePerEdge<double, true>(skeleton, data.adjacency, data.vertexPairToEdge, data.caveSizeDerivativesPerEdge, data.caveSizeCurvaturesPerEdge);


#ifdef WRITE_BRANCH_STATISTICS
	std::cout << "Writing branch statistics..." << std::endl;

	std::stack<int> branchStarts;
	branchStarts.push(rootVertex);

	std::ofstream f;
	f.open(outputDirectory + "/Branches.csv");
	f.imbue(std::locale(""));

	int iBranch = 0;
	while (!branchStarts.empty())
	{
		auto currentV = branchStarts.top();
		branchStarts.pop();

		f << "New branch" << std::endl;
		f << "ID;t;mean distance;min distance;max distance;cave size;cave scale;1st derivative;2nd derivative;t;1st derivative per edge;2nd derivative per edge" << std::endl;
		CurveSkeleton s;
		s.vertices.push_back(skeleton->vertices[currentV]);

		int lastV = -1;

		double dist = 0;
		while (currentV > 0)
		{
			int nextV = -1;

			f << currentV << ";" << dist << "; " << data.meanDistances[currentV] << ";" << data.minDistances[currentV] << ";" << data.maxDistances[currentV] << ";" << data.caveSizes[currentV] << "; " << data.caveScale[currentV] << "; " << data.caveSizeDerivatives[currentV] << "; " << data.caveSizeCurvatures[currentV];

			if (lastV != -1)
			{
				int edge = data.vertexPairToEdge.at(std::pair<int, int>(lastV, currentV));
				double inv = 1.0;
				//for direction dependent measures...
				if (lastV != skeleton->edges.at(edge).first)
					inv = -1.0;
				f << ";" << (dist - (data.skeleton->vertices.at(lastV).position - data.skeleton->vertices.at(currentV).position).norm() / 2) 
					<< "; " << data.caveSizeDerivativesPerEdge.at(edge) * inv
					<< "; " << data.caveSizeCurvaturesPerEdge.at(edge);
			}

			f << std::endl;
			
			for (auto& adj : data.children.at(currentV))
			{
				if (nextV == -1)
				{
					//Continue to the first next child
					nextV = adj;
					dist += (skeleton->vertices.at(nextV).position - skeleton->vertices.at(currentV).position).norm();
					s.vertices.push_back(skeleton->vertices.at(nextV));
					s.edges.push_back(std::make_pair((int)s.vertices.size() - 2, (int)s.vertices.size() - 1));
				}
				else
					//Push every other child on the stack
					branchStarts.push(adj);
			}
			lastV = currentV;
			currentV = nextV;
		}
		f << std::endl;
#ifdef EXPORT_BRANCHES
		std::stringstream ss;
		ss << outputDirectory << "/Branch" << (iBranch++) << ".obj";
		s.SaveToObjWithCorrespondences(ss.str().c_str(), offFile);
#endif
	}

	f.close();
#endif
	
	std::cout << "Finding chambers..." << std::endl;

	std::vector<int> segmentation;
	CurvatureBasedQPBO::FindChambers(data, segmentation);

	//assign unique chamber indices
	int nextSegment = 0;
	std::vector<bool> assignedNewIndex(segmentation.size(), false);
	for (int i = 0; i < segmentation.size(); ++i)
	{
		if (segmentation[i] < 0) //passage
			continue;
		if (assignedNewIndex[i])
			continue;

		std::stack<int> traversalStack;
		traversalStack.push(i);
		while (!traversalStack.empty())
		{
			int currentVertex = traversalStack.top();
			traversalStack.pop();
			if (segmentation[currentVertex] < 0)
				continue;
			if (assignedNewIndex[currentVertex])
				continue;
			assignedNewIndex[currentVertex] = true;
			segmentation[currentVertex] = nextSegment;
			for (auto n : data.adjacency[currentVertex])
				traversalStack.push(n);
		}
		++nextSegment;
	}

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

	WriteOff(segmentedMeshFile.c_str(), vertices, triIndices, [&](int i, int& r, int& g, int& b) {colorFunc(data.meshVertexCorrespondsTo[i], r, g, b); } );

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
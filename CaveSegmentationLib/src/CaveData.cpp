#include "Options.h"

#include "CaveData.h"
#include "FileInputOutput.h"
#include "GraphProc.h"
#include "ImageProc.h"

#include <stack>
#include <deque>

#include <boost/filesystem/operations.hpp>

//SDF
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <iostream>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel SDFKernel;
typedef CGAL::Polyhedron_3<SDFKernel> SDFPolyhedron;

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

template <typename TSphereVisualizer>
void FindStrongLocalExtrema(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<double>>& sphereDistances, double extremumSearchRadius, std::vector<PositionValue>& maxima, std::vector<PositionValue>& minima, TSphereVisualizer& visualizer)
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

		double myX = x + w / 2;
		double myY = y + h / 2;

		visualizer.FillRect(x, y, w, h, Gdiplus::Color(r, g, b));		
		//visualizer.FillCircle(myX, myY, 2, SphereVisualizer::SAMPLE_COLOR);					
	}
}

CaveData::CaveData()
	: skeleton(nullptr),
	  sphereSampling(SPHERE_SAMPLING_RESOLUTION),
	  CAVE_SCALE_KERNEL_FACTOR(10.0), CAVE_SIZE_KERNEL_FACTOR(0.2), CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR(0.2), CAVE_SCALE_ALGORITHM(CaveData::Max)
{
}

void CaveData::LoadMesh(const std::string & offFile)
{
	//Check if a cache for this file exists
	std::string cachePath = offFile + ".cache";
	bool loadFromCache = false;
	auto lastModifiedTimeOfMesh = boost::filesystem::last_write_time(offFile);
	if (boost::filesystem::exists(cachePath))
	{
		//check if the cache has the most recent version		
		FILE* cache = fopen(cachePath.c_str(), "rb");
		if (cache)
		{
			std::time_t cacheTime;
			fread(&cacheTime, sizeof(std::time_t), 1, cache);
			if (cacheTime == lastModifiedTimeOfMesh)
			{
				//Cache is up-to-date. Read it.

				std::cout << "Loading from cache instead of OFF..." << std::endl;

				_meshVertices.clear();
				_meshTriangles.clear();
				_meshTriIndices.clear();

				int32_t n_vertices;
				fread(&n_vertices, sizeof(int32_t), 1, cache);
				if (n_vertices > 0)
				{
					_meshVertices.resize(n_vertices);
					fread(&_meshVertices[0], sizeof(Eigen::Vector3f), n_vertices, cache);
				}

				int32_t n_triangles;
				fread(&n_triangles, sizeof(int32_t), 1, cache);
				if (n_triangles > 0)
				{
					_meshTriIndices.resize(n_triangles);
					_meshTriangles.resize(n_triangles);
					fread(&_meshTriIndices[0], sizeof(IndexedTriangle), n_triangles, cache);

					for (int i = 0; i < n_triangles; ++i)
					{
						auto& tri = _meshTriIndices.at(i);
						auto& v1 = _meshVertices.at(tri.i[0]);
						auto& v2 = _meshVertices.at(tri.i[1]);
						auto& v3 = _meshVertices.at(tri.i[2]);

						_meshTriangles.at(i) = Triangle(Point(v1.x(), v1.y(), v1.z()), Point(v2.x(), v2.y(), v2.z()), Point(v3.x(), v3.y(), v3.z()));
					}
				}

				fclose(cache);
				loadFromCache = true;
			}
			else
				std::cout << "Cannot read from cache." << std::endl;
		}
	}

	if(!loadFromCache)
		ReadOff(offFile, _meshVertices, _meshTriangles, _meshTriIndices);
	ResizeMeshAttributes(_meshVertices.size());

	_meshAABBTree.clear();
	_meshAABBTree.insert(_meshTriangles.begin(), _meshTriangles.end());
	_meshAABBTree.build();

	if (!loadFromCache)
	{
		//Write the cache
		FILE* cache = fopen(cachePath.c_str(), "wb");
		if (cache)
		{
			fwrite(&lastModifiedTimeOfMesh, sizeof(std::time_t), 1, cache);

			int32_t n_vertices = _meshVertices.size();
			fwrite(&n_vertices, sizeof(int32_t), 1, cache);
			if(n_vertices > 0)
				fwrite(&_meshVertices[0], sizeof(Eigen::Vector3f), n_vertices, cache);

			int32_t n_triangles = _meshTriIndices.size();
			fwrite(&n_triangles, sizeof(int32_t), 1, cache);
			if (n_triangles > 0)
				fwrite(&_meshTriIndices[0], sizeof(IndexedTriangle), n_triangles, cache);

			fclose(cache);
		}
		else
		{
			std::cout << "Cannot write cache." << std::endl;
		}
	}	

	//SDF test
	//std::cout << "Calculating SDF segmentation .." << std::endl;

	//// create and read Polyhedron
	//SDFPolyhedron mesh;
	//std::ifstream input(offFile);
	//if ( !input || !(input >> mesh) || mesh.empty() )
	//{
	//	std::cerr << "Not a valid off file." << std::endl;  
	//	return;
	//}    
	//// create a property-map for segment-ids   
	//typedef std::map<SDFPolyhedron::Facet_const_handle, std::size_t> Facet_int_map;  
	//Facet_int_map internal_segment_map;  
	//boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);    
	//// calculate SDF values and segment the mesh using default parameters.    
	//std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(mesh, segment_property_map, 2 * M_PI / 3, 23u, 5u, 0.5); 
	//std::cout << "Number of segments: " << number_of_segments << std::endl;    
	//// print segment-ids    
	//std::ofstream coff(offFile + ".sdf.off");
	//coff << "COFF" << std::endl;
	//coff << 3 * mesh.size_of_facets() << " " << mesh.size_of_facets() << " 0" << std::endl;
	//
	//for(SDFPolyhedron::Facet_const_iterator facet_it = mesh.facets_begin();
	//	facet_it != mesh.facets_end(); ++facet_it) 
	//{   
	//	auto v_it = facet_it->facet_begin();
	//	do
	//	{
	//		int segment = segment_property_map[facet_it];
	//		const int* color = GetSegmentColor(segment);	
	//		auto p = v_it->vertex()->point();
	//		coff << p.x() << " " << p.y() << " " << p.z() << " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
	//	} while (++v_it != facet_it->facet_begin());
	//}
	//for (int i = 0; i < mesh.size_of_facets(); ++i)
	//{
	//	coff << "3 " << 3 * i << " " << 3 * i + 1 << " " << 3 * i + 2 << std::endl;
	//}

	//coff.close();
	//std::cout << std::endl;
	//std::cout << "done." << std::endl;
}

void CaveData::SetSkeleton(CurveSkeleton * skeleton)
{
	this->skeleton = skeleton;
	if (skeleton)
	{
		ResizeSkeletonAttributes(skeleton->vertices.size(), skeleton->edges.size());
		CalculateBasicSkeletonData();
	}
	else
		ResizeSkeletonAttributes(0, 0);
}

template <typename TSphereVisualizer>
bool CaveData::CalculateDistancesSingleVertex(int iVert, float exponent)
{
	std::vector<std::vector<double>> sphereDistances;
	std::vector<std::vector<Vector>> distanceGradient;
	sphereSampling.PrepareDataContainer(sphereDistances);
	sphereSampling.PrepareDataContainer(distanceGradient);

	caveSizeCalculatorCustomData.resize(1);

	return CalculateDistancesSingleVertex<TSphereVisualizer>(iVert, exponent, sphereDistances, distanceGradient);
}

template bool CaveData::CalculateDistancesSingleVertex<SphereVisualizer>(int iVert, float exponent);
template bool CaveData::CalculateDistancesSingleVertex<VoidSphereVisualizer>(int iVert, float exponent);

template <typename TSphereVisualizer>
bool CaveData::CalculateDistancesSingleVertex(int iVert, float exponent, std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient)
{
	auto& vert = skeleton->vertices.at(iVert);

#ifdef WRITE_SPHERE_VIS
	auto sphereVisFilename = outputDirectoryW + L"/sphereVis" + std::to_wstring(iVert) + L".obj";
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
		double visDist = sqrt(GetSqrDistanceToMesh(Point(vert.position.x(), vert.position.y(), vert.position.z()), *it, _meshAABBTree));
		if (isinf(visDist))
		{
#ifndef NON_VERBOSE
#pragma omp critical
			{
				std::cout << "Skeleton vertex " << iVert << " lies outside of mesh!" << std::endl;
			}
#endif

			maxDistances.at(iVert) = std::numeric_limits<double>::quiet_NaN();
			minDistances.at(iVert) = std::numeric_limits<double>::quiet_NaN();
			meanDistances.at(iVert) = std::numeric_limits<double>::quiet_NaN();
			caveSizes.at(iVert) = std::numeric_limits<double>::quiet_NaN();
			caveSizeUnsmoothed.at(iVert) = std::numeric_limits<double>::quiet_NaN();

			return false;
		}
		sphereSampling.AccessContainerData(sphereDistances, it) = pow(visDist, exponent);

		++n;
		double delta = visDist - meanSphereDistance;
		meanSphereDistance += delta / n;
		M2 += delta * (visDist - meanSphereDistance);
		if (visDist > maxSphereDistance)
			maxSphereDistance = visDist;
		if (visDist < minSphereDistance)
			minSphereDistance = visDist;

#ifdef WRITE_SPHERE_VIS
		auto p = visDist * *it;
		sphereVis << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
#endif
	}

	double variance = n < 2 ? 0 : M2 / (n - 1);
	maxDistances.at(iVert) = maxSphereDistance;
	minDistances.at(iVert) = minSphereDistance;
	meanDistances.at(iVert) = meanSphereDistance;

#ifdef WRITE_SPHERE_STATS
	auto sphereStatsFilename = outputDirectory + "/sphereStats" + std::to_string(iVert) + ".csv";
	std::ofstream sphereStats(sphereStatsFilename.c_str());
	sphereStats.imbue(std::locale("de-DE"));
#endif


#ifdef WRITE_SPHERE_STATS
	sphereStats << vert.position.x() << ";" << vert.position.y() << ";" << vert.position.z() << std::endl;
#endif		

#ifdef WRITE_HEIGHTFIELD
	auto heightFieldName = outputDirectoryW + L"/heightField" + std::to_wstring(iVert) + L".obj";
	std::ofstream heightField(heightFieldName.c_str());
	heightField << "mtllib ./heightfield.mtl" << std::endl;
	heightField << "usemtl Default_Smoothing" << std::endl;

	auto heightFieldSphereName = outputDirectoryW + L"/heightFieldSphere" + std::to_wstring(iVert) + L".obj";
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

	TSphereVisualizer sphereVisualizer(outputDirectoryW);

	//calculate gradient
	CalculateGradient(sphereSampling, sphereDistances, distanceGradient);

	std::vector<PositionValue> sphereDistanceMaxima, sphereDistanceMinima;
	FindStrongLocalExtrema(sphereSampling, sphereDistances, MAXIMUM_SEARCH_RADIUS, sphereDistanceMaxima, sphereDistanceMinima, sphereVisualizer);

	sphereVisualizer.Save(L"distanceField" + std::to_wstring(iVert) + L".png");

	//sphereVisualizer.DrawGradientField(sphereSampling, distanceGradient);		

	int threadId = omp_in_parallel() ? omp_get_thread_num() : 0;

	caveSizeUnsmoothed.at(iVert) = pow(CaveSizeCalculator::CalculateDistance(sphereSampling, sphereDistances, distanceGradient, sphereDistanceMaxima, sphereDistanceMinima, sphereVisualizer, iVert, caveSizeCalculatorCustomData.at(threadId)), 1.0 / exponent);

	sphereVisualizer.Save(L"sphereVis" + std::to_wstring(iVert) + L".png");

#ifdef WRITE_SPHERE_STATS
	sphereStats.close();
#endif

#ifdef WRITE_SPHERE_VIS
	sphereVis.close();
#endif

	return true;
}

template bool CaveData::CalculateDistancesSingleVertex<SphereVisualizer>(int iVert, float exponent, std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient);
template bool CaveData::CalculateDistancesSingleVertex<VoidSphereVisualizer>(int iVert, float exponent, std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient);

bool CaveData::CalculateDistances(float exponent)
{
	/*std::cout.precision(5);
	char data[] = { 0x17, 0, 0, 0, 0, 0, 0, 0, 0x90, 0, 0, 0, 0, 0, 0, 0, 0x1d, 0xf8, 0x34, 0x58, 0x0d, 0x99, 0xbf, 0xbf };	
	Vector p = *reinterpret_cast<Vector*>(data);
	std::cout << "Vector: " << p << std::endl;
	std::cout << "Vector.squared_length(): " << p.squared_length() << std::endl;
	std::cout << "squared length: " << (p.x() * p.x() + p.y() * p.y() + p.z() * p.z()) << std::endl;
	system("pause");*/

	std::vector<std::vector<std::vector<double>>> sphereDistances(omp_get_num_procs());
	std::vector<std::vector<std::vector<Vector>>> distanceGradient(omp_get_num_procs());

	for (int i = 0; i < omp_get_num_procs(); ++i)
	{
		sphereSampling.PrepareDataContainer(sphereDistances.at(i));
		sphereSampling.PrepareDataContainer(distanceGradient.at(i));
	}

#ifndef NON_VERBOSE
	std::cout << "Calculating distances..." << std::endl;
#endif

	invalidVertices.clear();

	caveSizeCalculatorCustomData.resize(omp_get_num_procs());
#pragma omp parallel for
	for (int iVert = 0; iVert < skeleton->vertices.size(); ++iVert)
	{		
		bool vertexValid = CalculateDistancesSingleVertex(iVert, exponent, sphereDistances.at(omp_get_thread_num()), distanceGradient.at(omp_get_thread_num()));
		if(!vertexValid)
#pragma omp critical
		{
			invalidVertices.push_back(iVert);
		}
	}

#ifndef NON_VERBOSE
	std::cout << "Finished." << std::endl;
#endif

	std::deque<int> invalidWork(invalidVertices.begin(), invalidVertices.end());
	//TODO: instead of reconstruction, move skeleton vertices inside shape
	//try to reconstruct invalid skeleton vertices
	while (!invalidWork.empty())
	{
		auto it = invalidWork.begin();
		while (it != invalidWork.end())
		{
			int validNeighborCount = 0;
			double validSum = 0;
			for (int n : adjacency.at(*it))
			{
				double neighborSize = caveSizeUnsmoothed.at(n);
				if (!std::isnan(neighborSize))
				{
					++validNeighborCount;
					validSum += neighborSize;
				}
			}
			if (validNeighborCount == 0)
				++it;
			else
			{
				caveSizeUnsmoothed.at(*it) = validSum / validNeighborCount;
				it = invalidWork.erase(it);
			}
		}
	}

	return invalidVertices.size() == 0;
}

void CaveData::LoadDistances(const std::string & file)
{
	std::cout << "Loading distances from file..." << std::endl;

	std::ifstream distanceFile(file.c_str(), std::ios::binary);
	distanceFile.read(reinterpret_cast<char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&caveSizeUnsmoothed[0]), sizeof(double) * caveSizeUnsmoothed.size());
	distanceFile.close();
}

void CaveData::SaveDistances(const std::string & file) const
{
	std::ofstream distanceFile(file.c_str(), std::ios::binary);
	distanceFile.write(reinterpret_cast<const char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&caveSizeUnsmoothed[0]), sizeof(double) * caveSizeUnsmoothed.size());
	distanceFile.close();
}

void CaveData::SmoothAndDeriveDistances()
{
	if (skeleton == nullptr)
		return;

#ifndef NON_VERBOSE
	std::cout << "Smoothing distances..." << std::endl;
#endif

	std::vector<double> smoothWorkDouble(std::max(skeleton->vertices.size(), skeleton->edges.size()));

	//Calculate cave scale by smoothing with a very large window
	switch (CAVE_SCALE_ALGORITHM)
	{
	case Max:
		findMax(skeleton->vertices, adjacency, [this](int iVert) { return CAVE_SCALE_KERNEL_FACTOR * caveSizeUnsmoothed.at(iVert); }, caveSizeUnsmoothed, caveScale);
		break;
	case Smooth:
		smooth(skeleton->vertices, adjacency, [this](int iVert) { return CAVE_SCALE_KERNEL_FACTOR * caveSizeUnsmoothed.at(iVert); }, caveSizeUnsmoothed, caveScale);
		break;
	case Advect:
		maxAdvect(skeleton->vertices, adjacency, [this](int iVert) { return CAVE_SCALE_KERNEL_FACTOR * caveSizeUnsmoothed.at(iVert); }, caveSizeUnsmoothed, caveScale);
		break;
	}	

	//Derive per vertex
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return CAVE_SIZE_KERNEL_FACTOR * caveScale.at(iVert); }, caveSizeUnsmoothed, caveSizes);
	//memcpy(&caveSizes[0], &smoothWorkDouble[0], caveSizes.size() * sizeof(double));

	/*derive(rootVertex, skeleton->vertices, children, caveSizes, smoothWorkDouble);
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return 0.2 * caveScale.at(iVert); }, smoothWorkDouble, caveSizeDerivatives);

	derive(rootVertex, skeleton->vertices, children, caveSizeDerivatives, caveSizeCurvatures);*/

	//Derive per edge
	derivePerEdgeFromVertices(skeleton, caveSizes, smoothWorkDouble);
	smoothPerEdge<double, true>(skeleton, adjacency, vertexPairToEdge, [this](int iEdge)
	{
		auto edge = skeleton->edges.at(iEdge);
		return CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR * 0.5 * (caveScale.at(edge.first) + caveScale.at(edge.second));
	}, smoothWorkDouble, caveSizeDerivativesPerEdge);

	derivePerEdge<double, true>(skeleton, adjacency, vertexPairToEdge, caveSizeDerivativesPerEdge, caveSizeCurvaturesPerEdge);

#ifndef NON_VERBOSE
	std::cout << "Finished smoothing." << std::endl;
#endif
}

void CaveData::SetOutputDirectory(const std::wstring & outputDirectory)
{
	outputDirectoryW = outputDirectory;
}

void CaveData::WriteBranchStatistics(const std::string & directory) const
{
	std::cout << "Writing branch statistics..." << std::endl;

	std::stack<int> branchStarts;
	branchStarts.push(rootVertex);

	std::ofstream f;
	f.open(directory + "/Branches.csv");
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

			f << currentV << ";" 
				<< dist << "; " 
				<< meanDistances[currentV] << ";" 
				<< minDistances[currentV] << ";" 
				<< maxDistances[currentV] << ";" 
				<< caveSizes[currentV] << "; " 
				<< caveScale[currentV] << "; " 
				<< caveSizeDerivatives[currentV] << "; " 
				<< caveSizeCurvatures[currentV];

			if (lastV != -1)
			{
				int edge = vertexPairToEdge.at(std::pair<int, int>(lastV, currentV));
				double inv = 1.0;
				//for direction dependent measures...
				if (lastV != skeleton->edges.at(edge).first)
					inv = -1.0;
				f << ";" << (dist - (skeleton->vertices.at(lastV).position - skeleton->vertices.at(currentV).position).norm() / 2)
					<< "; " << caveSizeDerivativesPerEdge.at(edge) * inv
					<< "; " << caveSizeCurvaturesPerEdge.at(edge);
			}

			f << std::endl;

			for (auto& adj : children.at(currentV))
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
}

void CaveData::ResizeMeshAttributes(size_t vertexCount)
{
	meshVertexCorrespondsTo.resize(vertexCount);
}

void CaveData::ResizeSkeletonAttributes(size_t vertexCount, size_t edgeCount)
{
	meanDistances.resize(vertexCount);
	maxDistances.resize(vertexCount);
	minDistances.resize(vertexCount);
	caveSizes.resize(vertexCount);
	nodeRadii.resize(vertexCount);
	parents.resize(vertexCount);
	children.resize(vertexCount);
	caveSizeDerivatives.resize(vertexCount);
	caveSizeCurvatures.resize(vertexCount);
	caveScale.resize(vertexCount);	
	adjacency.clear(); adjacency.resize(vertexCount);

	caveSizeUnsmoothed.resize(caveSizes.size());	

	caveSizeDerivativesPerEdge.resize(edgeCount);
	caveSizeCurvaturesPerEdge.resize(edgeCount);
}

int noSegmentColor[3] = { 128, 128, 128 };
int segmentColors[10][3] =
{
	{ 166, 206, 227 },
	{ 31,120,180 },
	{ 251,154,153 },
	{ 227,26,28 },
	{ 253,191,111 },
	{ 255,127,0 },
	{ 202,178,214 },
	{ 106,61,154 },
	{ 255,255,153 },
	{ 177,89,40 }
};

const int* CaveData::GetSegmentColor(int segmentIndex)
{
	if (segmentIndex < 0)
		return noSegmentColor;
	else
		return segmentColors[segmentIndex % 10];
}

void CaveData::WriteSegmentationColoredOff(const std::string & path, const std::vector<int32_t>& segmentation)
{
	auto colorFunc = [&](int i, int& r, int& g, int& b)
	{
		const int* color = GetSegmentColor(segmentation[i]);
		r = color[0];
		g = color[1];
		b = color[2];
	};

	WriteOff(path.c_str(), meshVertices(), meshTriIndices(), [&](int i, int& r, int& g, int& b) {colorFunc(meshVertexCorrespondsTo[i], r, g, b); });
}

void CaveData::CalculateBasicSkeletonData()
{
	std::cout << "Calculating adjacency list..." << std::endl;

	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		auto& edge = skeleton->edges.at(iEdge);
		adjacency[edge.first].push_back(edge.second);
		adjacency[edge.second].push_back(edge.first);
		vertexPairToEdge[edge] = iEdge;
		vertexPairToEdge[std::pair<int, int>(edge.second, edge.first)] = iEdge;
	}

	//find root
	/*rootVertex = -1;
	for (int v = 0; v < skeleton->vertices.size(); ++v)
		if (adjacency[v].size() == 1)
		{
			rootVertex = v;
			break;
		}

	buildTree(rootVertex, skeleton->vertices.size(), adjacency, parents, children);*/

	//Clean correspondences (use the closer skeleton vertex of local neighbors)
	std::vector<std::vector<int>> cleanedCorrespondences(skeleton->vertices.size());

	int iVert = -1;
	for (auto& vert : skeleton->vertices)
	{
		++iVert;

		for (int c : vert.correspondingOriginalVertices)
		{
			auto meshVertex = _meshVertices.at(c);
			int closestSkeletonVertex = iVert;
			double closestDistance = (meshVertex - vert.position).norm();

			bool changed = false;
			do
			{
				changed = false;
				auto& adj = adjacency.at(closestSkeletonVertex);
				for (auto n : adj)
				{
					double currentDistance = (meshVertex - skeleton->vertices.at(n).position).norm();
					if (currentDistance < closestDistance)
					{
						closestDistance = currentDistance;
						closestSkeletonVertex = n;
						changed = true;
					}
				}
			} while (changed);
			cleanedCorrespondences.at(closestSkeletonVertex).push_back(c);
		}
	}
	for (int iVert = 0; iVert < skeleton->vertices.size(); ++iVert)
	{
		auto& v = skeleton->vertices.at(iVert);
		v.correspondingOriginalVertices = std::move(cleanedCorrespondences.at(iVert));
	}

	//calculate correspondences and node radii
	iVert = -1;
	for (auto& vert : skeleton->vertices)
	{
		++iVert;

		auto& adj = adjacency.at(iVert);
		for (auto c : vert.correspondingOriginalVertices)
		{
			meshVertexCorrespondsTo[c] = iVert;
		}

		double nodeRadius = 0.0;		
		for (auto adjV : adj)
		{
			auto& v = skeleton->vertices.at(adjV);
			nodeRadius += (vert.position - v.position).norm() / 2.0;
		}
		nodeRadii.at(iVert) = nodeRadius / adj.size();
	}

	std::cout << "Finished correspondences." << std::endl;
}

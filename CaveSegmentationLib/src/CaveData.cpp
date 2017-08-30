#include "Options.h"

#include "CaveData.h"
#include "FileInputOutput.h"
#include "GraphProc.h"
#include "ImageProc.h"

#include <stack>
#include <deque>

#include <boost/filesystem/operations.hpp>

void ReadOff(std::string filename, std::vector<Eigen::Vector3f>& vertices, std::vector<Triangle>& triangles, std::vector<IndexedTriangle>& triIndices)
{
	std::cout << "Reading file..." << std::endl;

	std::ifstream f;
	f.open(filename, std::ios::in);
	if (!f.good())
		throw;
	std::string line;
	int nVertices = -1, nFaces = -1, nEdges = -1;

	std::vector<Eigen::Vector3f>::iterator nextVertex;
	auto nextTriangle = triangles.begin();
	auto nextTriIndex = triIndices.begin();

	while (std::getline(f, line))
	{
		if (line.size() == 0)
			continue;
		if (line[0] == '#')
			continue;
		if (line == "OFF")
			continue;

		std::stringstream str(line);
		if (nVertices < 0)
		{
			str >> nVertices >> nFaces >> nEdges;
			vertices.resize(nVertices);
			triangles.resize(nFaces);
			triIndices.resize(nFaces);
			nextVertex = vertices.begin();
			nextTriangle = triangles.begin();
			nextTriIndex = triIndices.begin();
		}
		else
		{
			if (nVertices > 0)
			{
				str >> nextVertex->x() >> nextVertex->y() >> nextVertex->z();
				++nextVertex;
				--nVertices;
			}
			else if (nFaces > 0)
			{
				int n, a, b, c;
				str >> n >> a >> b >> c;
				if (n != 3)
					throw;
				(*nextTriangle) = Triangle(
					Point(vertices[a].x(), vertices[a].y(), vertices[a].z()),
					Point(vertices[b].x(), vertices[b].y(), vertices[b].z()),
					Point(vertices[c].x(), vertices[c].y(), vertices[c].z()));
				(*nextTriIndex).i[0] = a;
				(*nextTriIndex).i[1] = b;
				(*nextTriIndex).i[2] = c;

				++nextTriangle;
				++nextTriIndex;
				--nFaces;
			}
		}
	}
	f.close();
}

//Calculates the gradient field of sphereDistances over the sphere.
void CalculateGradient(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<double>>& sphereDistances, std::vector<std::vector<Vector>>& distanceGradient)
{
	for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
	{
		double currentDistance = sphereSampling.AccessContainerData(sphereDistances, it);
		Vector currentPos = *it;

		//Calculate a 3D gradient from directional derivatives.
		//Solve
		//    arg min  Σ ( dot(g, direction_i) - derivative_i )^2
		//       g

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

		//Solve the linear system
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

//Finds strong local extrema of sphereDistances, i.e. points that are extreme in a neighborhood of extremumSearchRadius
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
	: skeleton(nullptr), verbose(true),
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

			int32_t n_vertices = (int32_t)_meshVertices.size();
			fwrite(&n_vertices, sizeof(int32_t), 1, cache);
			if(n_vertices > 0)
				fwrite(&_meshVertices[0], sizeof(Eigen::Vector3f), n_vertices, cache);

			int32_t n_triangles = (int32_t)_meshTriIndices.size();
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

	ResetBoundingBox();
	for (auto& v : _meshVertices)
		AddPoint(v);	
}

void CaveData::WriteMesh(const std::string& offFile, std::function<void(int i, int& r, int& g, int& b)> colorFunc) const
{
	WriteOff(offFile, MeshVertices(), MeshTriIndices(), colorFunc);
}

void CaveData::WriteSegmentationColoredOff(const std::string & path, const std::vector<int32_t>& segmentation) const
{
	auto colorFunc = [&](int i, int& r, int& g, int& b)
	{
		const int* color = ICaveData::GetSegmentColor(segmentation.at(i));
		r = color[0];
		g = color[1];
		b = color[2];
	};

	WriteMesh(path.c_str(), [&](int i, int& r, int& g, int& b) {colorFunc(meshVertexCorrespondsTo[i], r, g, b); });
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

bool CaveData::CalculateDistancesSingleVertexWithDebugOutput(int iVert, float exponent)
{
	return CalculateDistancesSingleVertex<SphereVisualizer>(iVert, exponent);
}

//Calculates the cave size at a single skeleton vertex and stores it in caveSizeUnsmoothed.
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

//Calculates the cave size at a single skeleton vertex and stores it in caveSizeUnsmoothed.
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
			if(verbose)
#pragma omp critical
			{
				std::cout << "Skeleton vertex " << iVert << " lies outside of mesh!" << std::endl;
			}

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
	sphereStats.imbue(std::locale(""));
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

//Calculates the cave sizes for the entire skeleton and stores them in caveSizeUnsmoothed.
bool CaveData::CalculateDistances(float exponent)
{	
	if(verbose)
		std::cout << "Calculating distances..." << std::endl;

	invalidVertices.clear();

	caveSizeCalculatorCustomData.resize(omp_get_num_procs());
#pragma omp parallel
	{
		std::vector<std::vector<double>> sphereDistances;
		std::vector<std::vector<Vector>> distanceGradient;
		sphereSampling.PrepareDataContainer(sphereDistances);
		sphereSampling.PrepareDataContainer(distanceGradient);
#pragma omp for
		for (int iVert = 0; iVert < skeleton->vertices.size(); ++iVert)
		{
			bool vertexValid = CalculateDistancesSingleVertex(iVert, exponent, sphereDistances, distanceGradient);
			if (!vertexValid)
#pragma omp critical
			{
				invalidVertices.push_back(iVert);
			}
		}
	}

	if(verbose)
		std::cout << "Finished." << std::endl;	

	std::deque<int> invalidWork(invalidVertices.begin(), invalidVertices.end());
	//TODO: instead of reconstruction, move skeleton vertices inside shape
	//try to reconstruct invalid skeleton vertices by interpolating from valid neighbors
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
	if (!distanceFile.good())
		throw std::exception("Cannot open file");
	distanceFile.read(reinterpret_cast<char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&caveSizeUnsmoothed[0]), sizeof(double) * caveSizeUnsmoothed.size());
	distanceFile.close();
}

void CaveData::SaveDistances(const std::string & file) const
{
	std::ofstream distanceFile(file.c_str(), std::ios::binary);
	if (!distanceFile.good())
		throw std::exception("Cannot open file");
	distanceFile.write(reinterpret_cast<const char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&caveSizeUnsmoothed[0]), sizeof(double) * caveSizeUnsmoothed.size());
	distanceFile.close();
}

//Calculates additional measures from the unsmoothed cave sizes.
void CaveData::SmoothAndDeriveDistances()
{
	if (skeleton == nullptr)
		return;

	if(verbose)
		std::cout << "Smoothing distances..." << std::endl;

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

	//Smooth cave sizes: caveSizes <- smooth(caveSizeUnsmoothed)
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return CAVE_SIZE_KERNEL_FACTOR * caveScale.at(iVert); }, caveSizeUnsmoothed, caveSizes);

	//Derive cave sizes: smoothWorkDouble <- derive(caveSizes)
	derivePerEdgeFromVertices(skeleton, caveSizes, smoothWorkDouble);
	//Smooth derivatives: caveSizeDerivativesPerEdge <- smooth(smoothWorkDouble) = smooth(derive(caveSizes))
	smoothPerEdge<double, true>(*this, [this](int iEdge)
	{
		auto edge = skeleton->edges.at(iEdge);
		return CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR * 0.5 * (caveScale.at(edge.first) + caveScale.at(edge.second));
	}, smoothWorkDouble, caveSizeDerivativesPerEdge);

	//Derive second derivatives: caveSizeCurvaturesPerEdge <- derive(caveSizeDerivativesPerEdge)
	derivePerEdge<double, true>(*this, caveSizeDerivativesPerEdge, caveSizeCurvaturesPerEdge);

	if(verbose)
		std::cout << "Finished smoothing." << std::endl;
}

void CaveData::SetOutputDirectory(const std::wstring & outputDirectory)
{
	outputDirectoryW = outputDirectory;
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
	caveScale.resize(vertexCount);	
	adjacency.clear(); adjacency.resize(vertexCount);

	caveSizeUnsmoothed.resize(caveSizes.size());	

	caveSizeDerivativesPerEdge.resize(edgeCount);
	caveSizeCurvaturesPerEdge.resize(edgeCount);
}

void CaveData::CalculateBasicSkeletonData()
{
	if(verbose)
		std::cout << "Calculating adjacency list..." << std::endl;

	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		auto& edge = skeleton->edges.at(iEdge);
		adjacency[edge.first].push_back(edge.second);
		adjacency[edge.second].push_back(edge.first);
		vertexPairToEdge[edge] = iEdge;
		vertexPairToEdge[std::pair<int, int>(edge.second, edge.first)] = iEdge;
	}	

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

	if(verbose)
		std::cout << "Finished correspondences." << std::endl;
}

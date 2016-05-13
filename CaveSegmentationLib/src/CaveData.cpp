#include "Options.h"

#include "CaveData.h"
#include "FileInputOutput.h"
#include "GraphProc.h"
#include "ImageProc.h"

#include <stack>

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

CaveData::CaveData()
	: skeleton(nullptr),
	  sphereSampling(SPHERE_SAMPLING_RESOLUTION),
	  caveSizeCalculator(std::unique_ptr<CaveSizeCalculator>(new CaveSizeCalculatorLineFlow()))
{
}

void CaveData::LoadMesh(const std::string & offFile)
{
	ReadOff(offFile, _meshVertices, _meshTriangles, _meshTriIndices);
	ResizeMeshAttributes(_meshVertices.size());

	_meshAABBTree.insert(_meshTriangles.begin(), _meshTriangles.end());
	_meshAABBTree.build();
}

void CaveData::SetSkeleton(CurveSkeleton * skeleton)
{
	this->skeleton = skeleton;
	ResizeSkeletonAttributes(skeleton->vertices.size(), skeleton->edges.size());
	CalculateBasicSkeletonData();
}

void CaveData::CalculateDistances()
{
	std::vector<std::vector<double>> sphereDistances;
	std::vector<std::vector<Vector>> distanceGradient;
	sphereSampling.PrepareDataContainer(sphereDistances);
	sphereSampling.PrepareDataContainer(distanceGradient);

	std::cout << "Calculating distances..." << std::endl;

	int iVert = -1;
	for (auto& vert : skeleton->vertices)
	{
		++iVert;
		//if (iVert != 69) continue;
		std::cout << "\rProcessing skeleton vertex " << iVert;

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
			double visDist = sqrt(GetSqrDistanceToMesh(Point(vert.position.x(), vert.position.y(), vert.position.z()), *it, _meshAABBTree));
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

		sphereVisualizer.Save(L"distanceField" + std::to_wstring(iVert) + L".png");

		//sphereVisualizer.DrawGradientField(sphereSampling, distanceGradient);		

		caveSizes.at(iVert) = caveSizeCalculator->CalculateDistance(sphereSampling, sphereDistances, distanceGradient, sphereDistanceMaxima, sphereDistanceMinima, sphereVisualizer, iVert);

		sphereVisualizer.Save(L"sphereVis" + std::to_wstring(iVert) + L".png");

#ifdef WRITE_SPHERE_STATS
		sphereStats.close();
#endif

#ifdef WRITE_SPHERE_VIS
		sphereVis.close();
#endif									
	}

	std::cout << std::endl;
}

void CaveData::LoadDistances(const std::string & file)
{
	std::cout << "Loading distances from file..." << std::endl;

	std::ifstream distanceFile(file.c_str(), std::ios::binary);
	distanceFile.read(reinterpret_cast<char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.read(reinterpret_cast<char*>(&caveSizes[0]), sizeof(double) * caveSizes.size());
	distanceFile.close();
}

void CaveData::SaveDistances(const std::string & file) const
{
	std::ofstream distanceFile(file.c_str(), std::ios::binary);
	distanceFile.write(reinterpret_cast<const char*>(&maxDistances[0]), sizeof(double) * maxDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&minDistances[0]), sizeof(double) * minDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&meanDistances[0]), sizeof(double) * meanDistances.size());
	distanceFile.write(reinterpret_cast<const char*>(&caveSizes[0]), sizeof(double) * caveSizes.size());
	distanceFile.close();
}

void CaveData::SmoothAndDeriveDistances()
{
	std::cout << "Smoothing distances..." << std::endl;

	std::vector<double> smoothWorkDouble(std::max(skeleton->vertices.size(), skeleton->edges.size()));

	//Calculate cave scale by smoothing with a very large window
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return 10.0 * caveSizes.at(iVert); }, caveSizes, caveScale);

	//Derive per vertex
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return 0.2 * caveScale.at(iVert); }, caveSizes, smoothWorkDouble);
	memcpy(&caveSizes[0], &smoothWorkDouble[0], caveSizes.size() * sizeof(double));

	derive(rootVertex, skeleton->vertices, children, caveSizes, smoothWorkDouble);
	smooth(skeleton->vertices, adjacency, [this](int iVert) { return 0.2 * caveScale.at(iVert); }, smoothWorkDouble, caveSizeDerivatives);

	derive(rootVertex, skeleton->vertices, children, caveSizeDerivatives, caveSizeCurvatures);

	//Derive per edge
	derivePerEdgeFromVertices(skeleton, caveSizes, smoothWorkDouble);
	smoothPerEdge<double, true>(skeleton, adjacency, vertexPairToEdge, [this](int iEdge)
	{
		auto edge = skeleton->edges.at(iEdge);
		return 0.2 * 0.5 * (caveScale.at(edge.first) + caveScale.at(edge.second));
	}, smoothWorkDouble, caveSizeDerivativesPerEdge);

	derivePerEdge<double, true>(skeleton, adjacency, vertexPairToEdge, caveSizeDerivativesPerEdge, caveSizeCurvaturesPerEdge);
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
	adjacency.resize(vertexCount);

	caveSizeDerivativesPerEdge.resize(edgeCount);
	caveSizeCurvaturesPerEdge.resize(edgeCount);
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
	rootVertex = -1;
	for (int v = 0; v < skeleton->vertices.size(); ++v)
		if (adjacency[v].size() == 1)
		{
			rootVertex = v;
			break;
		}

	buildTree(rootVertex, skeleton->vertices.size(), adjacency, parents, children);

	//calculate correspondences and node radii
	int iVert = -1;
	for (auto& vert : skeleton->vertices)
	{
		++iVert;

		auto& adj = adjacency.at(iVert);
		for (auto c : vert.correspondingOriginalVertices)
		{
			meshVertexCorrespondsTo[c] = iVert;
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
		nodeRadii.at(iVert) = nodeRadius / adj.size();
	}
}

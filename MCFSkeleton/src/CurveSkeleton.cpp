#include "CurveSkeleton.h"
#include "SurfaceMeshModel.h"
#include "Skelcollapse.h"

#include <fstream>
#include <memory>

using namespace SurfaceMesh;

CurveSkeleton* ComputeCurveSkeleton(const std::string filename, AbortHandle* abort, float edgeCollapseThreshold, float w_smooth, float w_velocity, float w_medial)
{
	auto mesh = std::make_unique<SurfaceMeshModel>();
	abort->abort = false;

	if (!mesh->read(filename))
	{
		return nullptr;
	}
	mesh->updateBoundingBox();

	Skelcollapse collapse(mesh.get(), abort, edgeCollapseThreshold, w_smooth, w_velocity, w_medial);

	if (abort->abort)
		return nullptr;

	collapse.calculateVoronoi();

	if (abort->abort)
		return nullptr;

	collapse.contract();

	if (abort->abort)
		return nullptr;

	CurveSkeleton* skeleton = collapse.collapseToSkeleton();

	if (abort->abort)
		return nullptr;

	return skeleton;
}

void DestroySkeleton(CurveSkeleton* s)
{
	delete s;
}

void CurveSkeleton::Save(const char* filename)
{
	FILE* f = fopen(filename, "wb");
	if (f == nullptr)
		throw;

	//Write vertex count
	size_t nVertices = vertices.size();
	fwrite(&nVertices, sizeof(size_t), 1, f);

	//Write vertices
	for (const Vertex& v : vertices)
	{
		float xyz[3] = { v.position.x(), v.position.y(), v.position.z() };
		fwrite(xyz, sizeof(float), 3, f);

		size_t nCorrs = v.correspondingOriginalVertices.size();
		fwrite(&nCorrs, sizeof(size_t), 1, f);
		fwrite(&v.correspondingOriginalVertices[0], sizeof(int), nCorrs, f);
	}

	//Write edge count
	size_t nEdges = edges.size();
	fwrite(&nEdges, sizeof(size_t), 1, f);
	fwrite(&edges[0], sizeof(TEdge), nEdges, f);

	fclose(f);
}

CurveSkeleton* LoadCurveSkeleton(const char* filename)
{
	FILE* f = fopen(filename, "rb");
	if (f == nullptr)
		throw;

	auto skel = new CurveSkeleton();

	//Read vertex count
	size_t nVertices;
	fread(&nVertices, sizeof(size_t), 1, f);
	skel->vertices.resize(nVertices);

	//Read vertices
	for (int i = 0; i < nVertices; ++i)
	{
		auto& v = skel->vertices.at(i);
		float xyz[3];
		fread(xyz, sizeof(float), 3, f);
		v.position.x() = xyz[0];
		v.position.y() = xyz[1];
		v.position.z() = xyz[2];

		size_t nCorrs = v.correspondingOriginalVertices.size();
		fread(&nCorrs, sizeof(size_t), 1, f);
		v.correspondingOriginalVertices.resize(nCorrs);
		if(nCorrs > 0)
			fread(&v.correspondingOriginalVertices[0], sizeof(int), nCorrs, f);
	}

	//Read edges
	size_t nEdges;	
	fread(&nEdges, sizeof(size_t), 1, f);
	skel->edges.resize(nEdges);
	fread(&skel->edges[0], sizeof(CurveSkeleton::TEdge), nEdges, f);
	
	fclose(f);

	return skel;
}

void ToObj(CurveSkeleton* skel, const char* filename, SurfaceMeshModel* originalMesh, std::function<void(const CurveSkeleton::Vertex&, int i, int& r, int& g, int& b)> colorFunc, std::function<void(const CurveSkeleton::Vertex& v, int i, float& newX, float& newY, float& newZ)> positionFunc)
{
	std::ofstream f;
	f.open(filename, std::ios::out);

	int originalTotalVertices = 0;
	if (originalMesh != nullptr)
	{
		originalTotalVertices = originalMesh->n_vertices();
		auto pos = originalMesh->vertex_property<Eigen::Vector3d>("v:point");
		for (auto it = originalMesh->vertices_begin(); it != originalMesh->vertices_end(); ++it)
		{
			f << "v " << pos[it][0] << " " << pos[it][1] << " " << pos[it][2] << std::endl;
		}
	}

	int i = 0;
	for (auto it = skel->vertices.begin(); it != skel->vertices.end(); ++it)
	{
		int r, g, b;
		colorFunc(*it, i, r, g, b);

		float x, y, z;
		positionFunc(*it, i, x, y, z);

		//write the same vertex twice, so we can build a degenerate triangle
		f << "v " << x << " " << y << " " << z << " " << r << " " << g << " " << b << std::endl;
		f << "v " << x << " " << y << " " << z << " " << r << " " << g << " " << b << std::endl;

		if (originalMesh != nullptr)
		{
			for (auto cor = it->correspondingOriginalVertices.begin(); cor != it->correspondingOriginalVertices.end(); ++cor)
			{
				f << "f " << originalTotalVertices + 2 * i + 1 << " " << originalTotalVertices + 2 * i + 2 << " " << *cor + 1 << std::endl;
			}
		}
		i++;
	}
	for (auto it = skel->edges.begin(); it != skel->edges.end(); ++it)
	{
		f << "f " << originalTotalVertices + 2 * it->first + 1 << " " << originalTotalVertices + 2 * it->second + 1 << " " << originalTotalVertices + 2 * it->second + 2 << std::endl;
	}
	f.close();
}

void defaultPosFunc(const CurveSkeleton::Vertex& v, int, float& x, float& y, float& z){ x = v.position.x(); y = v.position.y(); z = v.position.z(); }
void defaultColorFunc(const CurveSkeleton::Vertex& v, int i, int& r, int& g, int& b) { r = g = b = 255; }

void CurveSkeleton::SaveToObj(const char* filename, std::function<void(const CurveSkeleton::Vertex&, int i, int& r, int& g, int& b)> colorFunc)
{
	ToObj(this, filename, nullptr, colorFunc, defaultPosFunc);
}

void CurveSkeleton::SaveToObj(const char* filename)
{
	ToObj(this, filename, nullptr, defaultColorFunc, defaultPosFunc);
}

void CurveSkeleton::SaveToObj(const char* filename, std::function<void(const CurveSkeleton::Vertex&, int i, int& r, int& g, int& b)> colorFunc, std::function<void(const CurveSkeleton::Vertex& v, int i, float& newX, float& newY, float& newZ)> positionFunc)
{
	ToObj(this, filename, nullptr, colorFunc, positionFunc);
}

void CurveSkeleton::SaveToObjWithCorrespondences(const char* filename, const std::string originalMesh, std::function<void(const CurveSkeleton::Vertex&, int i, int& r, int& g, int& b)> colorFunc)
{
	auto mesh = new SurfaceMeshModel;

	if (!mesh->read(originalMesh))
		throw;

	ToObj(this, filename, mesh, colorFunc, defaultPosFunc);
}

void CurveSkeleton::SaveToObjWithCorrespondences(const char* filename, const std::string originalMesh)
{
	SaveToObjWithCorrespondences(filename, originalMesh, defaultColorFunc);
}
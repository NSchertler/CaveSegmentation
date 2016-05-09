#pragma once

#include <vector>
#include "CGALCommon.h"

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

double GetSqrDistanceToMesh(const Point& p, const Vector& dir, const Tree& tree);

/*template <class HDS>
class BuildCavePolyhedron : public CGAL::Modifier_base < HDS >
{
public:
	BuildCavePolyhedron(std::vector<size_t>& skeletonClassification,
		std::vector<unsigned int>& vertexCorrespondence,
		std::vector<Eigen::Vector3f>& vertices,
		std::vector<IndexedTriangle>& triIndices)
		: skeletonClassification(skeletonClassification),
		vertexCorrespondence(vertexCorrespondence),
		vertices(vertices),
		triIndices(triIndices)
	{}

	void operator()(HDS& hds)
	{
		CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds);

		std::map<unsigned int, unsigned int> globalToLocal;

		builder.begin_surface(100, 100);

		int iLocal = 0;
		for (int iVert = 0; iVert < vertices.size(); ++iVert)
		{
			if (skeletonClassification[vertexCorrespondence[iVert]] == 0)
			{
				builder.add_vertex(HDS::Vertex::Point(vertices[iVert].x(), vertices[iVert].y(), vertices[iVert].z()));
				globalToLocal[iVert] = iLocal++;
			}
		}
		for (int iFace = 0; iFace < triIndices.size(); ++iFace)
		{
			if (skeletonClassification[vertexCorrespondence[triIndices[iFace].i[0]]] == 0
				|| skeletonClassification[vertexCorrespondence[triIndices[iFace].i[1]]] == 0
				|| skeletonClassification[vertexCorrespondence[triIndices[iFace].i[2]]] == 0)
			{
				for (int i = 0; i < 3; ++i)
				{
					int iVert = triIndices[iFace].i[i];
					if (globalToLocal.find(iVert) == globalToLocal.end())
					{
						builder.add_vertex(HDS::Vertex::Point(vertices[iVert].x(), vertices[iVert].y(), vertices[iVert].z()));
						globalToLocal[iVert] = iLocal++;
					}
				}

				builder.begin_facet();
				builder.add_vertex_to_facet(globalToLocal[triIndices[iFace].i[0]]);
				builder.add_vertex_to_facet(globalToLocal[triIndices[iFace].i[1]]);
				builder.add_vertex_to_facet(globalToLocal[triIndices[iFace].i[2]]);
				builder.end_facet();
			}
		}
		builder.end_surface();
	}

private:
	std::vector<size_t>& skeletonClassification;
	std::vector<unsigned int>& vertexCorrespondence;

	std::vector<Eigen::Vector3f>& vertices;
	std::vector<IndexedTriangle>& triIndices;

};*/
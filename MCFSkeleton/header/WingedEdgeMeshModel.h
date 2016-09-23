#pragma once

#include "WingedEdgeMesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "surface_mesh/Surface_mesh.h"

namespace WingedEdge
{
	class WingedEdgeMeshForEachVertexHelper;
	class WingedEdgeMeshForEachEdgeHelper;

	class WingedEdgeMeshModel : public WingedEdgeMesh < Surface_mesh::Scalar, Surface_mesh::Vector3 >
	{

	public:

		typedef Surface_mesh::Vector3 Vector;

		WingedEdgeMeshModel();
		void updateBoundingBox();

	public:
		WingedEdgeMeshForEachVertexHelper vertices();
		WingedEdgeMeshForEachEdgeHelper edges();
		/// @}

		std::set<Vertex> junctions();

		std::set<Vertex> adjacent_set(Vertex v);
		Vertex other_vertex(Edge e, Vertex v);

		Eigen::AlignedBox3d _bbox;
	};
}

#include "WingedEdgeMeshForEach.h"
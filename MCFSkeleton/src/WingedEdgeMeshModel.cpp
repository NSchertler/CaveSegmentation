#include "WingedEdgeMeshModel.h"
#include "WingedEdgeMeshForEach.h"

using namespace std;
using namespace WingedEdge;

WingedEdgeMeshModel::WingedEdgeMeshModel() : WingedEdgeMesh(){}

void WingedEdgeMeshModel::updateBoundingBox(){
	auto vcoord = get_vertex_property<Vector>("v:point");
	_bbox.setNull();
	for(Vertex v : this->vertices())
		_bbox = _bbox.merged(Eigen::AlignedBox3d(Vector(vcoord[v])));
}

WingedEdgeMeshForEachEdgeHelper WingedEdgeMeshModel::edges(){
	return WingedEdgeMeshForEachEdgeHelper(this);
}

std::set<WingedEdgeMeshModel::Vertex> WingedEdgeMeshModel::junctions()
{
	std::set<Vertex> j;

	for(Vertex v : this->vertices())
		if (this->valence(v) > 2)
			j.insert(v);

	return j;
}

WingedEdgeMeshForEachVertexHelper WingedEdgeMeshModel::vertices(){
	return WingedEdgeMeshForEachVertexHelper(this);
}

std::set<WingedEdgeMeshModel::Vertex> WingedEdgeMeshModel::adjacent_set(Vertex v)
{
	std::set<Vertex> adj;
	Edge_around_vertex e(this, v), eend = e;
	while (!e.end())
	{
		(v == this->vertex(e, 0)) ? adj.insert(this->vertex(e, 1)) : adj.insert(this->vertex(e, 0));
		++e;
	}
	return adj;
}

WingedEdgeMeshModel::Vertex WingedEdgeMeshModel::other_vertex(Edge e, Vertex v)
{
	return (v == this->vertex(e, 0)) ? this->vertex(e, 1) : this->vertex(e, 0);
}

// Explicit Template Instantiation
//template class WingedgeMesh <double, CurveskelTypes::SkelVector<double, 3> >;

#pragma once
#if !defined(_WIN32)
#pragma GCC diagnostic ignored "-Wwrite-strings"
#endif

#include <list>

#include "SurfaceMeshHelper.h"
#include "CurveSkeleton.h"
#include "AbortHandle.h"

typedef std::list<int> VertexList;
typedef ElementProperty<VertexList, Surface_mesh::Vertex> VertexListVertexProperty;

using namespace SurfaceMesh;

class Skelcollapse {

private:
	/// @{ algorithm parameters
	float omega_L_0;
	float omega_H_0;
	float omega_P_0;
	float edgelength_TH;
	float zero_TH;
	/// @} 

	/// @{ algorithm internal data
	VertexListVertexProperty corrs;
	Vector3VertexProperty points;
	Vector3VertexProperty poles;
	ScalarVertexProperty  omega_H;
	ScalarVertexProperty  omega_L;
	ScalarVertexProperty  omega_P;
	BoolVertexProperty    vissplit;
	BoolVertexProperty    visfixed;
	bool                  isInitialized;
	unsigned int		  fixedVertices;

	std::vector<Point> originalPoints;

	SurfaceMeshModel *mesh;
	SurfaceMeshHelper helper;
	/// @}

	AbortHandle* abort;

	void algorithm_iteration(const std::vector<Point>& originalPoints){
		contractGeometry();
		updateConstraints();
		updateTopology(originalPoints);
		detectDegeneracies();
	}

	void updateConstraints();
	void contractGeometry();
	void detectDegeneracies();
	void updateTopology(const std::vector<Point>& originalPoints);

public:
	Skelcollapse(SurfaceMeshModel* mesh, AbortHandle* abort, float edgeCollapseThreshold, float omega_L, float omega_H, float omega_P)
		: mesh(mesh), helper(SurfaceMeshHelper(mesh)), abort(abort)
	{
		//edgelength_TH = 0.002f*mesh->bbox().diagonal().norm();
		omega_L_0 = omega_L;
		omega_H_0 = omega_H;
		omega_P_0 = omega_P;
		edgelength_TH = edgeCollapseThreshold;
		zero_TH = 1e-7f;
		isInitialized = false;

		points = helper.getVector3VertexProperty(VPOINT);		
		this->omega_H = mesh->vertex_property<Scalar>("v:omega_H", omega_H_0);
		this->omega_L = mesh->vertex_property<Scalar>("v:omega_L", omega_L_0);
		this->omega_P = mesh->vertex_property<Scalar>("v:omega_P", omega_P_0);
		vissplit = mesh->vertex_property<bool>("v:issplit", false);
		visfixed = mesh->vertex_property<bool>("v:isfixed", false);
		corrs = mesh->vertex_property<VertexList>("v:corrs");		
	}

	void calculateVoronoi();

	void contract();

	CurveSkeleton* collapseToSkeleton();
};

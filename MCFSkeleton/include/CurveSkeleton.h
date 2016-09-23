/*
This is a stand-alone adaptation of https://code.google.com/p/starlab-mcfskel/

"Mean Curvature Skeletons" by Andrea Tagliasacchi and Ibraheem Alhashim and Matt Olson and Hao Zhang
Computer Graphics Forum (Proc. of the Symposium on Geometry Processing) Vol. 31, Is. 5, 2012
*/

#pragma once

#include <vector>
#include <string>
#include <functional>

#include <Eigen/Dense>

#include "AbortHandle.h"

#ifdef DLLEXPORT
#define MCF_SKELETON_API __declspec(dllexport)
#else
#define MCF_SKELETON_API __declspec(dllimport)
#endif

struct CurveSkeleton
{
	struct Vertex
	{
		Eigen::Vector3f position;
		std::vector<int> correspondingOriginalVertices;

		Vertex() { }
		Vertex(const Eigen::Vector3f& position) : position(position) { }
	};

	typedef std::pair<int, int> TEdge;

	std::vector<Vertex> vertices;
	std::vector<TEdge> edges;

	MCF_SKELETON_API void SaveToObj(const char* filename);
	MCF_SKELETON_API void SaveToObj(const char* filename, std::function<void(const Vertex&, int i, int& r, int& g, int& b)> colorFunc);
	MCF_SKELETON_API void SaveToObj(const char* filename, std::function<void(const Vertex&, int i, int& r, int& g, int& b)> colorFunc, std::function<void(const Vertex& v, int i, float& newX, float& newY, float& newZ)> positionFunc);

	MCF_SKELETON_API void SaveToObjWithCorrespondences(const char* filename, const std::string originalMesh);
	MCF_SKELETON_API void SaveToObjWithCorrespondences(const char* filename, const std::string originalMesh, std::function<void(const Vertex&, int i, int& r, int& g, int& b)> colorFunc);

	MCF_SKELETON_API void Save(const char* filename);
};

extern MCF_SKELETON_API CurveSkeleton* ComputeCurveSkeleton(const std::string filename, AbortHandle* abort, float edgeCollapseThreshold, float w_smooth = 1.0f, float w_velocity = 0.1f, float w_medial = 0.2f);
extern MCF_SKELETON_API CurveSkeleton* LoadCurveSkeleton(const char* filename);
extern MCF_SKELETON_API void DestroySkeleton(CurveSkeleton*);
#pragma once

#include "ICaveData.h"

class EdgeCurvatureAccessor
{
public:
	EdgeCurvatureAccessor(const ICaveData& data) : data(data) {}
	double operator()(size_t edge) const { return data.CaveSizeCurvature(edge); }

private:
	const ICaveData& data;
};

class VertexCaveSizeAccessor
{
public:
	VertexCaveSizeAccessor(const ICaveData& data) : data(data) {}
	double operator()(size_t vertex) const { return data.CaveSize(vertex); }

private:
	const ICaveData& data;
};
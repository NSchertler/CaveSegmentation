#pragma once

#include "Options.h"

#include "ImageProc.h"

#include "CGALCommon.h"
#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"
#include "LineProc.h"

#define DIFFERENT_SIGN(e1, e2) (((e1) > 0) != ((e2) > 0))

struct PositionValue
{
	Vector position;
	double value;
};

class CaveSizeCalculator
{
public:
	virtual double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		SphereVisualizer& visualizer,
		int iVert) = 0;
};

class CaveSizeCalculatorVoronoi : public CaveSizeCalculator
{
	struct CellCenter
	{
		Vector location;
		double distance;

		CellCenter(Vector location, double distance) : location(location), distance(distance) {}
		bool operator<(const CellCenter& other) const { return distance < other.distance; }
	};

public:
	virtual double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		SphereVisualizer& visualizer,
		int iVert);
};

class CaveSizeCalculatorLineFlow : public CaveSizeCalculator
{
public:
	virtual double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		SphereVisualizer& visualizer,
		int iVert);

private:
	MeanAverageWorkspace meanAverageWorkspace;
};

class CaveSizeCalculatorVoid : public CaveSizeCalculator
{
public:
	virtual double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		SphereVisualizer& visualizer,
		int iVert);
};
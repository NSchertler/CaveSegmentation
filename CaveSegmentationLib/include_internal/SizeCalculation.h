#pragma once

#include "Options.h"

#include "ImageProc.h"

#include "CGALCommon.h"
#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"
#include "LineProc.h"

#include <queue>
#include <omp.h>

#define DIFFERENT_SIGN(e1, e2) (((e1) > 0) != ((e2) > 0))

struct PositionValue
{
	Vector position;
	double value;
};

struct Empty
{

};

//Calculates the cave size by averaging the spherical radius over the Voronoi diagram defined by the minima of the radius field.
class CaveSizeCalculatorVoronoi
{
	struct CellCenter
	{
		Vector location;
		double distance;

		CellCenter(Vector location, double distance) : location(location), distance(distance) {}
		bool operator<(const CellCenter& other) const { return distance < other.distance; }
	};

public:
	typedef Empty TCustomData;

	template <typename TSphereVisualizer>
	static double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		TSphereVisualizer& visualizer,
		int iVert, TCustomData& customData)
	{
		double voronoiEdgeArea = 0.0;
		double voronoiDistance = 0.0;
		for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
		{
			std::priority_queue<CellCenter> nearestNeighbors;
			for (auto& clusterCenter : sphereDistanceMaxima)
			{
				double distance = 1 - clusterCenter.position * *it;
				if (nearestNeighbors.size() < 2)
					nearestNeighbors.push(CellCenter(clusterCenter.position, distance));
				else
				{
					if (distance < nearestNeighbors.top().distance)
					{
						nearestNeighbors.pop();
						nearestNeighbors.push(CellCenter(clusterCenter.position, distance));
					}
				}
			}
			Vector planeNormal = nearestNeighbors.top().location;
			nearestNeighbors.pop();
			if (nearestNeighbors.size() > 0)
				planeNormal = planeNormal - nearestNeighbors.top().location;

			double theta, phi, wTheta, wPhi;
			it.GetParameterSpaceRect(theta, phi, wTheta, wPhi);

			bool isOnVoronoiEdge = false;

			Vector r1(sin(phi) * sin(theta), sin(phi) * cos(theta), cos(phi));
			Vector r2(sin(phi + wPhi) * sin(theta), sin(phi + wPhi) * cos(theta), cos(phi + wPhi));
			if (DIFFERENT_SIGN(r1 * planeNormal, r2 * planeNormal))
				isOnVoronoiEdge = true;
			else
			{
				Vector r3(sin(phi + wPhi) * sin(theta + wTheta), sin(phi + wPhi) * cos(theta + wTheta), cos(phi + wPhi));
				if (DIFFERENT_SIGN(r1 * planeNormal, r3 * planeNormal))
					isOnVoronoiEdge = true;
				else
				{
					Vector r4(sin(phi) * sin(theta + wTheta), sin(phi) * cos(theta + wTheta), cos(phi));
					if (DIFFERENT_SIGN(r1 * planeNormal, r4 * planeNormal))
						isOnVoronoiEdge = true;
				}
			}

			if (isOnVoronoiEdge)
			{
				visualizer.DrawRect(theta, phi, wTheta, wPhi, SphereVisualizer::VORONOI_COLOR);

				double area = (double)sphereSampling.Area(it);
				voronoiDistance += area * sphereSampling.AccessContainerData(sphereDistances, it);
				voronoiEdgeArea += area;
			}
		}
		return voronoiDistance / voronoiEdgeArea;
	}
};

//Calculates the cave size by averaging the spherical radius field over a path network defined through line flow.
class CaveSizeCalculatorLineFlow 
{
public:
	typedef MeanAverageWorkspace TCustomData;

	template <typename TSphereVisualizer>
	static double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		TSphereVisualizer& visualizer,
		int iVert, TCustomData& workspace)
	{
		const int SEPARATING_CIRCLE_SAMPLE_RESOLUTION = 36;

		//The endpoints of all lines
		Vector p1, p2;

		//track lines through minima
		if (sphereDistanceMinima.size() >= 2)
		{
			//the two representative minima
			int min1, min2;
			if (sphereDistanceMinima.size() == 2)
			{
				min1 = 0;
				min2 = 1;
			}
			else
			{
				//the influence of the distance penalty
				const double lambda = 2.0;

				/* solve
				arg min   [ lambda * (1 - arc cos(min1 * min2) / PI) + (f(min1) + f(min2)) / (2 * max f) ]
				min1, min2
				*/

				//find max f
				double maxF = 0;
				for (auto& min : sphereDistanceMinima)
				{
					if (min.value > maxF)
						maxF = min.value;
				}

				double bestScore = std::numeric_limits<double>::infinity();
				for (int i1 = 0; i1 < sphereDistanceMinima.size(); ++i1)
				{
					auto& m1 = sphereDistanceMinima.at(i1);
					for (int i2 = i1 + 1; i2 < sphereDistanceMinima.size(); ++i2)
					{
						auto& m2 = sphereDistanceMinima.at(i2);
						double score = lambda * (1 - acos(m1.position * m2.position) / M_PI) + (m1.value + m2.value) / (2 * maxF);
						if (score < bestScore)
						{
							bestScore = score;
							min1 = i1;
							min2 = i2;
						}
					}
				}
			}

			p1 = sphereDistanceMinima[min1].position;
			p2 = sphereDistanceMinima[min2].position;
		}
		else
		{
			//only one minimum 

			p1 = sphereDistanceMinima.front().position;
			double phi, theta;
			sphereSampling.ParametersFromPoint(p1, phi, theta);
			theta += M_PI;
			if (theta > 2 * M_PI)
				theta -= 2 * M_PI;
			double minDistance = std::numeric_limits<double>::infinity();
			for (phi = 0; phi <= M_PI; phi += M_PI / (SPHERE_SAMPLING_RESOLUTION - 1))
			{
				double d = sphereSampling.AccessInterpolatedContainerData(sphereDistances, phi, theta);
				if (d < minDistance)
				{
					minDistance = d;
					p2 = sphereSampling.Point(phi, theta);
				}
			}
		}

		// (p1 + p2) and (p2 - p1) are orthogonal iff ||p1|| == ||p2||
		// (p1 + p2) x (p2 - p1) = 2 * p1 x p2
		Vector orthogonalAxis1 = p1 + p2; orthogonalAxis1 = orthogonalAxis1 / sqrt(orthogonalAxis1.squared_length());
		Vector orthogonalAxis2 = CGAL::cross_product(p1, p2); orthogonalAxis2 = orthogonalAxis2 / sqrt(orthogonalAxis2.squared_length());

#ifdef DRAW_DEBUG_IMAGES
		Gdiplus::SolidBrush passThroughBrush(Gdiplus::Color(255, 0, 0));
#endif

		const double SEPARATING_CIRCLE_SAMPLE_ANGULAR_DISTANCE = 2 * M_PI / SEPARATING_CIRCLE_SAMPLE_RESOLUTION;
		std::list<PositionGradient> separatingCircle;
		double minDistance = std::numeric_limits<double>::infinity();
		for (int i = 0; i < SEPARATING_CIRCLE_SAMPLE_RESOLUTION; ++i)
		{
			double angle = i * SEPARATING_CIRCLE_SAMPLE_ANGULAR_DISTANCE;
			PositionGradient pg;
			pg.position = orthogonalAxis1 * cos(angle) + orthogonalAxis2 * sin(angle);
			pg.position = pg.position * (1.0 / sqrt(pg.position.squared_length()));
#ifdef _DEBUG
			double phi, theta;
			sphereSampling.ParametersFromPoint(pg.position, phi, theta);
			assert(!isnan(phi));
			assert(!isnan(theta));
#endif
			separatingCircle.push_back(pg);
		}

		double partialAverageSize, partialLineLength;

		// let the separating line climb up on the mountains...
		LineFlow<true>(separatingCircle, sphereSampling, sphereDistances, distanceGradient, +1.0, partialAverageSize, partialLineLength, visualizer, L"separatingCircle_" + std::to_wstring(iVert));

		// find the angular distribution of the separating line

		// local coordinate system:
		Vector endpointDifference = p2 - p1; //local z axis
		Vector helpXAxis = (abs(endpointDifference.y()) < 0.01 && abs(endpointDifference.z()) < 0.01 ? Vector(0, 1, 0) : Vector(1, 0, 0));
		Vector localYAxis = CGAL::cross_product(endpointDifference, helpXAxis);
		Vector localXAxis = CGAL::cross_product(localYAxis, endpointDifference);
		localXAxis = localXAxis * sqrt(localYAxis.squared_length() / localXAxis.squared_length()); //make x and y equally long

		struct AngleValuePosition
		{
			AngleValuePosition(Vector& position) : position(position) {}
			void operator=(const AngleValuePosition& other) { position = other.position; }

			double angle;
			double value;
			Vector& position;
		};
		std::vector<AngleValuePosition> separatingLine, separatingLineLocalMinima;
		separatingLine.reserve(separatingCircle.size());

		double lastAngle;
		double lastValue = std::numeric_limits<double>::infinity();
		bool lastInResult = false;
		for (auto it = separatingCircle.begin(); it != separatingCircle.end(); ++it)
		{
			double localX = it->position * localXAxis;
			double localY = it->position * localYAxis;
			AngleValuePosition av(it->position);
			av.angle = atan2(localX, localY);
			if (it != separatingCircle.begin())
			{
				double diffToLast = av.angle - lastAngle;
				if (diffToLast < -M_PI)
				{
					av.angle += 2 * M_PI; //wrap around
					diffToLast += 2 * M_PI;
				}
				if (diffToLast < 0)
					continue; //assert monotonically increasing sequence							
			}
			av.value = sphereSampling.AccessInterpolatedContainerData(sphereDistances, it->position);

			separatingLine.push_back(av);

			//only use local minima
			if (lastValue < av.value) //don't insert points on the ascending slope
			{
				lastValue = av.value;
				lastInResult = false;
				continue;
			}
			else //remove points from the descending slope
			{
				lastValue = av.value;
				if (lastInResult)
					separatingLineLocalMinima.erase(separatingLineLocalMinima.end() - 1);
			}


			separatingLineLocalMinima.push_back(av);
			lastInResult = true;
			lastAngle = av.angle;
		}

		for (auto& sample : separatingLine)
		{
			double phi, theta;
			sphereSampling.ParametersFromPoint(sample.position, phi, theta);
			visualizer.FillCircle(theta, phi, 2, SphereVisualizer::SEPARATING_LINE_COLOR);
		}

		// Find the representative minima on the separating line
		std::vector<int> representativeMinimaOnSeparatingLine;
		std::vector<AngleValuePosition>* usedLine = &separatingLineLocalMinima;
		FindOptimalMeanAverage(separatingLineLocalMinima, CIRCLE_SUBSAMPLING_MIN_DISTANCE, CIRCLE_SUBSAMPLING_MAX_DISTANCE, workspace, representativeMinimaOnSeparatingLine);
		if (representativeMinimaOnSeparatingLine.size() == 0)
		{
			usedLine = &separatingLine;
			FindOptimalMeanAverage(separatingLine, CIRCLE_SUBSAMPLING_MIN_DISTANCE, CIRCLE_SUBSAMPLING_MAX_DISTANCE, workspace, representativeMinimaOnSeparatingLine);
		}

		double averageSize = 0;
		double lineLength = 0;

		int minimumIt = 0;
		for (int minimumIndex : representativeMinimaOnSeparatingLine)
		{
			std::list<PositionGradient> minimaLine;

			Vector passingThroughPoint = usedLine->at(minimumIndex).position;

			double phi, theta;
			sphereSampling.ParametersFromPoint(passingThroughPoint, phi, theta);
			visualizer.FillCircle(theta, phi, 4, SphereVisualizer::SEPARATING_LINE_COLOR);

			//Calculate the arc through both end points and the minimum
			Vector circleAxis = CGAL::cross_product(p1 - passingThroughPoint, p2 - passingThroughPoint);
			Vector m = (circleAxis * p1) / circleAxis.squared_length() * circleAxis; //the circle center
			Vector v1 = p1 - m; //the first circle intercept
			Vector v2 = CGAL::cross_product(v1, circleAxis);
			v2 = v2 / sqrt(v2.squared_length()) * sqrt(v1.squared_length());

			Vector localP2 = p2 - m;
			Vector localPassthrough = passingThroughPoint - m;
			double betaP2 = atan2(localP2 * v2, localP2 * v1);
			double betaPassThrough = atan2(localPassthrough * v2, localPassthrough * v1);

			double betaMin, betaMax;
			if (betaP2 > 0)
			{
				if (betaPassThrough > 0 && betaPassThrough < betaP2)
				{
					betaMin = 0; betaMax = betaP2;
				}
				else
				{
					betaMin = betaP2; betaMax = 2 * M_PI;
				}
			}
			else
			{
				if (betaPassThrough < 0 && betaPassThrough > betaP2)
				{
					betaMin = betaP2; betaMax = 0;
				}
				else
				{
					betaMin = -2 * M_PI; betaMax = betaP2;
				}
			}

			for (double beta = betaMin; beta <= betaMax; beta += LINE_POINT_DISTANCE)
			{
				Vector p = m + v1 * cos(beta) + v2 * sin(beta);
				minimaLine.push_back({ p });

				double phi, theta;
				sphereSampling.ParametersFromPoint(p, phi, theta);

				visualizer.FillCircle(theta, phi, 2, SphereVisualizer::SEPARATING_CIRCLE_COLOR);
			}

			//Let the line flow towards the valleys
			LineFlow<false>(minimaLine, sphereSampling, sphereDistances, distanceGradient, -1.0, partialAverageSize, partialLineLength, visualizer, L"minimaLineFlow_" + std::to_wstring(iVert) + L"_" + std::to_wstring(minimumIt));

			lineLength += partialLineLength;
			averageSize += partialLineLength / lineLength * (partialAverageSize - averageSize);

			//draw flown line
			for (auto& p : minimaLine)
			{
				double phi, theta;
				sphereSampling.ParametersFromPoint(p.position, phi, theta);

				visualizer.FillCircle(theta, phi, 2, SphereVisualizer::FLOW_COLOR);
			}

			++minimumIt;
		}

		return averageSize;
	}
};

class CaveSizeCalculatorVoid
{
public:
	typedef Empty TCustomData;

	template <typename TSphereVisualizer>
	static double CalculateDistance(const RegularUniformSphereSampling& sphereSampling,
		const std::vector<std::vector<double>>& sphereDistances,
		const std::vector<std::vector<Vector>>& distanceGradient,
		const std::vector<PositionValue>& sphereDistanceMaxima,
		const std::vector<PositionValue>& sphereDistanceMinima,
		TSphereVisualizer& visualizer,
		int iVert, TCustomData& customData)
	{
		return 0.0;
	}
};
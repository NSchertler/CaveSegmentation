#include "GraphProc.h"

double gauss(double x, double variance)
{
	return exp(-x*x / 2 / variance);
}

double gaussIntegrate(double standardDeviation, double lower, double upper)
{
	double sqrt2sigma = sqrt(2) * standardDeviation;
	return std::erf(upper / sqrt2sigma) - std::erf(lower / sqrt2sigma);
}

void addEdgeNeighborsToSet(const EdgeOrientationDistance& eod, double distanceAtTip, const CurveSkeleton* s, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, std::set<EdgeOrientationDistance>& edgeSet, std::map<int, double>& distances)
{
	auto& edge = s->edges.at(eod.edge);
	const int baseVertex = (eod.propagationReversed ? edge.second : edge.first);
	const int tipVertex = (eod.propagationReversed ? edge.first : edge.second);
	//propagation direction is baseVertex--->tipVertex

	const std::vector<int>& adj = adjacency.at(tipVertex);

	auto& tipPosition = s->vertices.at(tipVertex).position;
	for (auto adjV : adj)
	{
		if (adjV == baseVertex) //wrong direction
			continue;

		int e = vertexPairToEdge.at(std::pair<int, int>(adjV, tipVertex));
		auto distanceEntry = distances.find(adjV);
		if (distanceEntry == distances.end() || distanceEntry->second > distanceAtTip)
		{
			if (distanceEntry != distances.end())
				edgeSet.erase(EdgeOrientationDistance(e, distanceEntry->second));
			bool propReversed = (s->edges.at(e).first == adjV);
			edgeSet.insert(EdgeOrientationDistance(e, distanceAtTip, propReversed, eod.measureReversed ^ eod.propagationReversed ^ propReversed));
			distances[e] = distanceAtTip;
		}
	}
}
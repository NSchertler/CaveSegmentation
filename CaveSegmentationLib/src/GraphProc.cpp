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

void addEdgeNeighborsToSet(const EdgeOrientationDistance& eod, double distanceAtTip, const IGraph& graph, std::set<EdgeOrientationDistance>& edgeSet, std::map<size_t, double>& distances)
{
	size_t v1, v2;
	graph.IncidentVertices(eod.edge, v1, v2);
	auto baseVertex = (eod.propagationReversed ? v2 : v1);
	auto tipVertex = (eod.propagationReversed ? v1 : v2);
	//propagation direction is baseVertex--->tipVertex

	auto& adj = graph.AdjacentNodes(tipVertex);

	auto& tipPosition = graph.VertexPosition(tipVertex);
	for (auto adjV : adj)
	{
		if (adjV == baseVertex) //wrong direction
			continue;

		size_t e = graph.EdgeIdFromVertexPair(adjV, tipVertex);
		auto distanceEntry = distances.find(adjV);
		if (distanceEntry == distances.end() || distanceEntry->second > distanceAtTip)
		{
			if (distanceEntry != distances.end())
				edgeSet.erase(EdgeOrientationDistance(e, distanceEntry->second));
			size_t incident1, incident2;
			graph.IncidentVertices(e, incident1, incident2);
			bool propReversed = (incident1 == adjV);
			edgeSet.insert(EdgeOrientationDistance(e, distanceAtTip, propReversed, eod.measureReversed ^ eod.propagationReversed ^ propReversed));
			distances[e] = distanceAtTip;
		}
	}
}
#pragma once

#include <vector>
#include <CurveSkeleton.h>
#include <map>
#include <set>
#include <unordered_set>

#include "IGraph.h"

//Returns the unnormalized value of the Gaussian normal distribution
extern double gauss(double x, double variance);

//Returns the unnormalized definite integral of the Gaussian normal distribution between the given limits
extern double gaussIntegrate(double standardDeviation, double lower, double upper);

struct NodeDistance
{
	int node;
	double distance;

	NodeDistance(int node, double distance) : node(node), distance(distance) {}

	bool operator<(const NodeDistance& rhs) const
	{
		if(distance != rhs.distance)
			return distance < rhs.distance; 
		return node < rhs.node;
	}
};

class IDFSHook
{
	virtual void processNode(const NodeDistance& node) = 0;
};

// Performs a depth-first search on the graph, starting from iVert.
// The methods of the hook object are called during the execution of the search.
// THook must obey to class IDFSHook
template <typename THook>
void graphDFS(THook& hook, const std::vector<CurveSkeleton::Vertex>& vertices, int iVert, const std::vector<std::vector<int>>& adjacency, double searchDistance)
{
	//perform depth-first search

	std::stack<NodeDistance> dfsStack;
	dfsStack.emplace(iVert, 0.0);

	std::unordered_set<int> visited;
	visited.insert(iVert);

	while (!dfsStack.empty())
	{
		NodeDistance current = dfsStack.top();
		dfsStack.pop();
		
		hook.processNode(current);		

		auto currentP = vertices.at(current.node).position;

		for (int adj : adjacency.at(current.node))
		{
			double distance = current.distance + (vertices.at(adj).position - currentP).norm();
			if (distance <= searchDistance && visited.find(adj) == visited.end())
			{
				visited.insert(adj);
				dfsStack.emplace(adj, distance);
			}
		}
	}
}

template <typename T>
class MaxSearchDFSHook : IDFSHook
{
public:
	MaxSearchDFSHook(const std::vector<T>& source)
		: maximum(std::numeric_limits<T>::min()), source(source)
	{ }

	void processNode(const NodeDistance& node)
	{
		if (source.at(node.node) > maximum)
			maximum = source.at(node.node);
	}

	T getMaximum() const { return maximum; }

private:
	T maximum;
	const std::vector<T>& source;
};

template <typename T>
void findMax(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, double searchDistance, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		MaxSearchDFSHook<T> hook(source);
		graphDFS(hook, vertices, iVert, adjacency, searchDistance);
		target.at(iVert) = hook.getMaximum();
	}
}

template <typename T>
void findMax(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, std::function<double(int)> searchDistance, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		MaxSearchDFSHook<T> hook(source);
		graphDFS(hook, vertices, iVert, adjacency, searchDistance(iVert));
		target.at(iVert) = hook.getMaximum();
	}
}

template <typename T>
class MaxAdvectDFSHook : IDFSHook
{
public:
	MaxAdvectDFSHook(T advectValue, std::vector<T>& target)
		: advectValue(advectValue), target(target)
	{ }

	void processNode(const NodeDistance& node)
	{
		if (target.at(node.node) < advectValue)
			target.at(node.node) = advectValue;
	}

private:
	T advectValue;
	std::vector<T>& target;
};

template <typename T> 
void maxAdvect(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, std::function<double(int)> searchDistance, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		target.at(iVert) = source.at(iVert);
	}

	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		MaxAdvectDFSHook<T> hook(source.at(iVert), target);
		graphDFS(hook, vertices, iVert, adjacency, searchDistance(iVert));
	}
}

template <typename T>
T smoothSingleVertex(const std::vector<CurveSkeleton::Vertex>& vertices, int iVert, const std::vector<std::vector<int>>& adjacency, double smoothDeviation, const std::vector<T>& source)
{
	if (smoothDeviation <= 0)
		return source[iVert];

	double distanceThreshold = 3 * smoothDeviation; //Gaussian is practically 0 after 3 * standardDeviation

	//perform Dijkstra
	std::set<NodeDistance> activeNodes;
	activeNodes.insert({ iVert, 0.0 });

	std::map<int, double> minDistances;
	minDistances[iVert] = 0;

	double smoothVariance = smoothDeviation * smoothDeviation;
	double sumWeight = 0;
	T sumValue = 0;
	while (!activeNodes.empty())
	{
		const NodeDistance node = *activeNodes.begin();
		activeNodes.erase(activeNodes.begin());
		if (node.distance > distanceThreshold)
			break;

		auto v = source.at(node.node);
		if (!std::isnan(v))
		{
			double weight = gauss(node.distance, smoothVariance);
			sumWeight += weight;
			sumValue += (T)(weight * v);
		}

		auto& adj = adjacency.at(node.node);
		for (auto adjV : adj)
		{
			auto& v = vertices.at(adjV);
			double distance = (vertices.at(node.node).position - v.position).norm() + node.distance;
			auto distanceEntry = minDistances.find(adjV);
			if (distanceEntry == minDistances.end() || distance < distanceEntry->second)
			{
				if(distanceEntry != minDistances.end())
					activeNodes.erase({ adjV, distanceEntry->second });
				minDistances[adjV] = distance;
				activeNodes.insert({ adjV, distance });				
			}			
		}		
	}
	return static_cast<T>(sumValue / sumWeight);
}

template <typename T>
void smooth(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, double smoothDeviation, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		target.at(iVert) = smoothSingleVertex(vertices, iVert, adjacency, smoothDeviation, source);
	}
}

template <typename T>
void smooth(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, std::function<double(int)> smoothDeviation, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		target.at(iVert) = smoothSingleVertex(vertices, iVert, adjacency, smoothDeviation(iVert), source);
	}
}

struct EdgeOrientationDistance
{
	EdgeOrientationDistance(size_t edge, double distanceAtBase, bool propagationReversed, bool measureReversed)
		: edge(edge), distanceAtBase(distanceAtBase), propagationReversed(propagationReversed), measureReversed(measureReversed)
	{ }

	EdgeOrientationDistance(size_t edge, double distanceAtBase)
		: edge(edge), distanceAtBase(distanceAtBase)
	{ }

	size_t edge;
	double distanceAtBase;
	bool propagationReversed;
	bool measureReversed;

	bool operator<(const EdgeOrientationDistance& rhs) const
	{
		if (distanceAtBase != rhs.distanceAtBase)
			return distanceAtBase < rhs.distanceAtBase;
		return edge < rhs.edge;
	}
};

extern void addEdgeNeighborsToSet(const EdgeOrientationDistance& eod, double distanceAtTip, const IGraph& graph, std::set<EdgeOrientationDistance>& edgeSet, std::map<size_t, double>& distances);

template <typename T, bool DirectionDependentMeasure>
T smoothSingleEdge(const IGraph& graph, size_t iEdge, double smoothDeviation, const std::vector<T>& source)
{
	double smoothVariance = smoothDeviation * smoothDeviation;

	size_t firstV, secondV;
	graph.IncidentVertices(iEdge, firstV, secondV);	
	double halfEdgeLength = (graph.VertexPosition(firstV) - graph.VertexPosition(secondV)).norm() / 2;

	double distanceThreshold = 3 * smoothDeviation; //Gaussian is practically 0 after 3 * standardDeviation

	//perform Dijkstra
	std::set<EdgeOrientationDistance> activeEdges;
	std::map<size_t, double> minDistances;

	double sumWeight = gaussIntegrate(smoothDeviation, -halfEdgeLength, halfEdgeLength);
	T sumMeasure = sumWeight * source.at(iEdge);

	EdgeOrientationDistance initialEdge = { iEdge, 0, false, false };
	addEdgeNeighborsToSet(initialEdge, halfEdgeLength, graph, activeEdges, minDistances);
	initialEdge.propagationReversed = true;
	addEdgeNeighborsToSet(initialEdge, halfEdgeLength, graph, activeEdges, minDistances);

	while(!activeEdges.empty())
	{
		const EdgeOrientationDistance eod = *activeEdges.begin();
		activeEdges.erase(activeEdges.begin());
		if (eod.distanceAtBase > distanceThreshold)
			break;

		size_t currentV1, currentV2;
		graph.IncidentVertices(eod.edge, currentV1, currentV2);

		double edgeLength = (graph.VertexPosition(currentV1) - graph.VertexPosition(currentV2)).norm();
		double distanceAtTip = eod.distanceAtBase + edgeLength;
		double weight = gaussIntegrate(smoothDeviation, eod.distanceAtBase, distanceAtTip);

		sumWeight += weight;
		double v = source.at(eod.edge);
		if (DirectionDependentMeasure && eod.measureReversed)
			v = -v;
		sumMeasure += static_cast<T>(weight * v);

		addEdgeNeighborsToSet(eod, distanceAtTip, graph, activeEdges, minDistances);
	}

	return (T)((sumMeasure) / (sumWeight));
}

/// DirectionDependentMeasure: Set to true if the measure T must be inverted if an edge is traversed in its backwards direction.
template <typename T, bool DirectionDependentMeasure>
void smoothPerEdge(const IGraph& graph, double smoothDeviation, const std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		target.at(iEdge) = smoothSingleEdge<T, DirectionDependentMeasure>(graph, iEdge, smoothDeviation, source);
	}
}

template <typename T, bool DirectionDependentMeasure>
void smoothPerEdge(const IGraph& graph, std::function<double(int)> smoothDeviation, const std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < graph.NumberOfEdges(); ++iEdge)
	{
		target.at(iEdge) = smoothSingleEdge<T, DirectionDependentMeasure>(graph, iEdge, smoothDeviation(iEdge), source);
	}
}

template <typename T>
void deriveDfs(const int currentVertex, const int prevVertex, const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& children, std::vector<T>& source, std::vector<T>& target)
{
	T sumChildrenValues = 0;
	double sumChildrenDistance = 0;
	auto& adj = children[currentVertex];
	for (auto n : adj)
	{
		sumChildrenValues += source[n];
		sumChildrenDistance += (vertices[n].position - vertices[currentVertex].position).norm();
	}
	double prevDistance = (vertices[currentVertex].position - vertices[prevVertex].position).norm();
	T nextSum;
	double nextDistance;
	if (adj.size() == 0)
	{
		nextSum = source[currentVertex];
		nextDistance = 0;
	}
	else
	{
		nextSum = sumChildrenValues / adj.size();
		nextDistance = sumChildrenDistance / adj.size();
	}

	target[currentVertex] = (nextSum - source[prevVertex]) / (nextDistance + prevDistance);
	for (auto n : adj)
	{
		deriveDfs(n, currentVertex, vertices, children, source, target);
	}
}

//Estimates the first derivative of source with central differences and saves it in target.
template <typename T>
void derive(const int root, const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& children, std::vector<T>& source, std::vector<T>& target)
{
	int nextVertex = children[root][0];
	target[root] = (source[nextVertex] - source[root]) / (vertices[nextVertex].position - vertices[root].position).norm();
	deriveDfs(nextVertex, root, vertices, children, source, target);
}

/// Estimates the first derivative of source with central differences and saves it in target.
/// DirectionDependentMeasure: Set to true if the measure T must be inverted if an edge is traversed in its backwards direction.
template <typename T, bool DirectionDependentMeasure>
void derivePerEdge(const IGraph& graph, const std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < graph.NumberOfEdges(); ++iEdge)
	{
		//calculate central difference

		size_t v1, v2;
		graph.IncidentVertices(iEdge, v1, v2);
		double edgeLength = (graph.VertexPosition(v1) - graph.VertexPosition(v2)).norm();
		T currentValue = source.at(iEdge);

		T sumIn = 0, sumOut = 0;
		double distanceIn = 0, distanceOut = 0;

		//iterate neighbors of first vertex
		auto& inNeighbors = graph.AdjacentNodes(v1);
		int neighborCount = 0;
		for (auto neighbor : inNeighbors)
		{
			if (neighbor == v2)
				continue;
			auto edgeId = graph.EdgeIdFromVertexPair(neighbor, v1);
			T measure = source.at(edgeId);
			if (DirectionDependentMeasure)
			{
				size_t incident1, incident2;
				graph.IncidentVertices(edgeId, incident1, incident2);
				if (incident1 != neighbor)
					measure = -measure;
			}
			sumIn += measure;
			distanceIn += (graph.VertexPosition(neighbor) - graph.VertexPosition(v1)).norm();
			++neighborCount;
		}
		if (inNeighbors.size() == 1) //no incoming edges
		{
			sumIn = currentValue; //forward difference
			distanceIn = 0;
		}
		else
		{
			sumIn = (sumIn) / neighborCount;
			distanceIn = (distanceIn / inNeighbors.size() + edgeLength) / 2;
		}

		//iterate neighbors of second vertex
		auto& outNeighbors = graph.AdjacentNodes(v2);
		neighborCount = 0;
		for (auto neighbor : outNeighbors)
		{
			if (neighbor == v1)
				continue;
			auto edgeId = graph.EdgeIdFromVertexPair(v2, neighbor);
			T measure = source.at(edgeId);
			if (DirectionDependentMeasure)
			{
				size_t incident1, incident2;
				graph.IncidentVertices(edgeId, incident1, incident2);
				if (incident2 != neighbor)
					measure = -measure;
			}
			sumOut += measure;
			distanceOut += (graph.VertexPosition(neighbor) - graph.VertexPosition(v2)).norm();
			++neighborCount;
		}
		if (outNeighbors.size() == 1) //no outgoing edges
		{
			sumOut = currentValue; //backward difference
			distanceOut = 0;
		}
		else
		{
			sumOut = (sumOut) / neighborCount;
			distanceOut = (distanceOut / outNeighbors.size() + edgeLength) / 2;
		}

		//estimate derivative
		target[iEdge] = (sumOut - sumIn) / (distanceIn + distanceOut);
	}
}

template <typename T>
void derivePerEdgeFromVertices(CurveSkeleton* skeleton, std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		auto& edge = skeleton->edges.at(iEdge);
		target[iEdge] = (source[edge.second] - source[edge.first]) / (skeleton->vertices[edge.first].position - skeleton->vertices[edge.second].position).norm();
	}
}

//Checks if all edges between the reference vertex and its neighbors (excluding a given neighbor) have a lower or equal value than the given value.
//values - per edge values
template <typename TAccessor, typename T>
bool IsEdgeLocalMaximum(const IGraph& graph, const TAccessor& valueAccessor, int referenceVertex, int excludeNeighbor, T maxValue)
{
	for (int neighbor : graph.AdjacentNodes(referenceVertex))
	{
		if (neighbor == excludeNeighbor)
			continue;

		T neighborEdgeValue = valueAccessor(graph.EdgeIdFromVertexPair(referenceVertex, neighbor));
		if (neighborEdgeValue > maxValue)
		{
			return false;
		}
	}
	return true;
}
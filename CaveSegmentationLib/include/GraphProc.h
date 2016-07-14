#pragma once

#include <vector>
#include <CurveSkeleton.h>
#include <map>
#include <set>
#include <unordered_set>

//Returns the unnormalized value of the Gaussian normal distribution
extern double gauss(double x, double variance);

//Returns the unnormalized definite integral of the Gaussian normal distribution between the given limits
extern double gaussIntegrate(double standardDeviation, double lower, double upper);

extern void buildTreeDfs(const int currentVertex, const int prevVertex, const std::vector<std::vector<int>>& adjacency, std::vector<bool>& visited, std::vector<int>& parents, std::vector<std::vector<int>>& children);

extern void buildTree(const int root, const int vertexCount, const std::vector<std::vector<int>>& adjacency, std::vector<int>& parents, std::vector<std::vector<int>>& children);

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

template <typename T>
T findMaxSingleVertex(const std::vector<CurveSkeleton::Vertex>& vertices, int iVert, const std::vector<std::vector<int>>& adjacency, double searchDistance, const std::vector<T>& source)
{
	//perform DFS

	std::stack<NodeDistance> dfsStack;
	dfsStack.emplace(iVert, 0.0);

	std::unordered_set<int> visited;
	visited.insert(iVert);

	T max = std::numeric_limits<T>::lowest();

	while (!dfsStack.empty())
	{
		NodeDistance current = dfsStack.top();
		dfsStack.pop();
		if (source.at(current.node) > max)
			max = source.at(current.node);

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

	return max;
}

template <typename T>
void findMax(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, double searchDistance, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		target.at(iVert) = findMaxSingleVertex(vertices, iVert, adjacency, searchDistance, source);
	}
}

template <typename T>
void findMax(const std::vector<CurveSkeleton::Vertex>& vertices, const std::vector<std::vector<int>>& adjacency, std::function<double(int)> searchDistance, std::vector<T>& source, std::vector<T>& target)
{
	for (int iVert = 0; iVert < vertices.size(); ++iVert)
	{
		target.at(iVert) = findMaxSingleVertex(vertices, iVert, adjacency, searchDistance(iVert), source);
	}
}


template <typename T>
T smoothSingleVertex(const std::vector<CurveSkeleton::Vertex>& vertices, int iVert, const std::vector<std::vector<int>>& adjacency, double smoothDeviation, const std::vector<T>& source)
{
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
	EdgeOrientationDistance(int edge, double distanceAtBase, bool propagationReversed, bool measureReversed)
		: edge(edge), distanceAtBase(distanceAtBase), propagationReversed(propagationReversed), measureReversed(measureReversed)
	{ }

	EdgeOrientationDistance(int edge, double distanceAtBase)
		: edge(edge), distanceAtBase(distanceAtBase)
	{ }

	int edge;
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

extern void addEdgeNeighborsToSet(const EdgeOrientationDistance& eod, double distanceAtTip, const CurveSkeleton* s, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, std::set<EdgeOrientationDistance>& edgeSet, std::map<int, double>& distances);

template <typename T, bool DirectionDependentMeasure>
T smoothSingleEdge(const CurveSkeleton* skeleton, int iEdge, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, double smoothDeviation, std::vector<T>& source)
{
	double smoothVariance = smoothDeviation * smoothDeviation;

	auto& edge = skeleton->edges.at(iEdge);
	auto& firstV = skeleton->vertices.at(edge.first);
	auto& secondV = skeleton->vertices.at(edge.second);
	double halfEdgeLength = (firstV.position - secondV.position).norm() / 2;

	double distanceThreshold = 3 * smoothDeviation; //Gaussian is practically 0 after 3 * standardDeviation

	//perform Dijkstra
	std::set<EdgeOrientationDistance> activeEdges;
	std::map<int, double> minDistances;

	double sumWeight = gaussIntegrate(smoothDeviation, -halfEdgeLength, halfEdgeLength);
	T sumMeasure = sumWeight * source.at(iEdge);

	EdgeOrientationDistance initialEdge = { iEdge, 0, false, false };
	addEdgeNeighborsToSet(initialEdge, halfEdgeLength, skeleton, adjacency, vertexPairToEdge, activeEdges, minDistances);
	initialEdge.propagationReversed = true;
	addEdgeNeighborsToSet(initialEdge, halfEdgeLength, skeleton, adjacency, vertexPairToEdge, activeEdges, minDistances);

	while(!activeEdges.empty())
	{
		const EdgeOrientationDistance eod = *activeEdges.begin();
		activeEdges.erase(activeEdges.begin());
		if (eod.distanceAtBase > distanceThreshold)
			break;

		auto& edge = skeleton->edges.at(eod.edge);

		double edgeLength = (skeleton->vertices.at(edge.first).position - skeleton->vertices.at(edge.second).position).norm();
		double distanceAtTip = eod.distanceAtBase + edgeLength;
		double weight = gaussIntegrate(smoothDeviation, eod.distanceAtBase, distanceAtTip);

		sumWeight += weight;
		double v = source.at(eod.edge);
		if (DirectionDependentMeasure && eod.measureReversed)
			v = -v;
		sumMeasure += static_cast<T>(weight * v);

		addEdgeNeighborsToSet(eod, distanceAtTip, skeleton, adjacency, vertexPairToEdge, activeEdges, minDistances);
	}

	return (T)((sumMeasure) / (sumWeight));
}

/// DirectionDependentMeasure: Set to true if the measure T must be inverted if an edge is traversed in its backwards direction.
template <typename T, bool DirectionDependentMeasure>
void smoothPerEdge(const CurveSkeleton* skeleton, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, double smoothDeviation, std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		target.at(iEdge) = smoothSingleEdge<T, DirectionDependentMeasure>(skeleton, iEdge, adjacency, vertexPairToEdge, smoothDeviation, source);
	}
}

template <typename T, bool DirectionDependentMeasure>
void smoothPerEdge(const CurveSkeleton* skeleton, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, std::function<double(int)> smoothDeviation, std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		target.at(iEdge) = smoothSingleEdge<T, DirectionDependentMeasure>(skeleton, iEdge, adjacency, vertexPairToEdge, smoothDeviation(iEdge), source);
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
void derivePerEdge(CurveSkeleton* skeleton, const std::vector<std::vector<int>>& adjacency, const std::map<std::pair<int, int>, int>& vertexPairToEdge, std::vector<T>& source, std::vector<T>& target)
{
	for (int iEdge = 0; iEdge < skeleton->edges.size(); ++iEdge)
	{
		//calculate central difference

		auto& edge = skeleton->edges.at(iEdge);
		double edgeLength = (skeleton->vertices[edge.first].position - skeleton->vertices[edge.second].position).norm();
		T currentValue = source.at(iEdge);

		T sumIn = 0, sumOut = 0;
		double distanceIn = 0, distanceOut = 0;

		//iterate neighbors of first vertex
		auto& inNeighbors = adjacency.at(edge.first);
		int neighborCount = 0;
		for (auto neighbor : inNeighbors)
		{
			if (neighbor == edge.second)
				continue;
			int edgeId = vertexPairToEdge.at(std::pair<int, int>(neighbor, edge.first));
			T measure = source.at(edgeId);
			if (DirectionDependentMeasure)
			{
				if (skeleton->edges.at(edgeId).first != neighbor)
					measure = -measure;
			}
			sumIn += measure;
			distanceIn += (skeleton->vertices.at(neighbor).position - skeleton->vertices.at(edge.first).position).norm();
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
		auto& outNeighbors = adjacency.at(edge.second);
		neighborCount = 0;
		for (auto neighbor : outNeighbors)
		{
			if (neighbor == edge.first)
				continue;
			int edgeId = vertexPairToEdge.at(std::pair<int, int>(edge.second, neighbor));
			T measure = source.at(edgeId);
			if (DirectionDependentMeasure)
			{
				if (skeleton->edges.at(edgeId).second != neighbor)
					measure = -measure;
			}
			sumOut += measure;
			distanceOut += (skeleton->vertices.at(neighbor).position - skeleton->vertices.at(edge.second).position).norm();
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
template <typename T>
bool checkEdgeMaximum(const std::vector<std::vector<int>>& adjacency, const std::vector<T>& values, const std::map<std::pair<int, int>, int>& vertexPairToEdge, int referenceVertex, int excludeNeighbor, T maxValue)
{
	for (int neighbor : adjacency.at(referenceVertex))
	{
		if (neighbor == excludeNeighbor)
			continue;

		T neighborEdgeValue = values.at(vertexPairToEdge.at(std::pair<int, int>(referenceVertex, neighbor)));
		if (neighborEdgeValue > maxValue)
		{
			return false;
		}
	}
	return true;
}
#pragma once

#include "../CaveData.h"

#include <queue>
#include <iostream>

class MaximumDescent
{
	static const int NO_SEGMENTATION = -2;

	/// returns if an adjacent passage has been found
	template <typename TSizeContainer>
	static bool MaximumDescentDFS(int currentVertex, double passageSize, TSizeContainer sizes, const std::vector<std::vector<int>>& adjacency, std::vector<int>& dfsVisitedState, std::vector<int>& visitedNodes, std::vector<int>& segmentation)
	{
		if (dfsVisitedState[currentVertex] != 0)
			return false; //node has already been visited

		dfsVisitedState[currentVertex] = 1;

		if (segmentation[currentVertex] != NO_SEGMENTATION)
			return false;

		if (sizes[currentVertex] <= passageSize)
		{
			return true;
		}

		bool finalResult = false;
		visitedNodes.push_back(currentVertex);
		for (auto n : adjacency.at(currentVertex))
			if (dfsVisitedState[n] == 0)
			{
				bool localResult = MaximumDescentDFS(n, passageSize, sizes, adjacency, dfsVisitedState, visitedNodes, segmentation);
				finalResult = finalResult || localResult;
			}

		return finalResult;
	}

public:
	static void FindChambers(const CaveData& data, std::vector<int>& segmentation)
	{
		struct Maximum
		{
			Maximum(int vId, double value) : vId(vId), value(value) {}

			int vId;
			double value;

			bool operator<(const Maximum& other) const { return value < other.value; }
		};

		segmentation.resize(data.skeleton->vertices.size(), NO_SEGMENTATION);

		std::priority_queue<Maximum> localMaxima;

		for (int iVertex = 0; iVertex < data.skeleton->vertices.size(); ++iVertex)
		{
			double size = data.caveSizes[iVertex];
			//check if this is a local maximum

			auto& neighbors = data.adjacency.at(iVertex);
			bool isLocalMaximum = true;
			for (int iNeighbor : neighbors)
			{
				if (data.caveSizes[iNeighbor] > size)
				{
					isLocalMaximum = false;
					break;
				}
			}

			if (isLocalMaximum)
				localMaxima.push(Maximum(iVertex, size));
		}

		const double CHAMBER_PASSAGE_SIZE_RATIO = 2.0;
		int nextCaveId = 0;

		enum DfsState
		{
			SearchEntrance, //Inside a potential chamber, search a potential entrance
			CheckPassageAhead, //Outside of a potential chamber, check the length of the adjacent passage
		};

		std::vector<int> dfsVisitedState(data.skeleton->vertices.size());
		while (!localMaxima.empty())
		{
			int maximumId = localMaxima.top().vId;
			double passageSize = localMaxima.top().value / CHAMBER_PASSAGE_SIZE_RATIO;
			localMaxima.pop();
			if (segmentation[maximumId] != NO_SEGMENTATION)
				continue;

			std::cout << "Searching entry with size " << passageSize << " for maximum." << std::endl;

			std::vector<int> visitedNodes;

			memset(&dfsVisitedState[0], 0, data.skeleton->vertices.size() * sizeof(int));
			bool foundAdjacentPassage = MaximumDescentDFS(maximumId, passageSize, data.caveSizes, data.adjacency, dfsVisitedState, visitedNodes, segmentation);

			if (foundAdjacentPassage)
			{
				std::cout << "Found entry." << std::endl;
				for (int vId : visitedNodes)
				{
					segmentation[vId] = nextCaveId;
				}
				++nextCaveId;
			}
		}
	}
};
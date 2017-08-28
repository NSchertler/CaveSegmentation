#include "ChamberAnalyzation/MaximumDescent.h"
#include "CaveDataAccessors.h"

const int NO_SEGMENTATION = -2;

/// returns if an adjacent passage has been found
template <typename TAccessor>
bool MaximumDescentDFS(int currentVertex, double passageSize, TAccessor accessor, const IGraph& graph, std::vector<int>& dfsVisitedState, std::vector<int>& visitedNodes, std::vector<int>& segmentation)
{
	if (dfsVisitedState[currentVertex] != 0)
		return false; //node has already been visited

	dfsVisitedState[currentVertex] = 1;

	if (segmentation[currentVertex] != NO_SEGMENTATION)
		return false;

	if (accessor(currentVertex) <= passageSize)
	{
		return true;
	}

	bool finalResult = false;
	visitedNodes.push_back(currentVertex);
	for (auto n : graph.AdjacentNodes(currentVertex))
		if (dfsVisitedState[n] == 0)
		{
			bool localResult = MaximumDescentDFS(n, passageSize, accessor, graph, dfsVisitedState, visitedNodes, segmentation);
			finalResult = finalResult || localResult;
		}

	return finalResult;
}

void MaximumDescent::FindChambers(const ICaveData& data, std::vector<int>& segmentation)
{
	struct Maximum
	{
		Maximum(int vId, double value) : vId(vId), value(value) {}

		int vId;
		double value;

		bool operator<(const Maximum& other) const { return value < other.value; }
	};

	segmentation.resize(data.NumberOfVertices(), NO_SEGMENTATION);

	std::priority_queue<Maximum> localMaxima;

	for (int iVertex = 0; iVertex < data.NumberOfVertices(); ++iVertex)
	{
		double size = data.CaveSize(iVertex);
		//check if this is a local maximum

		auto& neighbors = data.AdjacentNodes(iVertex);
		bool isLocalMaximum = true;
		for (int iNeighbor : neighbors)
		{
			if (data.CaveSize(iNeighbor) > size)
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

	std::vector<int> dfsVisitedState(data.NumberOfVertices());
	while (!localMaxima.empty())
	{
		int maximumId = localMaxima.top().vId;
		double passageSize = localMaxima.top().value / CHAMBER_PASSAGE_SIZE_RATIO;
		localMaxima.pop();
		if (segmentation[maximumId] != NO_SEGMENTATION)
			continue;

		std::cout << "Searching entry with size " << passageSize << " for maximum." << std::endl;

		std::vector<int> visitedNodes;

		memset(&dfsVisitedState[0], 0, data.NumberOfVertices() * sizeof(int));
		VertexCaveSizeAccessor accessor(data);
		bool foundAdjacentPassage = MaximumDescentDFS(maximumId, passageSize, accessor, data, dfsVisitedState, visitedNodes, segmentation);

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
#include "ChamberAnalyzation/Utils.h"

#include <stack>

void AssignUniqueChamberIndices(const CaveData & data, std::vector<int32_t>& segmentation)
{
	//assign unique chamber indices
	int nextSegment = 0;
	std::vector<bool> assignedNewIndex(segmentation.size(), false);
	for (int i = 0; i < segmentation.size(); ++i)
	{
		if (segmentation[i] < 0) //passage
			continue;
		if (assignedNewIndex[i])
			continue;

		std::stack<int> traversalStack;
		traversalStack.push(i);
		while (!traversalStack.empty())
		{
			int currentVertex = traversalStack.top();
			traversalStack.pop();
			if (segmentation[currentVertex] < 0)
				continue;
			if (assignedNewIndex[currentVertex])
				continue;
			assignedNewIndex[currentVertex] = true;
			segmentation[currentVertex] = nextSegment;
			for (auto n : data.adjacency[currentVertex])
				traversalStack.push(n);
		}
		++nextSegment;
	}
}

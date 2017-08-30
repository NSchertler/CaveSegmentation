#pragma once

#include "ICaveData.h"
#include "CaveSegmentationLib.h"

#include <queue>
#include <iostream>

//Simple greedy chamber detector that starts at a local maximum of the cave size and grows a chamber until it hits a possible entrance (identified by a constant ratio of the size).
class CAVESEGMENTATIONLIB_API MaximumDescent
{
	

public:
	static void FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose = true);
};
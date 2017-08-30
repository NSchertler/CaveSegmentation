#pragma once

#include "ICaveData.h"
#include "CaveSegmentationLib.h"

class CAVESEGMENTATIONLIB_API SizeBasedQPBO
{
public:
	static void FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose = true);
};
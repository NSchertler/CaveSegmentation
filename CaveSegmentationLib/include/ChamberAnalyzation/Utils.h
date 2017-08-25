#pragma once

#include <CaveSegmentationLib.h>

#include "../CaveData.h"

//Turns the binary segmentation (passage / chamber) into a multi-label segmentation through connected component analysis.
void CAVESEGMENTATIONLIB_API AssignUniqueChamberIndices(const CaveData& data, std::vector<int32_t>& segmentation);
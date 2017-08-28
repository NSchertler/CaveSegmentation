#pragma once

#include <CaveSegmentationLib.h>

#include "ICaveData.h"

//Turns the binary segmentation (passage / chamber) into a multi-label segmentation through connected component analysis.
void CAVESEGMENTATIONLIB_API AssignUniqueChamberIndices(const ICaveData& data, std::vector<int32_t>& segmentation);
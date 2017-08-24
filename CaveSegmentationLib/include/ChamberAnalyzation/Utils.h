#pragma once

#include "../CaveData.h"

//Turns the binary segmentation (passage / chamber) into a multi-label segmentation through connected component analysis.
void AssignUniqueChamberIndices(const CaveData& data, std::vector<int32_t>& segmentation);
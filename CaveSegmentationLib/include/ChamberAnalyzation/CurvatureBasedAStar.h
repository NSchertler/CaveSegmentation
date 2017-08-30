#pragma once

#include <ICaveData.h>
#include <CaveSegmentationLib.h>

//Tries to find the minimizer of an energy (the sum of entrance probability energies) through A* in the graph of possible solutions.
class CAVESEGMENTATIONLIB_API CurvatureBasedAStar
{	
public:
	static void FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose = true);
};
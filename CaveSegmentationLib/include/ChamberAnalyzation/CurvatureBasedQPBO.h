#pragma once

#include <iostream>

#include "ICaveData.h"
#include "CaveSegmentationLib.h"


//Finds the minimizer of the energy presented in the paper through QPBO 
class CAVESEGMENTATIONLIB_API CurvatureBasedQPBO
{

public:
	static void FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose = true);
};
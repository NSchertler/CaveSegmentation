#pragma once

#include "CaveSegmentationLib.h"

namespace Energies
{
	CAVESEGMENTATIONLIB_API extern double CURVATURE_TIP_POINT;
	CAVESEGMENTATIONLIB_API extern double DIRECTION_TOLERANCE;
}

extern CAVESEGMENTATIONLIB_API double entranceProbability(double normalizedCurvature);

// Returns the probability that an entrance is in the direction of the derivative.
extern CAVESEGMENTATIONLIB_API double directionProbability(double derivative);
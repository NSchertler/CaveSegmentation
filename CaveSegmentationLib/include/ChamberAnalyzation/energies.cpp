#include "ChamberAnalyzation/energies.h"

#include <math.h>
#include <algorithm>

double Energies::CURVATURE_TIP_POINT = 0.30;
double Energies::DIRECTION_TOLERANCE = 0.10;

double entranceProbability(double normalizedCurvature)
{
	//either use
	// sigma = maxCurvature / 3  (-> maxCurvature results in probability almost 1)
	// sigma = tippingCurvature / sqrt(2 * ln 2)  (-> tippingCurvature results in probability of 0.5)
	const double sigma = Energies::CURVATURE_TIP_POINT / sqrt(2 * log(2));
	if (normalizedCurvature <= 0)
		return 0;
	return std::min(0.999, 1 - exp(-normalizedCurvature * normalizedCurvature / (2 * sigma * sigma)));
}

// Returns the probability that an entrance is in the direction of the derivative.
double directionProbability(double derivative)
{
	//return derivative > 0 ? 1 : 0;
	return std::max(0.0, std::min(1.0, 0.5 + derivative / Energies::DIRECTION_TOLERANCE));
}
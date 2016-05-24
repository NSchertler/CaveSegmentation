#pragma once

namespace Energies
{
	extern double CURVATURE_TIP_POINT;
	extern double DIRECTION_TOLERANCE;
}

double entranceProbability(double normalizedCurvature);

// Returns the probability that an entrance is in the direction of the derivative.
double directionProbability(double derivative);
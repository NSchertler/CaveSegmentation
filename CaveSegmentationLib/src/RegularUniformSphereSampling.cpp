#include "RegularUniformSphereSampling.h"

// -----  RegularUniformSphereSampling -----

int closestPowerOfTwo(double x)
{
	double l = std::max(0.0, log2(x));
	int lower = 1 << (int)std::floor(l);
	int upper = 1 << (int)std::ceil(l);
	if (x - lower <= upper - x)
		return lower;
	else
		return upper;
}

RegularUniformSphereSampling::RegularUniformSphereSampling(const int nPhi)
	:nPhi(nPhi)
{
	assert(nPhi > 2);
	const int nThetaEq = (nPhi - 1) * 2; //number of samples along the equator	

	directionSamples.resize(nPhi);
	nTheta.resize(nPhi);
	areaElements.resize(nPhi);

	int nDirectionSamples = 0;
	_maxNTheta = closestPowerOfTwo(nThetaEq);
	for (int iPhi = 0; iPhi < nPhi; ++iPhi)
	{
		double phi = iPhi * M_PI / (nPhi - 1);
		nTheta[iPhi] = closestPowerOfTwo(sin(phi)*nThetaEq);
		directionSamples[iPhi] .resize(nTheta[iPhi]);
		if (iPhi == 0 || iPhi == nPhi - 1)
			areaElements[iPhi] = (1 - cos(M_PI / (nPhi - 1) / 2.0)) * 2 * M_PI;
		else
			areaElements[iPhi] = (cos(phi - M_PI / (nPhi - 1) / 2) - cos(phi + M_PI / (nPhi - 1) / 2)) * 2 * M_PI / nTheta[iPhi];
		for (int iTheta = 0; iTheta < nTheta[iPhi]; ++iTheta)
		{
			double theta = iTheta * 2 * M_PI / nTheta[iPhi];
			directionSamples[iPhi][iTheta] = Point((double)phi, (double)theta);
			++nDirectionSamples;
		}
	}
	std::cout << nDirectionSamples << " direction samples." << std::endl;
}

RegularUniformSphereSampling::~RegularUniformSphereSampling()
{
}

Vector RegularUniformSphereSampling::Point(double phi, double theta) const
{
	double sinPhi = sin(phi);
	return Vector(sinPhi * sin(theta), sinPhi * cos(theta), cos(phi));
}

void RegularUniformSphereSampling::ParametersFromPoint(const Vector & point, double & phi, double & theta) const
{
	double z = point.z();
	if (z < -1)
		z = -1;
	if (z > 1)
		z = 1;
	phi = acos(z);
	theta = atan2(point.x(), point.y());
	if (theta < 0)
		theta += 2 * M_PI;
}

RegularUniformSphereSampling::sample_iterator RegularUniformSphereSampling::begin() const
{
	return RegularUniformSphereSampling::sample_iterator(this, 0, 0);
}
RegularUniformSphereSampling::sample_iterator RegularUniformSphereSampling::end() const
{
	return RegularUniformSphereSampling::sample_iterator(this, nPhi, 0);
}

RegularUniformSphereSampling::neighbor_helper RegularUniformSphereSampling::Neighbors(int iPhi, int iTheta) const
{
	return neighbor_helper(this, iPhi, iTheta);
}

RegularUniformSphereSampling::neighbor_helper RegularUniformSphereSampling::Neighbors(sample_iterator it) const
{
	return Neighbors(it.iPhi, it.iTheta);
}

RegularUniformSphereSampling::range_helper RegularUniformSphereSampling::Neighbors(const Vector& center, double angularDistance) const
{
	return range_helper(this, center, angularDistance);
}

double RegularUniformSphereSampling::Area(const RegularUniformSphereSampling::sample_iterator& it) const
{
	return areaElements[it.iPhi];
}

RegularUniformSphereSampling::sample_iterator RegularUniformSphereSampling::ClosestSample(double phi, double theta) const
{
	double phiSlice = M_PI / (nPhi - 1);
	int iPhi = (int)(phi / phiSlice + 0.5);
	double thetaSlice = 2 * M_PI / nTheta[iPhi];
	int iTheta = (int)(theta / thetaSlice + 0.5);
	return sample_iterator(this, iPhi, iTheta);
}

int RegularUniformSphereSampling::MaxNTheta() const
{
	return _maxNTheta;
}

// -----  RegularUniformSphereSampling::sample_iterator -----

RegularUniformSphereSampling::sample_iterator::sample_iterator(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta)
	: sampling(sampling), iPhi(iPhi), iTheta(iTheta)
{ 
	if (iTheta < 0)
		this->iTheta += sampling->nTheta[iPhi];
	if (iTheta >= sampling->nTheta[iPhi])
		this->iTheta -= sampling->nTheta[iPhi];
}

const Vector& RegularUniformSphereSampling::sample_iterator::operator*() const
{
	return sampling->directionSamples[iPhi][iTheta];
}

bool RegularUniformSphereSampling::sample_iterator::operator!=(const sample_iterator& rhs) const
{
	return !(*this == rhs);
}
bool RegularUniformSphereSampling::sample_iterator::operator==(const sample_iterator& rhs) const
{
	if (iPhi >= sampling->nPhi && rhs.iPhi >= sampling->nPhi)
		return true;
	return sampling == rhs.sampling && iPhi == rhs.iPhi && iTheta == rhs.iTheta;
}

RegularUniformSphereSampling::sample_iterator& RegularUniformSphereSampling::sample_iterator::operator++()
{
	if (iTheta < sampling->nTheta[iPhi] - 1)
	{
		++iTheta;
		return *this;
	}
	else
	{
		++iPhi;
		iTheta = 0;
		return *this;
	}
}
void RegularUniformSphereSampling::sample_iterator::GetParameters(double & phi, double & theta) const
{
	phi = iPhi * M_PI / (sampling->nPhi - 1);
	theta = iTheta * 2 * M_PI / (sampling->nTheta[iPhi]);
}

void RegularUniformSphereSampling::sample_iterator::GetParameterSpaceRect(double& x, double& y, double& w, double& h)
{	
	h = M_PI / (sampling->nPhi - 1);
	y = h * iPhi - h/2;

	w = 2.0 * M_PI / sampling->nTheta[iPhi];
	x = iTheta * w - w/2;

	if (iPhi == 0)
	{
		h /= 2;
		y = 0;
	}
	
}

// -----  RegularUniformSphereSampling::neighbor_iterator -----

void calculateRowRange(int sourceRowResolution, int targetRowResolution, int i, int& from, int& to)
{
	double rowFactor = (double)targetRowResolution / sourceRowResolution;
	if (rowFactor < 1)
	{
		double targetSample = rowFactor * i;
		if (targetSample - floor(targetSample) == 0.5)
		{
			from = (int)floor(targetSample);
			to = (int)ceil(targetSample);
		}
		else
			from = to = (int)round(targetSample);
	}
	else if (rowFactor > 1)
	{
		from = (int)((i - 0.5) * rowFactor);
		to = (int)((i + 0.5) * rowFactor);
	}
	else
	{
		from = i - 1;
		to = i + 1;
	}
}

RegularUniformSphereSampling::neighbor_iterator::neighbor_iterator(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta, int state)
	: sampling(sampling), iPhi(iPhi), iTheta(iTheta), state(state), subState(0)
{
	if (iPhi >= 0 && iPhi < sampling->nPhi)
	{
		if (iPhi == 0)
		{
			topRowFrom = 0;
			topRowTo = -1;
			bottomRowFrom = 0;
			bottomRowTo = sampling->nTheta[1];
		}
		else if (iPhi == sampling->nPhi - 1)
		{
			topRowFrom = 0;
			topRowTo = sampling->nTheta[iPhi - 1];
			bottomRowFrom = 0;
			bottomRowTo = -1;
		}
		else
		{
			calculateRowRange(sampling->nTheta[iPhi], sampling->nTheta[iPhi - 1], iTheta, topRowFrom, topRowTo);
			calculateRowRange(sampling->nTheta[iPhi], sampling->nTheta[iPhi + 1], iTheta, bottomRowFrom, bottomRowTo);
		}
	}

	skipNonExistingStates();
}
const RegularUniformSphereSampling::sample_iterator RegularUniformSphereSampling::neighbor_iterator::operator*() const
{
	switch (state)
	{
	case 0: //left neighbor
		return sample_iterator(sampling, iPhi, iTheta - 1);
	case 1: //top row neighbors
		return sample_iterator(sampling, iPhi - 1, topRowFrom + subState);
	case 2: //right neighbor
		return sample_iterator(sampling, iPhi, iTheta + 1);
	case 3: //bottom row neighbors
		return sample_iterator(sampling, iPhi + 1, bottomRowTo - subState);
	}
	throw; //Invalid state
}
bool RegularUniformSphereSampling::neighbor_iterator::operator!=(const neighbor_iterator& rhs) const
{
	if (state > 3 && rhs.state > 3)
		return false;
	return sampling != rhs.sampling || iPhi != rhs.iPhi || iTheta != rhs.iTheta || state != rhs.state || subState != rhs.subState;
}

void RegularUniformSphereSampling::neighbor_iterator::skipNonExistingStates()
{
	while (true)
	{
		if ((state == 0 || state == 2) && (iPhi == 0 || iPhi == sampling->nPhi - 1))
			++state;
		else if (state == 1 && topRowTo < topRowFrom)
			++state;
		else if (state == 3 && bottomRowTo < bottomRowFrom)
			++state;
		else
			return;
	}
}

RegularUniformSphereSampling::neighbor_iterator& RegularUniformSphereSampling::neighbor_iterator::operator++()
{
	if (state == 1 || state == 3) //top or bottom row
	{
		++subState;
		if (state == 1 && subState > topRowTo - topRowFrom) //top
		{
			++state;
			subState = 0;
		}
		if (state == 3 && subState > bottomRowTo - bottomRowFrom) //bottom
		{
			++state;
			subState = 0;
		}
	}
	else
	{
		++state;
	}	

	skipNonExistingStates();
	
	return *this;
}

// -----  RegularUniformSphereSampling::neighbor_helper -----

RegularUniformSphereSampling::neighbor_helper::neighbor_helper(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta)
	: sampling(sampling), iPhi(iPhi), iTheta(iTheta)
{ }

RegularUniformSphereSampling::neighbor_iterator RegularUniformSphereSampling::neighbor_helper::begin() const
{
	return neighbor_iterator(sampling, iPhi, iTheta, 0);
}

RegularUniformSphereSampling::neighbor_iterator RegularUniformSphereSampling::neighbor_helper::end() const
{	
	return neighbor_iterator(sampling, iPhi, iTheta, 4);
}

// -----  RegularUniformSphereSampling::range_iterator -----

RegularUniformSphereSampling::range_iterator::range_iterator(const RegularUniformSphereSampling * sampling, const Vector & center, double angularDistance, bool finished)
	: sampling(sampling), finished(finished)
{
	if (!finished)
	{
		thetaCenter = atan2(center.x(), center.y());
		phiCenter = acos(center.z());
		scalarThreshold = cos(angularDistance);

		double phi = acos(center.z());
		double phiFrom = std::max(0.0, phi - angularDistance);
		double phiTo = std::min((double)M_PI, phi + angularDistance);
		iPhiFrom = (int)ceil(phiFrom * (sampling->nPhi - 1) / M_PI);
		iPhiTo = (int)floor(phiTo * (sampling->nPhi - 1) / M_PI);

		iPhi = iPhiFrom;
		calculateThetaRange();
	}
}

const RegularUniformSphereSampling::sample_iterator RegularUniformSphereSampling::range_iterator::operator*() const
{
	if (finished)
		throw;
	return sample_iterator(sampling, iPhi, iTheta);
}

bool RegularUniformSphereSampling::range_iterator::operator!=(const range_iterator & rhs) const
{
	if (finished && rhs.finished)
		return false;
	return iPhi != rhs.iPhi || iTheta != rhs.iTheta || finished != rhs.finished;
}

RegularUniformSphereSampling::range_iterator & RegularUniformSphereSampling::range_iterator::operator++()
{
	++iTheta;
	if (iTheta > iThetaTo)
		incrementIPhi();

	return *this;
}

void RegularUniformSphereSampling::range_iterator::calculateThetaRange()
{
	double phi = iPhi * M_PI / (sampling->nPhi - 1);
	double numerator = scalarThreshold - cos(phi) * cos(phiCenter);
	double denominator = sin(phi) * sin(phiCenter);
	if (denominator < 0)
		denominator = 0; //numerical stability

	//solve:
	//  denominator * cos theta >= numerator

	if (denominator == 0)
	{
		if (numerator <= 0)
		{
			//the entire range
			iThetaFrom = 0;
			iThetaTo = sampling->nTheta[iPhi];
		}
		else
		{
			//no valid solution
			iThetaFrom = 0;
			iThetaTo = -1;
		}
	}
	else
	{
		double ratio = numerator / denominator;
		if (ratio <= -1)
		{
			//the entire range
			iThetaFrom = 0;
			iThetaTo = sampling->nTheta[iPhi];
		}
		else if(ratio > 1)
		{
			//no valid solution
			iThetaFrom = 0;
			iThetaTo = -1;
		}
		else
		{
			double deltaTheta = acos(ratio);
			iThetaFrom = (int)ceil((thetaCenter - deltaTheta) * sampling->nTheta[iPhi] / (2 * M_PI));
			iThetaTo = (int)floor((thetaCenter + deltaTheta) * sampling->nTheta[iPhi] / (2 * M_PI));
		}
	}

	iTheta = iThetaFrom;
	if (iThetaFrom > iThetaTo)
		incrementIPhi();
}

void RegularUniformSphereSampling::range_iterator::incrementIPhi()
{
	++iPhi;
	if (iPhi > iPhiTo)
		finished = true;
	else
		calculateThetaRange();
}

// -----  RegularUniformSphereSampling::range_helper -----

RegularUniformSphereSampling::range_helper::range_helper(const RegularUniformSphereSampling * sampling, const Vector & center, double angularDistance)
	: sampling(sampling), center(center), angularDistance(angularDistance)
{
}

RegularUniformSphereSampling::range_iterator RegularUniformSphereSampling::range_helper::begin() const
{
	return RegularUniformSphereSampling::range_iterator(sampling, center, angularDistance);
}

RegularUniformSphereSampling::range_iterator RegularUniformSphereSampling::range_helper::end() const
{
	return RegularUniformSphereSampling::range_iterator(sampling, center, angularDistance, true);
}
#pragma once

#include "Options.h"

#include "CGALCommon.h"

class RegularUniformSphereSampling
{
public:
	RegularUniformSphereSampling(const int nPhi);
	~RegularUniformSphereSampling();

	Vector Point(double phi, double theta) const;
	void ParametersFromPoint(const Vector& point, double& phi, double& theta) const;

	class sample_iterator
	{
	public:
		sample_iterator(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta);
		const Vector& operator*() const;
		bool operator!=(const sample_iterator& rhs) const;
		bool operator==(const sample_iterator& rhs) const;
		sample_iterator& operator++();	

		void GetParameters(double& phi, double& theta) const;
		void GetParameterSpaceRect(double& x, double& y, double& w, double& h);
		
	private:
		const RegularUniformSphereSampling* sampling;
		
		int iPhi, iTheta;
		friend class RegularUniformSphereSampling;
		friend struct std::hash < RegularUniformSphereSampling::sample_iterator > ;
	};

	class neighbor_iterator
	{
	public:
		neighbor_iterator(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta, int state);
		const sample_iterator operator*() const;
		bool operator!=(const neighbor_iterator& rhs) const;
		neighbor_iterator& operator++();
	private:
		const RegularUniformSphereSampling* sampling;
		int iPhi, iTheta, state;
		int topRowFrom, topRowTo, bottomRowFrom, bottomRowTo;
		int subState;

		void neighbor_iterator::skipNonExistingStates();
	};

	class neighbor_helper
	{
	public:
		neighbor_helper(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta);
		neighbor_iterator begin() const;
		neighbor_iterator end() const;

	private:
		const RegularUniformSphereSampling* sampling;
		int iPhi, iTheta;
	};

	class range_iterator
	{
	public:
		range_iterator(const RegularUniformSphereSampling* sampling, const Vector& center, double angularDistance, bool finished = false);
		const sample_iterator operator*() const;
		bool operator!=(const range_iterator& rhs) const;
		range_iterator& operator++();
	private:

		void calculateThetaRange();
		void incrementIPhi();

		const RegularUniformSphereSampling* sampling;
		int iPhiFrom, iPhiTo, iPhi;
		int iThetaFrom, iThetaTo, iTheta;
		double thetaCenter, phiCenter;
		double scalarThreshold;
		bool finished;
	};

	class range_helper
	{
	public:
		range_helper(const RegularUniformSphereSampling* sampling, const Vector& center, double angularDistance);
		RegularUniformSphereSampling::range_iterator begin() const;
		RegularUniformSphereSampling::range_iterator end() const;

	private:
		const RegularUniformSphereSampling* sampling;
		const Vector& center;
		double angularDistance;
	};

	class opposite_neighbor_iterator
	{
	public:
		opposite_neighbor_iterator(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta, int state = 0);
		const std::pair<sample_iterator, sample_iterator> operator*() const;
		bool operator!=(const opposite_neighbor_iterator& rhs) const;
		opposite_neighbor_iterator& operator++();

	private:
		int iPhi, iTheta, state, states;
		int topRow, bottomRow;
		int topThetaFrom, topThetaTo, bottomThetaFrom, bottomThetaTo;
		const RegularUniformSphereSampling* sampling;
	};

	class opposite_neighbor_helper
	{
	public:
		opposite_neighbor_helper(const RegularUniformSphereSampling* sampling, int iPhi, int iTheta);
		RegularUniformSphereSampling::opposite_neighbor_iterator begin() const;
		RegularUniformSphereSampling::opposite_neighbor_iterator end() const;

	private:
		const RegularUniformSphereSampling* sampling;
		int iPhi, iTheta;
	};

	sample_iterator begin() const;
	sample_iterator end() const;
	neighbor_helper Neighbors(int iPhi, int iTheta) const;
	neighbor_helper Neighbors(sample_iterator it) const;
	range_helper Neighbors(const Vector& center, double angularDistance) const;
	opposite_neighbor_helper Opposite_neighbors(int iPhi, int iTheta) const;
	opposite_neighbor_helper Opposite_neighbors(sample_iterator it) const;
	double Area(const sample_iterator&) const;
	sample_iterator ClosestSample(double phi, double theta) const;
	int MaxNTheta() const;

	// TContainer must be a 2D container with a resize() method, e.g. std::vector<std::vector<DATA>>
	template<typename TContainer>
	void PrepareDataContainer(TContainer& container) const
	{
		container.resize(nPhi);
		for (int i = 0; i < nPhi; ++i)
		{
			container[i].resize(nTheta[i]);
		}
	}

	template<typename TContainer>
	typename TContainer::value_type::reference AccessContainerData(TContainer& container, sample_iterator it) const
	{
		return container[it.iPhi][it.iTheta];
	}

	template<typename TContainer>
	typename TContainer::value_type::const_reference AccessContainerData(const TContainer& container, sample_iterator it) const
	{
		return container[it.iPhi][it.iTheta];
	}

	template<typename TContainer, typename T = TContainer::value_type::value_type>
	const T AccessInterpolatedContainerData(const TContainer& container, double phi, double theta) const
	{

		double phiSlice = M_PI / (nPhi - 1);
		double iPhi = (phi / phiSlice);
		int lowerPhi = (int)floor(iPhi);
		int upperPhi = (int)ceil(iPhi);
		
		if (lowerPhi < 0 || std::isnan(iPhi) || upperPhi >= nPhi)
			std::cout << "Phi " << phi << " resulted in iPhi = " << iPhi << ", lowerPhi = " << lowerPhi << ", upperPhi = " << upperPhi << ", with phiSlice = " << phiSlice << ", nPhi = " << nPhi;

		double lowerThetaSlice = 2 * M_PI / nTheta[lowerPhi];
		double iLowerTheta = (theta / lowerThetaSlice);
		int lowerLeftTheta = (int)floor(iLowerTheta);
		int lowerRightTheta = (int)ceil(iLowerTheta);

		if (lowerLeftTheta >= nTheta[lowerPhi])
		{
			iLowerTheta -= nTheta[lowerPhi];
			lowerLeftTheta -= nTheta[lowerPhi];
			lowerRightTheta -= nTheta[lowerPhi];
		}

		//No interpolation required
		if(lowerPhi == upperPhi && lowerLeftTheta == lowerRightTheta)
			return container[lowerPhi][lowerLeftTheta];

		//Interpolation along the theta-axis
		if (lowerPhi == upperPhi)
		{
			double alpha = (iLowerTheta - lowerLeftTheta);
			return (1 - alpha ) * container[lowerPhi][lowerLeftTheta] + alpha * container[lowerPhi][lowerRightTheta % nTheta[lowerPhi]];
		}

		double upperThetaSlice = 2 * M_PI / nTheta[upperPhi];
		double iUpperTheta = (theta / upperThetaSlice);
		int upperLeftTheta = (int)floor(iUpperTheta);
		int upperRightTheta = (int)ceil(iUpperTheta);		

		if (upperLeftTheta >= nTheta[upperPhi])
		{
			iUpperTheta -= nTheta[upperPhi];
			upperLeftTheta -= nTheta[upperPhi];
			upperRightTheta -= nTheta[upperPhi];
		}

		T interpolLower, interpolUpper;

		//Interpolation along the phi-axis
		if (lowerLeftTheta == lowerRightTheta)
		{
			interpolLower = container[lowerPhi][lowerLeftTheta];
		}
		else
		{
			double alphaLowerTheta = (iLowerTheta - lowerLeftTheta);
			interpolLower = (1 - alphaLowerTheta) * container[lowerPhi][lowerLeftTheta] + alphaLowerTheta * container[lowerPhi][lowerRightTheta % nTheta[lowerPhi]];
		}

		if (upperLeftTheta == upperRightTheta)
		{
			interpolUpper = container[upperPhi][upperLeftTheta];
		}
		else
		{
			double alphaUpperTheta = (iUpperTheta - upperLeftTheta);
			interpolUpper = (1 - alphaUpperTheta) * container[upperPhi][upperLeftTheta] + alphaUpperTheta * container[upperPhi][upperRightTheta % nTheta[upperPhi]];
		}

		//Bilinear interpolation
		double alphaPhi = (iPhi - lowerPhi);
		return (1 - alphaPhi) * interpolLower + alphaPhi * interpolUpper;
	}

	template<typename TContainer, typename T = TContainer::value_type::value_type>
	const T AccessInterpolatedContainerData(const TContainer& container, const Vector& p) const
	{
		if (abs(p.squared_length() - 1) > 0.01)
			std::cout << "Point for container access has non-unit length:" << p << std::endl;

		double phi, theta;
		ParametersFromPoint(p, phi, theta);
		if (phi < 0 ||std::isnan(phi))
			std::cout << "Vector " << p << " resulted in phi = " << phi;
		return AccessInterpolatedContainerData<TContainer, T>(container, phi, theta);
	}
	
private:
	const int nPhi;
	Vector** directionSamples;

	int* nTheta;
	double* areaElements;

	int _maxNTheta;

	friend class sample_iterator;
};

namespace std
{
	template <> 
	struct hash < RegularUniformSphereSampling::sample_iterator >
	{
		size_t operator()(const RegularUniformSphereSampling::sample_iterator& x) const
		{
			return hash<size_t>()(hash<int>()(x.iPhi) + hash<int>()(x.iTheta));
		}
	};
}
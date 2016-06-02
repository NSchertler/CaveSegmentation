#include <stdafx.h>
#include "ViewModel.h"
#include <ChamberAnalyzation/energies.h>

ViewModel::ViewModel(QObject * parent) : QObject(parent), lookThrough(true), 
	tippingCurvature(Energies::CURVATURE_TIP_POINT), directionTolerance(Energies::DIRECTION_TOLERANCE),
	caveScaleKernelFactor(caveData.CAVE_SCALE_KERNEL_FACTOR), caveSizeKernelFactor(caveData.CAVE_SIZE_KERNEL_FACTOR), caveSizeDerivativeKernelFactor(caveData.CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR),
	hoveredElement(_hoveredElement), selectedVertex(_selectedVertex), cursorPos(_cursorPos)
{
	marker.x = std::numeric_limits<double>::quiet_NaN();

	connect(&caveScaleKernelFactor, &ObservableVariable<double>::changed, &caveData, &CaveGLData::SmoothAndDeriveDistances);
	connect(&caveSizeKernelFactor, &ObservableVariable<double>::changed, &caveData, &CaveGLData::SmoothAndDeriveDistances);
	connect(&caveSizeDerivativeKernelFactor, &ObservableVariable<double>::changed, &caveData, &CaveGLData::SmoothAndDeriveDistances);

	_cursorPos.x = std::numeric_limits<float>::quiet_NaN();
}

ViewModel::~ViewModel() 
{
	
}

void ViewModel::setMarker(glm::vec3 marker)
{
	this->marker = marker;
	emit markerChanged();
}

const glm::vec3 & ViewModel::getMarker() const
{
	return marker;
}

void ViewModel::setLookThrough(bool lt) 
{
	lookThrough = lt;
	emit lookThroughChanged();
}

bool ViewModel::getLookThrough() const
{
	return lookThrough;
}
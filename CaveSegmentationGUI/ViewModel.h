#pragma once
#include <QObject>

#include "CaveGLData.hpp"
#include "ObservableVariable.h"

class ViewModel : public QObject {
	Q_OBJECT

public:
	ViewModel(QObject * parent = Q_NULLPTR);
	~ViewModel();

	CaveGLData caveData;
	std::vector<int> selectedPath;

	void setMarker(glm::vec3 marker);
	const glm::vec3& getMarker() const;	

	void setLookThrough(bool);
	bool getLookThrough() const;

	ObservableVariable<double> tippingCurvature;
	ObservableVariable<double> directionTolerance;

	ObservableVariable<double> caveScaleKernelFactor;
	ObservableVariable<double> caveSizeKernelFactor;
	ObservableVariable<double> caveSizeDerivativeKernelFactor;

signals:
	void selectedPathChanged();
	void markerChanged();	
	void lookThroughChanged();	
	
private:

	glm::vec3 marker;

	bool lookThrough;
	
};

#pragma once
#include "QCustomPlot/qcustomplot.h"
#include "ViewModel.h"

//Widget for charts.
class DataPlot : public QCustomPlot
{
	Q_OBJECT

public:
	DataPlot(ViewModel& vm, QWidget * parent = Q_NULLPTR);
	~DataPlot();

private slots:
	void selectedPathChanged();
	void distancesChanged();
	void selectionChanged();

	void hideSelectedGraph();
	void showSelectedGraph();

	void energyDefChanged();

private:
	ViewModel& vm;

	void updatePlot(bool keepXAxis = false);
	void updateYAxesScaling();
	QVector<double> tValuesPerVertex;

	int caveSizeIdx, caveSizeUnsmoothedIdx, globalSizeIdx, caveSizeDerivativeIdx, caveSizeCurvatureIdx, normalizedCurvatureIdx, entranceProbIdx, directionProbIdx;

protected:
	void mouseMoveEvent(QMouseEvent*);
	void leaveEvent(QEvent*);
	void contextMenuEvent(QContextMenuEvent *event);
};
#include <stdafx.h>
#include "DataPlot.h"


#include <ChamberAnalyzation/energies.h>

DataPlot::DataPlot(ViewModel& vm, QWidget * parent) : QCustomPlot(parent), vm(vm)
{
	setBackground(this->palette().background().color());

	yAxis2->setVisible(true);
	yAxis2->grid()->setVisible(true);
	yAxis2->grid()->setZeroLinePen(QPen(Qt::black));
	yAxis2->grid()->setPen(Qt::NoPen);

	caveSizeIdx = graphCount();
	addGraph();
	graph(graphCount() - 1)->setName("Cave Size");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssDisc);
	graph(graphCount() - 1)->setPen(QPen(Qt::black));

	caveSizeUnsmoothedIdx = graphCount();
	addGraph();
	graph(graphCount() - 1)->setName("Cave Size Unsmoothed");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssDisc);
	graph(graphCount() - 1)->setPen(QPen(Qt::gray));	

	globalSizeIdx = graphCount();
	addGraph();
	graph(graphCount() - 1)->setName("Cave Scale");
	QPen globalSizePen;
	globalSizePen.setColor(QColor(30, 40, 255, 150));
	globalSizePen.setStyle(Qt::DotLine);
	globalSizePen.setWidth(2);
	graph(graphCount() - 1)->setPen(globalSizePen);
	graph(graphCount() - 1)->setVisible(false);

	caveSizeDerivativeIdx = graphCount();
	addGraph(xAxis, yAxis2);
	graph(graphCount() - 1)->setName("Cave Size Derivative");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssDiamond);
	graph(graphCount() - 1)->setPen(QPen(QColor(229, 123, 45)));
	graph(graphCount() - 1)->setVisible(false);

	caveSizeCurvatureIdx = graphCount();
	addGraph(xAxis, yAxis2);
	graph(graphCount() - 1)->setName("Cave Size Curvature");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssSquare);
	graph(graphCount() - 1)->setPen(QPen(QColor(128, 128, 255)));
	graph(graphCount() - 1)->setVisible(false);

	normalizedCurvatureIdx = graphCount();
	addGraph(xAxis, yAxis2);
	graph(graphCount() - 1)->setName("Normalized Curvature");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssSquare);
	graph(graphCount() - 1)->setPen(QPen(Qt::blue));	
	graph(graphCount() - 1)->setVisible(false);

	entranceProbIdx = graphCount();
	addGraph(xAxis, yAxis2);
	graph(graphCount() - 1)->setName("Entrance Probability");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssTriangle);
	graph(graphCount() - 1)->setPen(QPen(QColor(166,86,40)));

	directionProbIdx = graphCount();
	addGraph(xAxis, yAxis2);
	graph(graphCount() - 1)->setName("Direction Probability");
	graph(graphCount() - 1)->setScatterStyle(QCPScatterStyle::ssTriangleInverted);
	graph(graphCount() - 1)->setPen(QPen(QColor(77, 175, 74)));

	legend->setVisible(true);	
	
	setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
	axisRect()->setRangeDrag(Qt::Horizontal);
	axisRect()->setRangeZoomAxes(QCustomPlot::xAxis, nullptr);

	setInteraction(QCP::iSelectPlottables);
	setInteraction(QCP::iSelectLegend);

	connect(&vm, &ViewModel::selectedPathChanged, this, &DataPlot::selectedPathChanged);
	connect(&vm.caveData, &CaveGLData::distancesChanged, this, &DataPlot::distancesChanged);
	connect(this, &DataPlot::selectionChangedByUser, this, &DataPlot::selectionChanged);

	connect(&vm.tippingCurvature, &ObservableVariable<double>::changed, this, &DataPlot::energyDefChanged);
	connect(&vm.directionTolerance, &ObservableVariable<double>::changed, this, &DataPlot::energyDefChanged);
}

DataPlot::~DataPlot() 
{
	
}

void DataPlot::updatePlot(bool keepXAxis)
{
	if (vm.selectedPath.size() == 0 || !vm.caveData.skeleton || vm.caveData.caveSizes.size() == 0)
	{
		for(int i = 0; i < graphCount(); ++i)
			graph(i)->clearData();
		replot();

		return;
	}

	tValuesPerVertex.resize(vm.selectedPath.size());
	QVector<double> tValuesPerEdge(vm.selectedPath.size() - 1);
	tValuesPerVertex[0] = 0;
	double t = 0;
	for (int i = 1; i < vm.selectedPath.size(); ++i)
	{
		double edgeLength = (vm.caveData.skeleton->vertices.at(vm.selectedPath.at(i)).position - vm.caveData.skeleton->vertices.at(vm.selectedPath.at(i - 1)).position).norm();
		tValuesPerEdge[i - 1] = t + edgeLength / 2;
		t += edgeLength;
		tValuesPerVertex[i] = t;
	}

	QVector<double> caveSizes(vm.selectedPath.size());
	QVector<double> caveSizesUnsmoothed(vm.selectedPath.size());
	QVector<double> globalSize(vm.selectedPath.size());
	for (int i = 0; i < vm.selectedPath.size(); ++i)
	{
		caveSizes[i] = vm.caveData.caveSizes.at(vm.selectedPath.at(i));
		caveSizesUnsmoothed[i] = vm.caveData.caveSizeUnsmoothed.at(vm.selectedPath.at(i));
		globalSize[i] = vm.caveData.caveScale.at(vm.selectedPath.at(i));
	}

	QVector<double> caveSizeDerivatives(vm.selectedPath.size() - 1);
	QVector<double> caveSizeCurvatures(vm.selectedPath.size() - 1);
	QVector<double> normalizedCurvature(vm.selectedPath.size() - 1);
	QVector<double> entranceProb(vm.selectedPath.size() - 1);
	QVector<double> directionProb(vm.selectedPath.size() - 1);
	for (int i = 0; i < vm.selectedPath.size() - 1; ++i)
	{
		int v1 = vm.selectedPath.at(i);
		int v2 = vm.selectedPath.at(i + 1);
		int edge = vm.caveData.vertexPairToEdge.at(std::pair<int, int>(v1, v2));
		double inv = 1.0;
		//for direction dependent measures...
		if (v1 != vm.caveData.skeleton->edges.at(edge).first)
			inv = -1.0;

		caveSizeDerivatives[i] = vm.caveData.caveSizeDerivativesPerEdge.at(edge) * inv;
		caveSizeCurvatures[i] = vm.caveData.caveSizeCurvaturesPerEdge.at(edge);
		normalizedCurvature[i] = caveSizeCurvatures[i] * 0.5 * (vm.caveData.caveScale.at(v1) + vm.caveData.caveScale.at(v2));

		entranceProb[i] = entranceProbability(normalizedCurvature[i]);
		directionProb[i] = directionProbability(caveSizeDerivatives[i]);
	}

	graph(caveSizeIdx)->setData(tValuesPerVertex, caveSizes);	
	graph(caveSizeUnsmoothedIdx)->setData(tValuesPerVertex, caveSizesUnsmoothed);
	graph(globalSizeIdx)->setData(tValuesPerVertex, globalSize);
	graph(caveSizeDerivativeIdx)->setData(tValuesPerEdge, caveSizeDerivatives);
	graph(caveSizeCurvatureIdx)->setData(tValuesPerEdge, caveSizeCurvatures);
	graph(normalizedCurvatureIdx)->setData(tValuesPerEdge, normalizedCurvature);
	graph(entranceProbIdx)->setData(tValuesPerEdge, entranceProb);
	graph(directionProbIdx)->setData(tValuesPerEdge, directionProb);

	if(!keepXAxis)
		xAxis->setRange(0, t);	
	updateYAxesScaling();

	replot();	
}

void DataPlot::updateYAxesScaling()
{
	yAxis->rescale(true);
	yAxis->setRangeLower(0);
	for(auto g : yAxis->graphs())
		if (g->visible())
		{
			yAxis->setRangeUpper(yAxis->range().upper * 1.1);
			break;
		}

	yAxis2->rescale(true);
	for (auto g : yAxis2->graphs())
		if (g->visible())
		{
			auto yAxis2Range = yAxis2->range();
			yAxis2->setRange(yAxis2Range.lower - yAxis2Range.size() * 0.1, yAxis2Range.upper + yAxis2Range.size() * 0.1);
			break;
		}
	
	if (graph(entranceProbIdx)->visible() || graph(directionProbIdx)->visible())
	{
		auto yAxis2Range = yAxis2->range();
		yAxis2Range = yAxis2->range();
		if (yAxis2Range.upper < 1.1)
			yAxis2->setRangeUpper(1.1);

		if (yAxis2Range.lower > 0)
			yAxis2->setRangeLower(0);
	}
}

void DataPlot::mouseMoveEvent(QMouseEvent *e)
{	
	QCustomPlot::mouseMoveEvent(e);

	if (vm.selectedPath.size() == 0)
		return;

	double t = xAxis->pixelToCoord(e->x());
	if (t < 0 || t > tValuesPerVertex.at(tValuesPerVertex.size() - 1))
	{
		vm.setMarker(glm::vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0));
		return;
	}

	//binary search for index
	int lower = 0, upper = tValuesPerVertex.size() - 1;
	while (upper - lower > 1)
	{
		int half = (lower + upper) / 2;
		double halfValue = tValuesPerVertex.at(half);
		if (halfValue < t)
			lower = half;
		else
			upper = half;
	}

	double interpol = (t - tValuesPerVertex.at(lower)) / (tValuesPerVertex.at(upper) - tValuesPerVertex.at(lower));
	auto pos = (1 - interpol) * vm.caveData.skeleton->vertices.at(vm.selectedPath.at(lower)).position + interpol * vm.caveData.skeleton->vertices.at(vm.selectedPath.at(upper)).position;
	vm.setMarker(glm::vec3(pos.x(), pos.y(), pos.z()));
}

void DataPlot::leaveEvent(QEvent *e)
{
	QCustomPlot::leaveEvent(e);
	vm.setMarker(glm::vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0));
}

void DataPlot::selectionChanged()
{
	// synchronize selection of graphs with selection of corresponding legend items:
	for (int i = 0; i < graphCount(); ++i)
	{
		QCPGraph *g = graph(i);
		QCPPlottableLegendItem *item = legend->itemWithPlottable(g);
		if (item->selected() || g->selected())
		{
			item->setSelected(true);
			g->setSelected(true);
		}
	}
}

void DataPlot::contextMenuEvent(QContextMenuEvent *event)
{
	QMenu *menu = new QMenu(this);
	menu->setAttribute(Qt::WA_DeleteOnClose);

	if(selectedGraphs().size() == 0)
		menu->addAction("Select a graph to toggle visibility")->setEnabled(false);	
	else
	{
		if (selectedGraphs().at(0)->visible())
			menu->addAction("Hide selected graph", this, &DataPlot::hideSelectedGraph);
		else
			menu->addAction("Show selected graph", this, &DataPlot::showSelectedGraph);
	}

	menu->popup(mapToGlobal(event->pos()));
}

void DataPlot::selectedPathChanged()
{
	updatePlot();
}

void DataPlot::distancesChanged()
{
	updatePlot(true);
}

void DataPlot::hideSelectedGraph()
{
	selectedGraphs().at(0)->setVisible(false);
	updateYAxesScaling();
	replot();
}

void DataPlot::showSelectedGraph()
{
	selectedGraphs().at(0)->setVisible(true);
	updateYAxesScaling();
	replot();
}

void DataPlot::energyDefChanged()
{
	updatePlot(true);
}
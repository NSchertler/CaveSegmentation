#ifndef PCPLOT_H
#define PCPLOT_H

#include <QtWidgets/QMainWindow>
#include "ui_pcplot.h"

#include <QPainter>

class PCPlot : public QMainWindow
{
	Q_OBJECT

public:
	PCPlot(QWidget *parent = 0);
	~PCPlot();


private:
	Ui::PCPlotClass ui;

	std::vector<float> data;
};

#endif // PCPLOT_H

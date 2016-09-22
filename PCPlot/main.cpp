#include "pcplot.h"
#include <QtWidgets/QApplication>

#pragma once

int main(int argc, char *argv[])
{


	QApplication a(argc, argv);
	PCPlot w;
	w.show();
	return a.exec();
}

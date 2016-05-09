#include "stdafx.h"

#include "cavesegmentationgui.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	CaveSegmentationGUI w;
	w.show();
	return a.exec();
}

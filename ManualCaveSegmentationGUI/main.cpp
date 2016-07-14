#include "ManualCaveSegmentation.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ManualCaveSegmentation w;
	w.show();
	return a.exec();
}

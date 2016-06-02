#include "stdafx.h"

#include "cavesegmentationgui.h"
#include <QtWidgets/QApplication>
#include <QCommandLineParser>

int main(int argc, char *argv[])
{
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

	QApplication a(argc, argv);

	QCommandLineParser parser;
	QCommandLineOption stereoOption("stereo", "Enables side-by-side stereo rendering.");
	parser.addOption(stereoOption);	

	QCommandLineOption stretchStereoOption("stretch", "Should be specified if a side-by-side stereo display is used that stretches the image.");
	parser.addOption(stretchStereoOption);

	QCommandLineOption screenNumberOption("screen", "Specify the screen number on which to display stereo windows.", "screenNumber", "-1");
	parser.addOption(screenNumberOption);

	QCommandLineOption dataDirOption("data", "Specify the data directory from which to load data.", "directory");
	parser.addOption(dataDirOption);

	parser.process(a);

	AppOptions o;
	o.stereo = parser.isSet(stereoOption);	
	o.aspectMultiplier = (parser.isSet(stretchStereoOption) ? 2.0f : 1.0f);
	o.screenNumber = parser.value(screenNumberOption).toInt();
	o.dataDir = parser.value(dataDirOption);

	a.setStyle("fusion");

	CaveSegmentationGUI w(o);
	w.show();
	return a.exec();
}

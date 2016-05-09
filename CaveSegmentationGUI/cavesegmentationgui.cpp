#include "stdafx.h"

#include "cavesegmentationgui.h"
#include "CaveDataGLView.h"
#include <qmessagebox.h>
#include <qfiledialog.h>

QString initialDirectory = "D:\\Data\\CaveSegmentation\\data";

CaveSegmentationGUI::CaveSegmentationGUI(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	auto glView = new CaveDataGLView(data, this);
	this->setCentralWidget(glView);
	
	connect(ui.btnLoadOff, &QPushButton::clicked, this, &CaveSegmentationGUI::loadOff);	
}

CaveSegmentationGUI::~CaveSegmentationGUI()
{
}


void CaveSegmentationGUI::loadOff(bool)
{
		auto filename = QFileDialog::getOpenFileName(this, "Load OFF", initialDirectory, "3D Models (*.off)");
		if(!filename.isEmpty())
			data.LoadMesh(filename.toStdString());
}
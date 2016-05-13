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
	connect(ui.btnLoadSkeleton, &QPushButton::clicked, this, &CaveSegmentationGUI::loadSkeleton);
}

CaveSegmentationGUI::~CaveSegmentationGUI()
{
}


void CaveSegmentationGUI::loadOff(bool)
{
		auto filename = QFileDialog::getOpenFileName(this, "Load OFF", initialDirectory, "3D Models (*.off)");
		if (!filename.isEmpty())
		{
			data.LoadMesh(filename.toStdString());
			dataDirectory = QDir(filename);
			dataDirectory.cdUp();
		}
}

void CaveSegmentationGUI::loadSkeleton(bool)
{
	const QString skelFile = "model.skel";

	QString filename;
	if (dataDirectory.exists() && dataDirectory.exists(skelFile))
	{		
		filename = dataDirectory.absoluteFilePath(skelFile);
	}

	if(filename.isEmpty())
		filename = QFileDialog::getOpenFileName(this, "Load Skeleton", initialDirectory, "Skeleton (*.skel)");

	if (!filename.isEmpty())
	{
		CurveSkeleton* skeleton = LoadCurveSkeleton(filename.toStdString().c_str());
		data.SetSkeleton(skeleton);
	}
}

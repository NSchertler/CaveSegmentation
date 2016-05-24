#include "stdafx.h"

#include "cavesegmentationgui.h"
#include "CaveDataGLView.h"
#include <qmessagebox.h>
#include <qfiledialog.h>

#include <FileInputOutput.h>

QString initialDirectory = "E:\\Data\\CaveSegmentation\\data";

CaveSegmentationGUI::CaveSegmentationGUI(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);	

	auto glView = new CaveDataGLView(vm, this);
	this->setCentralWidget(glView);

	plot = std::make_unique<DataPlot>(vm, ui.grpData);
	ui.grpData->layout()->addWidget(plot.get());
	plot->setFixedHeight(300);

	ui.chkLookThrough->setChecked(vm.getLookThrough());
	connect(ui.chkLookThrough, &QCheckBox::stateChanged, this, &CaveSegmentationGUI::lookThroughChanged);

	vm.caveScaleKernelFactor.setupLabel(ui.lblCaveScaleKernel);
	vm.caveScaleKernelFactor.setupSlider(ui.sldCaveScaleKernel, 1.0, 20.0, 190);

	vm.caveSizeKernelFactor.setupLabel(ui.lblSizeKernel);
	vm.caveSizeKernelFactor.setupSlider(ui.sldSizeKernel, 0.01, 2.0, 199);

	vm.caveSizeDerivativeKernelFactor.setupLabel(ui.lblDerivativeKernel);
	vm.caveSizeDerivativeKernelFactor.setupSlider(ui.sldDerivativeKernel, 0.01, 2.0, 199);

	vm.tippingCurvature.setupLabel(ui.lblTippingCurvature);
	vm.tippingCurvature.setupSlider(ui.sldTippingCurvature, 0.01, 1.0, 99);

	vm.directionTolerance.setupLabel(ui.lblDirectionTolerance);
	vm.directionTolerance.setupSlider(ui.sldDirectionTolerance, 0.01, 2.0, 199);

	connect(&vm.caveData, &CaveGLData::distancesChanged, this, &CaveSegmentationGUI::segmentationParametersChanged);
	connect(&vm.tippingCurvature, &ObservableVariable<double>::changed, this, &CaveSegmentationGUI::segmentationParametersChanged);
	connect(&vm.directionTolerance, &ObservableVariable<double>::changed, this, &CaveSegmentationGUI::segmentationParametersChanged);

	connect(ui.btnLoadOff, &QPushButton::clicked, this, &CaveSegmentationGUI::loadOff);	
	connect(ui.btnLoadSkeleton, &QPushButton::clicked, this, &CaveSegmentationGUI::loadSkeleton);
	connect(ui.btnLoadDistances, &QPushButton::clicked, this, &CaveSegmentationGUI::loadDistances);
	connect(ui.btnRunSegmentation, &QPushButton::clicked, this, &CaveSegmentationGUI::runSegmentation);
	connect(ui.btnLoadSegmentation, &QPushButton::clicked, this, &CaveSegmentationGUI::loadSegmentation);
	connect(ui.btnResetDefaults, &QPushButton::clicked, this, &CaveSegmentationGUI::resetToDefaults);	
	connect(ui.btnSaveSegmentation, &QPushButton::clicked, this, &CaveSegmentationGUI::saveSegmentation);
	connect(ui.btnSaveSegmentedMesh, &QPushButton::clicked, this, &CaveSegmentationGUI::saveSegmentedMesh);

	setWindowState(windowState() | Qt::WindowMaximized);
}

CaveSegmentationGUI::~CaveSegmentationGUI()
{
}


void CaveSegmentationGUI::loadOff(bool)
{
	auto filename = QFileDialog::getOpenFileName(this, "Load OFF", initialDirectory, "3D Models (*.off)");
	if (!filename.isEmpty())
	{
		vm.selectedPath.clear();
		emit vm.selectedPathChanged();
		vm.caveData.LoadMesh(filename.toStdString());
		dataDirectory = QDir(filename);
		dataDirectory.cdUp();
	}
}

QString CaveSegmentationGUI::getFilepath(QString defaultFile, QString title, QString filter)
{
	QString filename;
	if (dataDirectory.exists() && dataDirectory.exists(defaultFile))
	{
		filename = dataDirectory.absoluteFilePath(defaultFile);
	}

	if (filename.isEmpty())
		filename = QFileDialog::getOpenFileName(this, title, initialDirectory, filter);
	return filename;
}

void CaveSegmentationGUI::loadSkeleton(bool)
{
	QString filename = getFilepath("model.skel", "Load Skeleton", "Skeleton (*.skel)");

	if (!filename.isEmpty())
	{
		CurveSkeleton* skeleton = LoadCurveSkeleton(filename.toStdString().c_str());
		vm.caveData.SetSkeleton(skeleton);
	}
}

void CaveSegmentationGUI::loadDistances(bool)
{
	QString filename = getFilepath("distances.bin", "Load Distances", "Binary File (*.bin)");

	if (!filename.isEmpty())
	{
		vm.caveData.LoadDistances(filename.toStdString());
		vm.caveData.SmoothAndDeriveDistances();
	}

}

void CaveSegmentationGUI::runSegmentation(bool)
{
	vm.caveData.RunSegmentation();
}

void CaveSegmentationGUI::loadSegmentation(bool)
{
	QString filename = QFileDialog::getOpenFileName(this, "Load Segmentation", initialDirectory, "Segmentation file (*.seg)");
	if (!filename.isEmpty())
	{
		vm.caveData.LoadSegmentation(filename.toStdString());
	}
}

void CaveSegmentationGUI::lookThroughChanged(int state)
{
	vm.setLookThrough(state == Qt::Checked);
}

void CaveSegmentationGUI::segmentationParametersChanged()
{
	if (ui.chkUpdateSegmentation->isChecked() && vm.caveData.skeleton)
	{
		vm.caveData.RunSegmentation();
	}
}

void CaveSegmentationGUI::resetToDefaults()
{
	vm.caveScaleKernelFactor.set(10.0);
	vm.caveSizeKernelFactor.set(0.2);
	vm.caveSizeDerivativeKernelFactor.set(0.2);
	vm.tippingCurvature.set(0.3);
	vm.directionTolerance.set(0.1);
}

void CaveSegmentationGUI::saveSegmentation()
{
	QString filename = QFileDialog::getSaveFileName(this, "Save Segmentation", QString(), "Segmentation file (*.seg)");
	if (!filename.isEmpty())
	{
		WriteSegmentation(filename.toStdString(), vm.caveData.segmentation);
	}
}

void CaveSegmentationGUI::saveSegmentedMesh()
{
	QString filename = QFileDialog::getSaveFileName(this, "Save Segmented Mesh", QString(), "3D Models (*.off)");
	if (!filename.isEmpty())
	{
		vm.caveData.WriteSegmentationColoredOff(filename.toStdString(), vm.caveData.segmentation);
	}
}
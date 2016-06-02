#include "stdafx.h"

#include "cavesegmentationgui.h"
#include <qmessagebox.h>
#include <qfiledialog.h>

#include <FileInputOutput.h>

QString initialDirectory;// = "E:\\Data\\CaveSegmentation\\data";

CaveSegmentationGUI::CaveSegmentationGUI(const AppOptions& o, QWidget *parent)
	: QMainWindow(parent)
{
	options = o;

	ui.setupUi(this);	

	glView = new CaveDataGLView(vm, (o.stereo ? -1 : 0), this);
	glView->setVirtualAspectMultiplier(o.aspectMultiplier);
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

	if (!o.stereo)
	{
		setWindowState(windowState() | Qt::WindowMaximized);
		connect(glView, &GLView::inited, this, &CaveSegmentationGUI::preloadData);
	}
	else
	{
		secondary = std::make_unique<Secondary>(ui.dockWidget, this);
		secondary->setWindowTitle(this->windowTitle());
		secondary->show();

		secondarySynchronizationTimer.setInterval(40);
		connect(&secondarySynchronizationTimer, &QTimer::timeout, this, &CaveSegmentationGUI::synchronizeSecondary);
		secondarySynchronizationTimer.start();

		connect(glView, &GLView::inited, this, &CaveSegmentationGUI::glViewInited);

		QRect rec = QApplication::desktop()->availableGeometry(o.screenNumber);
		setGeometry(rec.x(), rec.y(), rec.width() / 2, rec.height());
	}	
}

void CaveSegmentationGUI::glViewInited()
{
	if (options.stereo)
	{
		secondaryGlView = std::make_unique<CaveDataGLView>(vm, 1, secondary.get(), glView);
		secondaryGlView->setVirtualAspectMultiplier(options.aspectMultiplier);
		secondary->setCentralWidget(secondaryGlView.get());

		connect(secondaryGlView.get(), &GLView::inited, this, &CaveSegmentationGUI::preloadData);
	}	
}

void CaveSegmentationGUI::preloadData()
{
	if (!options.dataDir.isEmpty())
	{
		dataDirectory = QDir(options.dataDir);

		QString modelPath = dataDirectory.absoluteFilePath("model.off");

		if (QFile(modelPath).exists())
			vm.caveData.LoadMesh(modelPath.toStdString());
		else
			return;

		QString skeletonPath = dataDirectory.absoluteFilePath("model.skel");
		if (QFile(modelPath).exists())
			vm.caveData.SetSkeleton(LoadCurveSkeleton(skeletonPath.toStdString().c_str()));
		else
			return;

		QString distancesPath = dataDirectory.absoluteFilePath("distances.bin");
		if (QFile(distancesPath).exists())
		{
			vm.caveData.LoadDistances(distancesPath.toStdString());
			vm.caveData.SmoothAndDeriveDistances();
		}
		else
			return;
	}
}

void CaveSegmentationGUI::synchronizeSecondary()
{
	secondary->setGeometry(QRect(this->geometry().x() + QApplication::desktop()->availableGeometry(this).width() / 2, this->geometry().y(), this->geometry().width(), this->geometry().height()));

	secondary->CopyFromPrimary();

	if(secondaryGlView)
		secondaryGlView->setGeometry(glView->geometry());
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
#include "ManualCaveSegmentation.h"
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qformlayout>
#include <qlabel>
#include <qinputdialog>
#include <memory>

#include "CaveListDialog.h"
#include "AvailableCaveListDialog.h"
#include "UploadSegmentationDialog.h"

ManualCaveSegmentation::ManualCaveSegmentation(QWidget *parent)
	: QMainWindow(parent), data(std::make_shared<SharedData>())
{
	ui.setupUi(this);

	glView = new GLCaveView(this, data);
	this->setCentralWidget(glView);

	connect(ui.btnLoadModel, SIGNAL(clicked()), this, SLOT(load_model()));
	connect(ui.sldBrushSize, SIGNAL(valueChanged(int)), this, SLOT(brushSize_changed(int)));
	connect(ui.rbChamber, SIGNAL(toggled(bool)), this, SLOT(brushType_changed(bool)));
	connect(ui.rbPassage, SIGNAL(toggled(bool)), this, SLOT(brushType_changed(bool)));
	connect(ui.rbErase, SIGNAL(toggled(bool)), this, SLOT(brushType_changed(bool)));
	connect(ui.btnDownloadCaves, SIGNAL(clicked()), this, SLOT(downloadCaves()));
	connect(ui.sldNearClip, &QSlider::valueChanged, this, &ManualCaveSegmentation::nearClipChanged);
	connect(ui.btnSaveSegmentation, &QPushButton::clicked, this, &ManualCaveSegmentation::saveSegmentation);
	connect(ui.btnLoadSegmentation, &QPushButton::clicked, this, &ManualCaveSegmentation::loadSegmentation);
	connect(ui.btnUploadSegmentation, &QPushButton::clicked, this, &ManualCaveSegmentation::uploadSegmentation);
	connect(ui.btnUpdateServerURL, &QPushButton::clicked, this, &ManualCaveSegmentation::updateServerURL);

	if (GlobalData.serverUrl().isEmpty())
		updateServerURL();

	glView->setBrushType(ui.rbChamber->isChecked() ? Chamber : Passage);
	glView->setBrushSize(ui.sldBrushSize->value());

	data->view = glView;

	setWindowState(Qt::WindowMaximized);
}

ManualCaveSegmentation::~ManualCaveSegmentation()
{
}

void ManualCaveSegmentation::brushSize_changed(int v)
{
	glView->setBrushSize(v);
}

void ManualCaveSegmentation::brushType_changed(bool)
{
	if (ui.rbChamber->isChecked())
		glView->setBrushType(Chamber);
	else if(ui.rbPassage->isChecked())
		glView->setBrushType(Passage);
	else if(ui.rbErase->isChecked())
		glView->setBrushType(Erase);
}

void ManualCaveSegmentation::downloadCaves()
{
	auto dialog = new CaveListDialog(this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
}

void ManualCaveSegmentation::nearClipChanged()
{
	glView->setFrontViewCutoff(ui.sldNearClip->value() / (float)ui.sldNearClip->maximum());
}

void ManualCaveSegmentation::load_model()
{
	auto dialog = new AvailableCaveListDialog(data, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
}

void ManualCaveSegmentation::saveSegmentation()
{
	if (data->getMesh() == nullptr)
	{
		QMessageBox::critical(this, "Save Segmentation", "You have to load a cave first before you can save its segmentation.");
		return;
	}
	QString filename = QFileDialog::getSaveFileName(this, "Save Segmentation", "", "Cave Segmentation (*.caveseg)");
	if (filename.length() > 0)
	{
		try
		{
			data->getMesh()->saveSegmentation(filename.toStdString());
		}
		catch (...)
		{
			QMessageBox::critical(this, "Save Segmentation", "Error saving cave segmentation.");
		}
	}
}

void ManualCaveSegmentation::loadSegmentation()
{
	if (data->getMesh() == nullptr)
	{
		QMessageBox::critical(this, "Load Segmentation", "You have to load a cave first before you can load its segmentation.");
		return;
	}
	QString filename = QFileDialog::getOpenFileName(this, "Load Segmentation", "", "Cave Segmentation (*.caveseg)");
	if (filename.length() > 0)
	{
		try
		{
			data->getMesh()->loadSegmentation(filename.toStdString());
		}
		catch (...)
		{
			QMessageBox::critical(this, "Load Segmentation", "Error loading the specified cave segmentation. Make sure that the segmentation file belongs to this cave.");
		}
	}
}

void ManualCaveSegmentation::uploadSegmentation()
{
	if (data->getMesh() == nullptr)
	{
		QMessageBox::critical(this, "Upload Segmentation", "You have to load a cave first before you can upload its segmentation.");
		return;
	}
	
	auto dialog = new UploadSegmentationDialog(data, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
}

void ManualCaveSegmentation::updateServerURL()
{
	bool ok;
	QString newURL = QInputDialog::getText(this, "Specify Server URL", "Please specify the URL of the Manual Cave Segmentation Service", QLineEdit::Normal, GlobalData.serverUrl(), &ok);
	if (ok)
		GlobalData.setServerUrl(newURL);
}
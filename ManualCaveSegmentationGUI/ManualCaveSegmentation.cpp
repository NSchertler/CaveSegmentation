#include "ManualCaveSegmentation.h"
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qformlayout>
#include <qlabel>
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

	glView->setBrushType(ui.rbChamber->isChecked() ? Chamber : Passage);
	glView->setBrushSize(ui.sldBrushSize->value());

#ifdef ADMIN
	QPushButton* loadExternal = new QPushButton("Load External Cave", ui.grpData);
	ui.verticalLayout_4->addWidget(loadExternal);
	connect(loadExternal, &QPushButton::clicked, this, &ManualCaveSegmentation::loadExternalModel);
#endif

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

void ManualCaveSegmentation::loadExternalModel()
{
	auto filename = QFileDialog::getOpenFileName(this, "External Cave", QString(), "3D Models (*.off *.obj *.bin)");
	if (filename != nullptr)
	{
		data->setMesh(std::make_shared<Mesh>(data->view, filename.toStdString()));
	}
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
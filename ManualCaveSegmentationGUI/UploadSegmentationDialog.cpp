#include "UploadSegmentationDialog.h"
#include "Common.h"
#include "RestClient.h"
#include <QMessageBox>

UploadSegmentationDialog::UploadSegmentationDialog(std::shared_ptr<SharedData> data, QWidget* parent)
	: QDialog(parent), data(data)
{
	ui.setupUi(this);

	ui.cmbExpertise->addItem("1 - No Experience", 1);
	ui.cmbExpertise->addItem("2", 2);
	ui.cmbExpertise->addItem("3", 3);
	ui.cmbExpertise->addItem("4", 4);
	ui.cmbExpertise->addItem("5 - Expert", 5);

	ui.cmbCertainty->addItem("1 - Doubting", 1);
	ui.cmbCertainty->addItem("2", 2);
	ui.cmbCertainty->addItem("3", 3);
	ui.cmbCertainty->addItem("4", 4);
	ui.cmbCertainty->addItem("5 - Confident", 5);

	ui.progressBar->setVisible(false);

	connect(ui.buttonBox, &QDialogButtonBox::clicked, this, &UploadSegmentationDialog::dialogButtonClicked);
	connect(this, &UploadSegmentationDialog::actionFinished, this, &UploadSegmentationDialog::stopProgress);
}

void UploadSegmentationDialog::dialogButtonClicked(QAbstractButton* button)
{
	QDialogButtonBox::StandardButton stdButton = ui.buttonBox->standardButton(button);
	if (stdButton == QDialogButtonBox::StandardButton::Ok)
	{
		auto serverUrl = GlobalData.serverUrl().toStdWString();
		try
		{
			RestClient c(serverUrl);
			startProgress();

			const unsigned int* segmentation;
			size_t segmentationSize = data->getMesh()->getSegmentationData(segmentation);
			QByteArray qSegmentation(reinterpret_cast<const char*>(segmentation), sizeof(unsigned int) * segmentationSize);
			QString segmentationB64 = qSegmentation.toBase64();


			GetTaskResult(c.UploadSegmentation(data->currentCaveId, segmentationB64.toStdWString(), ui.txtName->text().toStdWString(), ui.cmbExpertise->currentData().toInt(), ui.cmbCertainty->currentData().toInt())).then(
				[=](TaskResult<void> result)
			{
				emit actionFinished(result.success);
			}
			);
		}
		catch (std::exception& e)
		{
			QMessageBox::critical(this, "Upload Segmentation", QString("There was an error uploading the segmentation: ") + e.what());
		}
	}
}

UploadSegmentationDialog::~UploadSegmentationDialog()
{

}

void UploadSegmentationDialog::startProgress()
{
	ui.progressBar->setVisible(true);
	ui.progressBar->setMaximum(0);
	ui.progressBar->setValue(0);
	ui.buttonBox->setEnabled(false);
}

void UploadSegmentationDialog::stopProgress(bool success)
{
	ui.progressBar->setMaximum(1);
	ui.progressBar->setValue(1);
	if (success)
		accept();
	else
	{
		QMessageBox::critical(this, "Upload Segmentation", "There was an error uploading the segmentation.");
		ui.buttonBox->setEnabled(true);
	}
}
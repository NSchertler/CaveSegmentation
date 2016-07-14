#pragma once

#include <QDialog>
#include <QAbstractButton>
#include "SharedData.h"
#include "ui_UploadSegmentation.h"

class UploadSegmentationDialog : public QDialog
{
	Q_OBJECT

public:
	UploadSegmentationDialog(std::shared_ptr<SharedData> data, QWidget *parent = 0);
	~UploadSegmentationDialog();

signals:
	void actionFinished(bool success);

protected slots:
	void dialogButtonClicked(QAbstractButton*);
	void startProgress();
	void stopProgress(bool success);
	
private:
	Ui::UploadSegmentation ui;
	std::shared_ptr<SharedData> data;
};
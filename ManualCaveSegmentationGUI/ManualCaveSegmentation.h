#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_ManualCaveSegmentation.h"
#include "GLCaveView.h"
#include "Common.h"

class ManualCaveSegmentation : public QMainWindow
{
	Q_OBJECT

public:
	ManualCaveSegmentation(QWidget *parent = 0);
	~ManualCaveSegmentation();

private slots:	
	void load_model();	
	void brushSize_changed(int);
	void brushType_changed(bool);
	void downloadCaves();
	void nearClipChanged();
	void saveSegmentation();
	void loadSegmentation();
	void uploadSegmentation();
	void updateServerURL();

private:

	Ui::ManualCaveSegmentationClass ui;
	GLCaveView* glView;
	
	std::shared_ptr<SharedData> data;
};

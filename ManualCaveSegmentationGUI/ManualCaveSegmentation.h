#ifndef KOMPLEX2_H
#define KOMPLEX2_H

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
	void loadExternalModel();
	void saveSegmentation();
	void loadSegmentation();
	void uploadSegmentation();

private:

	Ui::ManualCaveSegmentationClass ui;
	GLCaveView* glView;
	
	std::shared_ptr<SharedData> data;
};

#endif // KOMPLEX2_H

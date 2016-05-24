#ifndef CAVESEGMENTATIONGUI_H
#define CAVESEGMENTATIONGUI_H

#include <QtWidgets/QMainWindow>
#include <QDir>
#include "ui_cavesegmentationgui.h"
#include "ViewModel.h"
#include "DataPlot.h"

class CaveSegmentationGUI : public QMainWindow
{
	Q_OBJECT

public:
	CaveSegmentationGUI(QWidget *parent = 0);	
	~CaveSegmentationGUI();

private slots:
	void loadOff(bool);	
	void loadSkeleton(bool);
	void loadDistances(bool);
	void runSegmentation(bool);
	void loadSegmentation(bool);

	void lookThroughChanged(int state);

	void segmentationParametersChanged();

	void resetToDefaults();

	void saveSegmentation();

	void saveSegmentedMesh();

private:
	Ui::CaveSegmentationGUIClass ui;
	std::unique_ptr<DataPlot> plot;
	ViewModel vm;

	QString getFilepath(QString defaultFile, QString title, QString filter);
	QDir dataDirectory;
};

#endif // CAVESEGMENTATIONGUI_H

#ifndef CAVESEGMENTATIONGUI_H
#define CAVESEGMENTATIONGUI_H

#include <QtWidgets/QMainWindow>
#include <QDir>
#include "ui_cavesegmentationgui.h"
#include "CaveGLData.hpp"

class CaveSegmentationGUI : public QMainWindow
{
	Q_OBJECT

public:
	CaveSegmentationGUI(QWidget *parent = 0);	
	~CaveSegmentationGUI();

private slots:
	void loadOff(bool);
	void loadSkeleton(bool);

private:
	Ui::CaveSegmentationGUIClass ui;
	CaveGLData data;

	QDir dataDirectory;
};

#endif // CAVESEGMENTATIONGUI_H

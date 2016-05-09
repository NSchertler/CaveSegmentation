#ifndef CAVESEGMENTATIONGUI_H
#define CAVESEGMENTATIONGUI_H

#include <QtWidgets/QMainWindow>
#include "ui_cavesegmentationgui.h"
#include "CaveGLData.hpp"

class CaveSegmentationGUI : public QMainWindow
{
	Q_OBJECT

public:
	CaveSegmentationGUI(QWidget *parent = 0);
	void loadOff(bool);
	~CaveSegmentationGUI();

private:
	Ui::CaveSegmentationGUIClass ui;
	CaveGLData data;
};

#endif // CAVESEGMENTATIONGUI_H

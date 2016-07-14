#ifndef CAVESEGMENTATIONGUI_H
#define CAVESEGMENTATIONGUI_H

#include <QtWidgets/QMainWindow>
#include <QDir>
#include "ui_cavesegmentationgui.h"
#include "ViewModel.h"
#include "DataPlot.h"

#include "CaveDataGLView.h"

class Secondary;

struct AppOptions
{
	bool stereo;
	float aspectMultiplier;
	int screenNumber;
	QString dataDir;
	QString initialDirectory;
};

class CaveSegmentationGUI : public QMainWindow
{
	Q_OBJECT

public:
	CaveSegmentationGUI(const AppOptions& o, QWidget *parent = 0);		
	~CaveSegmentationGUI();

private slots:
	void meshChanged();

	void skeletonChanged();

	void distancesChanged();

	void loadOff(bool);
	void loadSkeleton(bool);
	void calculateSkeleton(bool);
	void skeletonComputationFinished();
	void abortSkeleton(bool);
	void saveSkeleton(bool);
	void loadDistances(bool);
	void calculateDistances(bool);
	void calculateSpecificVertexDistances(bool);
	void saveDistances(bool);
	void runSegmentation(bool);
	void loadSegmentation(bool);

	void lookThroughChanged(int state);

	void segmentationParametersChanged();

	void resetToDefaults();

	void saveSegmentation();

	void saveSegmentedMesh();

	void glViewInited();

	void preloadData();
	
private:
	QTimer secondarySynchronizationTimer;
	std::unique_ptr<Secondary> secondary;
	HDC mainDC, secDC;
	std::unique_ptr<CaveDataGLView> secondaryGlView;	
	void synchronizeSecondary();

	CaveDataGLView* glView;

	Ui::CaveSegmentationGUIClass ui;
	std::unique_ptr<DataPlot> plot;
	ViewModel vm;

	QString getFilepath(QString defaultFile, QString title, QString filter);
	QDir dataDirectory;

	AppOptions options;

	std::string modelFilename;

	AbortHandle skeletonAbort;
	QFutureWatcher<CurveSkeleton*> skeletonWatcher;
	QFuture<CurveSkeleton*> skeletonComputation;
};

class Secondary : public QMainWindow
{
public:
	Secondary(QWidget* copyWidget, QWidget* parent) : QMainWindow(parent), copyWidget(copyWidget)
	{
		setAttribute(Qt::WA_PaintOnScreen);

		setCursor(QCursor(Qt::BlankCursor));
	}

	void CopyFromPrimary()
	{
		update(copyWidget->geometry());
	}

protected:

	void paintEvent(QPaintEvent*)
	{		
		QPixmap copy = copyWidget->grab();
		QPainter painter(this);
		painter.drawPixmap(copyWidget->geometry(), copy);
	}	

	QWidget* copyWidget;
};

#endif // CAVESEGMENTATIONGUI_H

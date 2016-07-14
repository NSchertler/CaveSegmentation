#pragma once

#include <QDialog>
#include "ui_CaveList.h"
#include "CaveData.h"
#include <memory>
#include <concrt.h>

class CaveListDialog : public QDialog
{
	Q_OBJECT

public:
	CaveListDialog(QWidget *parent = 0);
	~CaveListDialog();

signals:
	void newList();
	void newListItem(QString);
	void newCave(CaveMetadata*);
	void caveArrived(QListWidgetItem*);
	void actionStarted();
	void actionFinished();

private slots:
	void clearList();
	void insertListItem(QString);
	void insertCave(CaveMetadata*);
	void markCave(QListWidgetItem*);
	void startProgress();
	void stopProgress();
	void downloadSelectedCaves();
	
private:

	void downloadCaveList();
	std::shared_ptr<std::vector<CaveMetadata>> caves;

	Ui::UICaveListDialog ui;
	int elementsInQueue;
	Concurrency::critical_section sec;
};
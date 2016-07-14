#pragma once

#include <QDialog>
#include "ui_AvailableCaveList.h"
#include "CaveData.h"
#include <memory>
#include <concrt.h>
#include "SharedData.h"
#include <QAbstractButton>

class AvailableCaveListDialog : public QDialog
{
	Q_OBJECT

public:
	AvailableCaveListDialog(std::shared_ptr<SharedData> data, QWidget *parent = 0);
	~AvailableCaveListDialog();

	protected slots:
	void dialogButtonClicked(QAbstractButton*);
	
private:
	Ui::AvailableCaveList ui;
	std::shared_ptr<SharedData> data;
};
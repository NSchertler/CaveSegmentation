#include "CaveListDialog.h"
#include "RestClient.h"
#include "Common.h"
#include <QPushButton>

CaveListDialog::CaveListDialog(QWidget *parent)
	: QDialog(parent), elementsInQueue(0)
{
	ui.setupUi(this);

	ui.lstCaves->addItem("Loading caves from server...");

	auto btnDownload = new QPushButton("Download");
	ui.buttonBox->addButton(btnDownload, QDialogButtonBox::ActionRole);

	connect(this, SIGNAL(newList()), this, SLOT(clearList()));
	connect(this, SIGNAL(newListItem(QString)), this, SLOT(insertListItem(QString)));
	connect(this, SIGNAL(newCave(CaveMetadata*)), this, SLOT(insertCave(CaveMetadata*)));
	connect(this, SIGNAL(actionFinished()), this, SLOT(stopProgress()));
	connect(this, &CaveListDialog::actionStarted, this, &CaveListDialog::startProgress);
	connect(this, &CaveListDialog::caveArrived, this, &CaveListDialog::markCave);
	connect(btnDownload, &QPushButton::clicked, this, &CaveListDialog::downloadSelectedCaves);

	downloadCaveList();
}

CaveListDialog::~CaveListDialog()
{

}

void CaveListDialog::downloadCaveList()
{
	auto serverUrl = GlobalData.serverUrl().toStdWString();
	RestClient c(serverUrl);
	GetTaskResult(c.GetCaves()).then([=](TaskResult<std::shared_ptr<std::vector<CaveMetadata>>> v)
	{
		emit newList();
		if (v.success)
		{
			caves = v.result;
			for (auto& cave : *caves)
				emit newCave(&cave);
		}
		else
			emit newListItem("Error retrieving caves from server.");

		emit actionFinished();
	});
}

void CaveListDialog::downloadSelectedCaves()
{
	if (elementsInQueue > 0)
		return;
	
	for (int i = 0; i < ui.lstCaves->count(); ++i)
	{
		auto item = ui.lstCaves->item(i);
		if (item->checkState() == Qt::Checked)
		{
			{
				pplx::critical_section::scoped_lock lock(sec);
				if (elementsInQueue == 0)
					emit actionStarted();
				++elementsInQueue;
			}

			auto cave = static_cast<CaveMetadata*>(item->data(Qt::UserRole).value<void*>());
			std::wcout << cave->Name << std::endl;

			auto serverUrl = GlobalData.serverUrl().toStdWString();
			RestClient c(serverUrl);
			GlobalData.dataDirectory.mkdir(QString::number(cave->Id));
			QDir d(GlobalData.dataDirectory);
			d.cd(QString::number(cave->Id));
			GetTaskResult(c.DownloadCave(cave->Id, d.filePath("caveData.bin"))).then([this, item, cave, d](TaskResult<void> r){
				if (r.success)
				{
					QSettings metadata(d.filePath("metadata.conf"), QSettings::IniFormat);
					metadata.setValue("cave/name", QString::fromStdWString(cave->Name));

					emit caveArrived(item);
				}

				{
					pplx::critical_section::scoped_lock lock(sec);
					--elementsInQueue;
					if (elementsInQueue == 0)
						emit actionFinished();
				}
			});			
		}
	}
}

void CaveListDialog::clearList()
{
	ui.lstCaves->clear();
}

void CaveListDialog::insertListItem(QString s)
{
	QListWidgetItem* item = new QListWidgetItem(s, ui.lstCaves);	
}

void CaveListDialog::insertCave(CaveMetadata* cave)
{	
	QListWidgetItem* item = new QListWidgetItem(QString::fromWCharArray(cave->Name.c_str()), ui.lstCaves);
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	item->setCheckState(Qt::Checked);
	item->setData(Qt::UserRole, QVariant::fromValue((void*)cave));
	if(GlobalData.dataDirectory.exists(QString::number(cave->Id)))
		item->setTextColor(QColor::fromRgb(0, 175, 0));
}

void CaveListDialog::startProgress()
{
	ui.progressBar->setMaximum(0);
	ui.progressBar->setValue(0);
}

void CaveListDialog::stopProgress()
{
	ui.progressBar->setMaximum(1);
	ui.progressBar->setValue(1);
}

void CaveListDialog::markCave(QListWidgetItem* item)
{
	item->setTextColor(QColor::fromRgb(0, 175, 0));
}
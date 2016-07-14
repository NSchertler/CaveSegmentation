#include "AvailableCaveListDialog.h"
#include "Common.h"
#include <QDir>
#include <QDirIterator>
#include "GLCaveView.h"

AvailableCaveListDialog::AvailableCaveListDialog(std::shared_ptr<SharedData> data, QWidget* parent)
	: QDialog(parent), data(data)
{
	ui.setupUi(this);

	connect(ui.buttonBox, &QDialogButtonBox::clicked, this, &AvailableCaveListDialog::dialogButtonClicked);

	QDir d = GlobalData.dataDirectory;
	QDirIterator directories(d.absolutePath(), QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);

	 while (directories.hasNext())
	 {
		 try{
			 directories.next();
			 
			 bool isId = false;
			 int id = directories.fileName().toInt(&isId);

			 if (isId)
			 {
				 QSettings metadata(QDir(directories.filePath()).filePath("metadata.conf"), QSettings::IniFormat);

				 QListWidgetItem* item = new QListWidgetItem(metadata.value("cave/name").toString(), ui.lstCaves);

				 item->setData(Qt::UserRole, id);
			 }
		 }
		 catch (...)
		 { }
	}
}

void AvailableCaveListDialog::dialogButtonClicked(QAbstractButton* button)
{
	if (ui.lstCaves->selectedItems().size() == 1)
	{
		QDir dir = GlobalData.dataDirectory;
		int id = ui.lstCaves->selectedItems().at(0)->data(Qt::UserRole).toInt();
		dir.cd(QString::number(id));
		QDialogButtonBox::StandardButton stdButton = ui.buttonBox->standardButton(button);
		if (stdButton== QDialogButtonBox::StandardButton::Open)
		{
			data->setMesh(std::make_shared<Mesh>(data->view, dir.filePath("caveData.bin").toStdString()));
			data->currentCaveId = id;
		}
	}
}

AvailableCaveListDialog::~AvailableCaveListDialog()
{

}
#pragma once

#include <QDir>
#include <QSettings>
#include <memory>

struct _global
{


	_global()
	{
		QDir::home().mkdir("CaveSegmentation");
		dataDirectory = QDir::home();
		dataDirectory.cd("CaveSegmentation");;

		settings = std::make_shared<QSettings>(dataDirectory.filePath("settings.conf"), QSettings::IniFormat);		
	}

	QString serverUrl() const { return settings->value("server/url").toString(); }

	void setServerUrl(const QString& url) { settings->setValue("server/url", url); }

	QDir dataDirectory;

private:
	std::shared_ptr<QSettings> settings;

};

extern _global GlobalData;
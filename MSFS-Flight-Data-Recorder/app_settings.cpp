#include "app_settings.h"

#include <QCoreApplication>
#include <QDir>
#include <QSettings>

namespace {

QSettings makeSettings() {
	QString path = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("settings.ini"));
	return QSettings(path, QSettings::IniFormat);
}

}

AppSettings& AppSettings::instance() {
	static AppSettings settings;
	return settings;
}

QStringList AppSettings::dataTableHiddenFields() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("data_table/hidden_fields")).toStringList();
}

void AppSettings::setDataTableHiddenFields(const QStringList& fields) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("data_table/hidden_fields"), fields);
}

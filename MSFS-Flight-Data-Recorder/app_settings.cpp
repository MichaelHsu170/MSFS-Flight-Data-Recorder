#include "app_settings.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace {

QSettings makeSettings() {
	// Debug: read/write settings from cwd so each project checkout is self-contained.
	// Release: keep settings next to the exe so they follow the installation.
#ifdef _DEBUG
	QString dir = QDir::currentPath();
#else
	QString dir = QCoreApplication::applicationDirPath();
#endif
	return QSettings(QDir(dir).filePath(QStringLiteral("settings.ini")), QSettings::IniFormat);
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

QByteArray AppSettings::topSplitterState() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/top_splitter_state")).toByteArray();
}

void AppSettings::setTopSplitterState(const QByteArray& state) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("layout/top_splitter_state"), state);
}

QByteArray AppSettings::trajectoryDataTableSplitterState() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/trajectory_data_table_splitter_state")).toByteArray();
}

void AppSettings::setTrajectoryDataTableSplitterState(const QByteArray& state) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("layout/trajectory_data_table_splitter_state"), state);
}

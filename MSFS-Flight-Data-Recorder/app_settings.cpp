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

int AppSettings::rightPanelWidth() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/right_panel_width"), 260).toInt();
}

void AppSettings::setRightPanelWidth(int w) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("layout/right_panel_width"), w);
}

int AppSettings::chartsPanelHeight() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/charts_panel_height"), 400).toInt();
}

void AppSettings::setChartsPanelHeight(int h) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("layout/charts_panel_height"), h);
}

QString AppSettings::geminiApiKey() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("ai/gemini_api_key")).toString();
}

void AppSettings::setGeminiApiKey(const QString& key) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("ai/gemini_api_key"), key);
}

int AppSettings::sampleIntervalMs() const {
	QSettings settings = makeSettings();
	bool ok = false;
	int v = settings.value(QStringLiteral("recording/sample_interval_ms")).toInt(&ok);
	return (ok && v > 0) ? v : 500;
}

QString AppSettings::verboseLevel() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("logging/verbose"), QStringLiteral("INFO")).toString();
}

void AppSettings::setVerboseLevel(const QString& level) {
	QSettings settings = makeSettings();
	settings.setValue(QStringLiteral("logging/verbose"), level);
}

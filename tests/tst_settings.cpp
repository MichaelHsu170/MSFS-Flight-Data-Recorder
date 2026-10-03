// settings.ini (app_settings.cpp): default file, defaults for missing or bad
// values, and in-place edits that keep comments and other sections intact.
#include "test_support.h"

#include "app_settings.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QtTest>

using namespace TestSupport;

namespace {

QString readSettingsFile() {
	QFile f(AppSettings::filePath());
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return QString();
	return QString::fromUtf8(f.readAll());
}

void writeSettingsFile(const QString& text) {
	QFile f(AppSettings::filePath());
	QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate));
	f.write(text.toUtf8());
}

}

class TstSettings : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() { isolateFiles(); }

	// Must run first: the default file is only created the first time
	// AppSettings::instance() is used in a process.
	void firstUseCreatesTheDefaultFile() {
		QVERIFY(!QFile::exists(AppSettings::filePath()));
		AppSettings::instance();
		const QString text = readSettingsFile();
		for (const char* line : { "[ai]", "gemini_api_key=", "[recording]", "sample_interval_ms=500", "enabled=true",
				"[logging]", "verbose=INFO", "[layout]", "right_panel_width=260", "charts_panel_height=400",
				"[data_table]", "hidden_fields=", "[table_column_width]", "data_table_field_column_width=140",
				"trip_history_column_widths=" })
			QVERIFY2(text.contains(QLatin1String(line) + QLatin1Char('\n')), line);
	}

	void defaultFileReadsAsDefaults() {
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.dataTableHiddenFields(), QStringList());
		QCOMPARE(s.dataTableFieldColumnWidth(), 140);
		QCOMPARE(s.rightPanelWidth(), 260);
		QCOMPARE(s.chartsPanelHeight(), 400);
		QVERIFY(s.tripHistoryColumnWidths().isEmpty());
		QCOMPARE(s.geminiApiKey(), QString());
		QCOMPARE(s.sampleIntervalMs(), 500);
		QCOMPARE(s.recordingEnabled(), true);
	}

	void missingFileReadsAsDefaults() {
		removeSettings();
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.dataTableFieldColumnWidth(), 140);
		QCOMPARE(s.rightPanelWidth(), 260);
		QCOMPARE(s.chartsPanelHeight(), 400);
		QCOMPARE(s.sampleIntervalMs(), 500);
		QCOMPARE(s.recordingEnabled(), true);
	}

	void invalidNumbersFallBackToDefaults() {
		writeSettingsFile("[recording]\nsample_interval_ms=abc\n[layout]\nright_panel_width=-5\ncharts_panel_height=0\n"
			"[table_column_width]\ndata_table_field_column_width=wide\n");
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.sampleIntervalMs(), 500);
		QCOMPARE(s.rightPanelWidth(), 260);
		QCOMPARE(s.chartsPanelHeight(), 400);
		QCOMPARE(s.dataTableFieldColumnWidth(), 140);
	}

	void valuesAreReadFromTheFile() {
		writeSettingsFile("[ai]\ngemini_api_key=KEY123\n[recording]\nsample_interval_ms=250\nenabled=false\n");
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.geminiApiKey(), QStringLiteral("KEY123"));
		QCOMPARE(s.sampleIntervalMs(), 250);
		QCOMPARE(s.recordingEnabled(), false);
	}

	void setterReplacesTheValueInPlace() {
		writeSettingsFile("; top comment\n[layout]\n; width comment\nright_panel_width=260\ncharts_panel_height=400\n\n[other]\nkeep=me\n");
		AppSettings::instance().setRightPanelWidth(300);
		QCOMPARE(readSettingsFile(),
			QStringLiteral("; top comment\n[layout]\n; width comment\nright_panel_width=300\ncharts_panel_height=400\n\n[other]\nkeep=me\n"));
		QCOMPARE(AppSettings::instance().rightPanelWidth(), 300);
	}

	void setterAddsAMissingKeyToItsSection() {
		writeSettingsFile("[layout]\nright_panel_width=260\n\n[other]\nkeep=me\n");
		AppSettings::instance().setChartsPanelHeight(500);
		const QString text = readSettingsFile();
		QVERIFY(text.startsWith("[layout]\nright_panel_width=260\n"));
		QVERIFY(text.indexOf("charts_panel_height=500") < text.indexOf("[other]"));
		QVERIFY(text.contains("; Height in pixels of the Charts panel"));
		QVERIFY(text.endsWith("[other]\nkeep=me\n"));
		QCOMPARE(AppSettings::instance().chartsPanelHeight(), 500);
	}

	void setterAddsAMissingSection() {
		removeSettings();
		AppSettings::instance().setDataTableFieldColumnWidth(180);
		const QString text = readSettingsFile();
		QVERIFY(text.contains("[table_column_width]\n"));
		QVERIFY(text.contains("data_table_field_column_width=180\n"));
		QCOMPARE(AppSettings::instance().dataTableFieldColumnWidth(), 180);
	}

	void hiddenFieldsRoundTrip() {
		AppSettings& s = AppSettings::instance();
		s.setDataTableHiddenFields({ "G Force", "Plane Altitude" });
		QCOMPARE(s.dataTableHiddenFields(), (QStringList{ "G Force", "Plane Altitude" }));
		s.setDataTableHiddenFields({ "Only One" });
		QCOMPARE(s.dataTableHiddenFields(), (QStringList{ "Only One" }));
		s.setDataTableHiddenFields({});
		QCOMPARE(s.dataTableHiddenFields(), QStringList());
	}

	void columnWidthsRoundTripAndIgnoreJunk() {
		AppSettings& s = AppSettings::instance();
		QMap<QString, int> widths{ { "TitleColumn", 120 }, { "GroupColumn", 64 } };
		s.setTripHistoryColumnWidths(widths);
		QCOMPARE(s.tripHistoryColumnWidths(), widths);
		s.setTripHistoryColumnWidths({ { "OnlyColumn", 90 } });
		QCOMPARE(s.tripHistoryColumnWidths(), (QMap<QString, int>{ { "OnlyColumn", 90 } }));
		writeSettingsFile("[table_column_width]\ntrip_history_column_widths=120,TitleColumn=abc,GroupColumn=70\n");
		QCOMPARE(s.tripHistoryColumnWidths(), (QMap<QString, int>{ { "GroupColumn", 70 } }));
	}

	void sectionHeaderWithATrailingCommentIsUpdatedInPlace() {
		writeSettingsFile("[layout] ; edited by hand\nright_panel_width=260\n[other] # too\nkeep=me\n");
		AppSettings::instance().setRightPanelWidth(300);
		QCOMPARE(readSettingsFile(), QStringLiteral("[layout] ; edited by hand\nright_panel_width=300\n[other] # too\nkeep=me\n"));
	}

	void unreadableFileIsNotOverwritten() {
		removeSettings();
		// A directory exists at the path but can't be opened as a file.
		QVERIFY(QDir().mkpath(AppSettings::filePath()));
		AppSettings::instance().setRightPanelWidth(300);
		QVERIFY(QFileInfo(AppSettings::filePath()).isDir());
		QVERIFY(QDir().rmdir(AppSettings::filePath()));
	}

	void readOnlyFileIsLeftUnchanged() {
		writeSettingsFile("[layout]\nright_panel_width=260\n");
		QFile::setPermissions(AppSettings::filePath(), QFile::ReadOwner | QFile::ReadUser);
		AppSettings::instance().setRightPanelWidth(300);
		const QString text = readSettingsFile();
		QFile::setPermissions(AppSettings::filePath(), QFile::ReadOwner | QFile::WriteOwner | QFile::ReadUser | QFile::WriteUser);
		QCOMPARE(text, QStringLiteral("[layout]\nright_panel_width=260\n"));
		QCOMPARE(AppSettings::instance().rightPanelWidth(), 260);
	}

	void logLevelIsReadVerbatimWithInfoAsDefault() {
		writeSettingsFile("[logging]\nverbose=trace\n");
		QCOMPARE(AppSettings::logLevel(), QStringLiteral("trace"));
		removeSettings();
		QCOMPARE(AppSettings::logLevel(), QStringLiteral("INFO"));
	}

	void recordingEnabledRoundTrip() {
		AppSettings& s = AppSettings::instance();
		s.setRecordingEnabled(false);
		QCOMPARE(s.recordingEnabled(), false);
		s.setRecordingEnabled(true);
		QCOMPARE(s.recordingEnabled(), true);
	}
};

QTEST_MAIN(TstSettings)
#include "tst_settings.moc"

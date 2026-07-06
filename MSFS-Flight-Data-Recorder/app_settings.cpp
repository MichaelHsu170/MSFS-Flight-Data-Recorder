#include "app_settings.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSettings>
#include <QTextStream>

namespace {

QString settingsFilePath() {
	// Debug: cwd so each project checkout is self-contained.
	// Release: exe directory so settings follow the installation.
#ifdef _DEBUG
	return QDir(QDir::currentPath()).filePath(QStringLiteral("settings.ini"));
#else
	return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("settings.ini"));
#endif
}

QSettings makeSettings() {
	return QSettings(settingsFilePath(), QSettings::IniFormat);
}

// Writes a single key=value in the named INI section, touching only that one
// line. Every other line — comments, blank lines, other keys, other sections —
// is preserved exactly.
//
// sectionComment and keyComment are plain text (no leading "; "). They are
// written only when new content is appended to the file:
//   - sectionComment is written before the [section] header when the section
//     itself is absent from the file.
//   - keyComment is written before the key=value line when the key is absent
//     (whether or not the section already existed).
// This means the file stays self-documenting even when keys are added by a
// newer version of the app to an older settings.ini.
void writeIniValue(const QString& section, const QString& key, const QString& value,
                   const QString& sectionComment = {},
                   const QString& keyComment = {}) {
	const QString path = settingsFilePath();
	QFile file(path);
	QStringList lines;
	if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QTextStream in(&file);
		in.setEncoding(QStringConverter::Utf8);
		while (!in.atEnd())
			lines.append(in.readLine());
		file.close();
	}

	const QString sectionHeader = '[' + section + ']';
	bool inSection = false;
	int sectionLastLine = -1; // last line index belonging to the target section
	int keyLine = -1;         // line index of the existing key=… entry, or -1

	for (int i = 0; i < lines.size(); ++i) {
		const QString t = lines[i].trimmed();
		if (t.startsWith('[')) {
			if (inSection) break;             // just left the target section
			inSection = (t == sectionHeader);
			if (inSection) sectionLastLine = i;
		} else if (inSection) {
			sectionLastLine = i;
			if (keyLine < 0 && !t.startsWith(';') && !t.startsWith('#')
					&& t.section('=', 0, 0).trimmed() == key)
				keyLine = i;
		}
	}

	auto commentLines = [](const QString& comment) {
		QStringList out;
		if (!comment.isEmpty())
			for (const QString& line : comment.split('\n'))
				out.append("; " + line);
		return out;
	};

	const QString entry = key + '=' + value;

	if (keyLine >= 0) {
		// Key already present — patch value only, leave comment untouched.
		lines[keyLine] = entry;
	} else if (sectionLastLine >= 0) {
		// Section exists but key is missing — insert key (with comment) after
		// the last line of the section, preserving everything that follows.
		QStringList toInsert = commentLines(keyComment);
		toInsert.append(entry);
		for (int j = toInsert.size() - 1; j >= 0; --j)
			lines.insert(sectionLastLine + 1, toInsert[j]);
	} else {
		// Section not present — append section header and key at end of file.
		if (!lines.isEmpty() && !lines.last().trimmed().isEmpty())
			lines.append(QString());
		lines << commentLines(sectionComment);
		lines.append(sectionHeader);
		lines << commentLines(keyComment);
		lines.append(entry);
	}

	if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
		QTextStream out(&file);
		out.setEncoding(QStringConverter::Utf8);
		for (const QString& line : lines)
			out << line << '\n';
	}
}

// Creates a fully-documented settings.ini on first launch. Only runs when the
// file does not yet exist — never modifies an existing file, even a partial one.
void ensureSettingsFileExists() {
	const QString path = settingsFilePath();
	if (QFile::exists(path))
		return;

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QTextStream out(&file);
	out.setEncoding(QStringConverter::Utf8);
	out <<
		"; MSFS Flight Data Recorder — settings\n"
		"; Edit while the app is not running. All values are human-readable.\n"
		"\n"
		"[ai]\n"
		"; Gemini API key for the AI touchdown analysis feature.\n"
		"; Obtain a free key from Google AI Studio (aistudio.google.com), then paste it\n"
		"; here and restart the app. The app never writes this value.\n"
		"; Without a key the Analyze Landing button is disabled.\n"
		"gemini_api_key=\n"
		"\n"
		"[recording]\n"
		"; Maximum time between telemetry samples written to trip_data, in milliseconds.\n"
		"; Lower values produce finer trajectory and chart resolution at the cost of\n"
		"; a larger database and slower trip load times. Must be a positive integer.\n"
		"; Default: 500  (0.5 s — adequate for all aircraft types including fast jets\n"
		"; at subsonic speeds; go lower only for supersonic recording needs).\n"
		"sample_interval_ms=500\n"
		"\n"
		"[logging]\n"
		"; Maximum log level written to msfs_fdr_debug.log.\n"
		"; Levels (inclusive — each includes all levels above it):\n"
		";   FATAL    — unrecoverable errors only\n"
		";   WARNING  — unexpected conditions that don't abort the app\n"
		";   INFO     — operational events (connect, recording start/stop, takeoff, touchdown)\n"
		";   PROFILE  — performance timing for all subsystems (high-volume; for profiling only)\n"
		"; Default: INFO\n"
		"verbose=INFO\n"
		"\n"
		"[layout]\n"
		"; Width in pixels of the Live Status panel (top-right) and Data Table panel\n"
		"; (bottom-right). Both columns share one value so they stay aligned when\n"
		"; either splitter is dragged. Default: 260.\n"
		"right_panel_width=260\n"
		"\n"
		"; Height in pixels of the Charts panel (below the map). The map takes the\n"
		"; remaining vertical space. Default: 400.\n"
		"charts_panel_height=400\n"
		"\n"
		"[data_table]\n"
		"; Comma-separated list of field labels hidden in the Data Table panel via the\n"
		"; Fields dialog. Absent or empty means all fields are visible.\n"
		"hidden_fields=\n";
}

}

AppSettings& AppSettings::instance() {
	static bool _ = (ensureSettingsFileExists(), true);
	static AppSettings settings;
	(void)_;
	return settings;
}

QStringList AppSettings::dataTableHiddenFields() const {
	QSettings settings = makeSettings();
	const QVariant raw = settings.value(QStringLiteral("data_table/hidden_fields"));
	// QSettings may have written a native QStringList (legacy); our custom writer
	// stores a plain comma-separated string — handle both.
	if (raw.typeId() == QMetaType::QStringList)
		return raw.toStringList();
	const QString s = raw.toString();
	return s.isEmpty() ? QStringList{} : s.split(',', Qt::SkipEmptyParts);
}

void AppSettings::setDataTableHiddenFields(const QStringList& fields) {
	writeIniValue(
		QStringLiteral("data_table"),
		QStringLiteral("hidden_fields"),
		fields.join(','),
		QStringLiteral("Auto-managed by the app."),
		QStringLiteral("Comma-separated list of field labels hidden in the Data Table panel via the\n"
		               "Fields dialog. Absent or empty means all fields are visible.")
	);
}

int AppSettings::rightPanelWidth() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/right_panel_width"), 260).toInt();
}

void AppSettings::setRightPanelWidth(int w) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("right_panel_width"),
		QString::number(w),
		QStringLiteral("Auto-managed by the app."),
		QStringLiteral("Width in pixels of the Live Status panel (top-right) and Data Table panel\n"
		               "(bottom-right). Both columns share one value so they stay aligned when\n"
		               "either splitter is dragged. Default: 260.")
	);
}

int AppSettings::chartsPanelHeight() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("layout/charts_panel_height"), 400).toInt();
}

void AppSettings::setChartsPanelHeight(int h) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("charts_panel_height"),
		QString::number(h),
		{},
		QStringLiteral("Height in pixels of the Charts panel (below the map). The map takes the\n"
		               "remaining vertical space. Default: 400.")
	);
}

QString AppSettings::geminiApiKey() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("ai/gemini_api_key")).toString();
}

void AppSettings::setGeminiApiKey(const QString& key) {
	writeIniValue(
		QStringLiteral("ai"),
		QStringLiteral("gemini_api_key"),
		key,
		QStringLiteral("AI touchdown analysis settings."),
		QStringLiteral("Gemini API key for the AI touchdown analysis feature.\n"
		               "Obtain a free key from Google AI Studio (aistudio.google.com), then paste it\n"
		               "here and restart the app. The app never writes this value.\n"
		               "Without a key the Analyze Landing button is disabled.")
	);
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
	writeIniValue(
		QStringLiteral("logging"),
		QStringLiteral("verbose"),
		level,
		QStringLiteral("Logging settings."),
		QStringLiteral("Maximum log level written to msfs_fdr_debug.log.\n"
		               "Levels (inclusive — each includes all levels above it):\n"
		               "  FATAL    — unrecoverable errors only\n"
		               "  WARNING  — unexpected conditions that don't abort the app\n"
		               "  INFO     — operational events (connect, recording start/stop, takeoff, touchdown)\n"
		               "  PROFILE  — performance timing for all subsystems (high-volume; for profiling only)\n"
		               "Default: INFO")
	);
}

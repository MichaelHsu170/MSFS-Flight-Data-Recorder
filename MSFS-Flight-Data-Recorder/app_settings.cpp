#include "app_settings.h"
#include "logger.h"

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
	if (file.exists()) {
		// The file exists but couldn't be read (e.g. locked, permissions) --
		// bail out rather than falling through to the write below, which
		// would truncate it and replace its entire contents with just this
		// one key, discarding every other saved setting.
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
			Logger::logf(Logger::Warning, "Settings",
				"Failed to read %s (section [%s], key %s) - change was not saved",
				qUtf8Printable(path), qUtf8Printable(section), qUtf8Printable(key));
			return;
		}
		QTextStream in(&file);
		in.setEncoding(QStringConverter::Utf8);
		while (!in.atEnd())
			lines.append(in.readLine());
		file.close();
	}

	const QString sectionHeader = '[' + section + ']';
	bool inSection = false;
	int sectionHeaderLine = -1; // index of this section's own "[section]" line, or -1
	int sectionLastLine = -1;   // last line index belonging to the target section
	int keyLine = -1;           // line index of the existing key=… entry, or -1

	for (int i = 0; i < lines.size(); ++i) {
		const QString t = lines[i].trimmed();
		if (t.startsWith('[')) {
			if (inSection) break;             // just left the target section
			inSection = (t == sectionHeader);
			if (inSection) sectionHeaderLine = sectionLastLine = i;
		} else if (inSection) {
			sectionLastLine = i;
			if (keyLine < 0 && !t.startsWith(';') && !t.startsWith('#')
					&& t.section('=', 0, 0).trimmed() == key)
				keyLine = i;
		}
	}

	// A trailing blank line + comment here aren't necessarily this section's
	// own trailing content -- they're also exactly what the "section not
	// present" branch below writes as the auto-generated preamble (blank
	// separator + sectionComment) of a *later* section, written before its
	// own header ever appeared in the file. Trim them back off the end of
	// this section so a later insertion into *this* section can't land
	// inside that preamble and separate it from the header it belongs to.
	while (sectionLastLine > sectionHeaderLine) {
		const QString t = lines[sectionLastLine].trimmed();
		if (t.isEmpty() || t.startsWith(';') || t.startsWith('#'))
			--sectionLastLine;
		else
			break;
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
		// the last line of the section, preserving everything that follows. A
		// blank line separates it from the prior key, matching the grouping
		// convention used between distinct settings elsewhere in this file
		// (see ensureSettingsFileExists()) -- skipped if there's no comment to
		// separate (nothing to visually group) or the prior line is already blank.
		QStringList toInsert;
		if (!keyComment.isEmpty() && !lines[sectionLastLine].trimmed().isEmpty())
			toInsert.append(QString());
		toInsert += commentLines(keyComment);
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
	} else {
		Logger::logf(Logger::Warning, "Settings",
			"Failed to write %s (section [%s], key %s) - change was not saved",
			qUtf8Printable(path), qUtf8Printable(section), qUtf8Printable(key));
	}
}

// Creates a fully-documented settings.ini on first launch. Only runs when the
// file does not yet exist — never modifies an existing file, even a partial one.
void ensureSettingsFileExists() {
	const QString path = settingsFilePath();
	if (QFile::exists(path)) {
		Logger::logf(Logger::Trace, "Settings", "Using existing settings.ini at %s", qUtf8Printable(path));
		return;
	}
	Logger::logf(Logger::Trace, "Settings", "No settings.ini found; creating default at %s", qUtf8Printable(path));

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		Logger::logf(Logger::Warning, "Settings",
			"Failed to create default settings.ini at %s", qUtf8Printable(path));
		return;
	}

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
		"; Comma-separated list of SimConnect event names to suppress from trip_events.\n"
		"; Not all aircraft emit events in the same pattern. Some third-party or WASM-based\n"
		"; aircraft fire certain events at extremely high frequency as internal signals that\n"
		"; carry no meaningful flight data — logging them bloats trip_events and makes\n"
		"; recordings hard to read. If you notice an event appearing excessively in a\n"
		"; recording, add its name here and restart the app to silence it.\n"
		";\n"
		"; Each entry can be written either as the value stored in the database\n"
		"; (e.g. APU_STARTER) or with the EVENT_ prefix (e.g. EVENT_APU_STARTER).\n"
		"; Names are case-insensitive.\n"
		"skip_events=APU_STARTER,BRAKES,AP_VS_ON,AUTOPILOT_OFF,APU_OFF_SWITCH\n"
		"\n"
		"[logging]\n"
		"; Maximum log level written to msfs_fdr_debug.log.\n"
		"; Levels (inclusive — each includes all levels above it):\n"
		";   FATAL    — unrecoverable errors only\n"
		";   WARNING  — unexpected conditions that don't abort the app\n"
		";   INFO     — operational events (connect, recording start/stop, takeoff, touchdown)\n"
		";   TRACE    — fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)\n"
		";   PROFILE  — performance timing for all subsystems (highest volume; for profiling only)\n"
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
		"hidden_fields=\n"
		"\n"
		"; Auto-managed by the app. Persisted column widths for the tables in the\n"
		"; UI that support user resizing.\n"
		"[table_column_width]\n"
		"; Width in pixels of the Field column in the Data Table panel. The Value\n"
		"; column always stretches to fill the rest. Default: 140.\n"
		"data_table_field_column_width=140\n"
		"\n"
		"; Column widths in pixels for the Trip History table, as comma-separated\n"
		"; key=value pairs keyed by TripHistoryModel::Column enum member name (e.g.\n"
		"; TitleColumn=120). Columns using Stretch sizing are never stored. Unknown\n"
		"; or missing keys fall back to that column's coded default.\n"
		"trip_history_column_widths=\n";

	Logger::logf(Logger::Trace, "Settings", "Default settings.ini created at %s", qUtf8Printable(path));
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
	Logger::logf(Logger::Trace, "Settings", "Data Table field visibility changed: %d field(s) now hidden", (int)fields.size());
	writeIniValue(
		QStringLiteral("data_table"),
		QStringLiteral("hidden_fields"),
		fields.join(','),
		QStringLiteral("Auto-managed by the app."),
		QStringLiteral("Comma-separated list of field labels hidden in the Data Table panel via the\n"
		               "Fields dialog. Absent or empty means all fields are visible.")
	);
}

int AppSettings::dataTableFieldColumnWidth() const {
	QSettings settings = makeSettings();
	bool ok = false;
	int v = settings.value(QStringLiteral("table_column_width/data_table_field_column_width"), 140).toInt(&ok);
	return (ok && v > 0) ? v : 140;
}

void AppSettings::setDataTableFieldColumnWidth(int w) {
	writeIniValue(
		QStringLiteral("table_column_width"),
		QStringLiteral("data_table_field_column_width"),
		QString::number(w),
		QStringLiteral("Auto-managed by the app. Persisted column widths for the tables in the\n"
		               "UI that support user resizing."),
		QStringLiteral("Width in pixels of the Field column in the Data Table panel. The Value\n"
		               "column always stretches to fill the rest. Default: 140.")
	);
}

int AppSettings::rightPanelWidth() const {
	QSettings settings = makeSettings();
	bool ok = false;
	int v = settings.value(QStringLiteral("layout/right_panel_width"), 260).toInt(&ok);
	return (ok && v > 0) ? v : 260;
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
	bool ok = false;
	int v = settings.value(QStringLiteral("layout/charts_panel_height"), 400).toInt(&ok);
	return (ok && v > 0) ? v : 400;
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

QMap<QString, int> AppSettings::tripHistoryColumnWidths() const {
	QSettings settings = makeSettings();
	const QVariant raw = settings.value(QStringLiteral("table_column_width/trip_history_column_widths"));
	// QSettings' ini reader auto-detects a comma-separated value as a list and
	// returns it typed as QStringList rather than QString -- QVariant::toString()
	// on a multi-element QStringList yields an empty string, silently discarding
	// every saved width. Same issue as dataTableHiddenFields()/skipEvents(); handle
	// both forms.
	QStringList parts;
	if (raw.typeId() == QMetaType::QStringList)
		parts = raw.toStringList();
	else if (!raw.toString().isEmpty())
		parts = raw.toString().split(',', Qt::SkipEmptyParts);
	QMap<QString, int> widths;
	for (const QString& part : parts) {
		const int eq = part.indexOf('=');
		if (eq < 0)
			continue; // e.g. leftover from the old positional format -- ignore
		bool ok = false;
		int w = part.mid(eq + 1).toInt(&ok);
		if (ok)
			widths.insert(part.left(eq), w);
	}
	return widths;
}

void AppSettings::setTripHistoryColumnWidths(const QMap<QString, int>& widths) {
	QStringList parts;
	for (auto it = widths.constBegin(); it != widths.constEnd(); ++it)
		parts.append(it.key() + '=' + QString::number(it.value()));
	writeIniValue(
		QStringLiteral("table_column_width"),
		QStringLiteral("trip_history_column_widths"),
		parts.join(','),
		QStringLiteral("Auto-managed by the app. Persisted column widths for the tables in the\n"
		               "UI that support user resizing."),
		QStringLiteral("Column widths in pixels for the Trip History table, as comma-separated\n"
		               "key=value pairs keyed by TripHistoryModel::Column enum member name (e.g.\n"
		               "TitleColumn=120). Columns using Stretch sizing are never stored. Unknown\n"
		               "or missing keys fall back to that column's coded default.")
	);
}

QString AppSettings::geminiApiKey() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("ai/gemini_api_key")).toString();
}

void AppSettings::setGeminiApiKey(const QString& key) {
	// Never log the key value itself, only whether one is now configured.
	Logger::logf(Logger::Trace, "Settings", "Gemini API key %s", key.isEmpty() ? "cleared" : "updated");
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
	if (ok && v > 0) {
		Logger::logf(Logger::Trace, "Settings", "sample_interval_ms=%d read from settings.ini", v);
		return v;
	}
	Logger::logf(Logger::Trace, "Settings", "sample_interval_ms missing or invalid in settings.ini; falling back to default 500");
	return 500;
}

QString AppSettings::verboseLevel() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("logging/verbose"), QStringLiteral("INFO")).toString();
}

QStringList AppSettings::skipEvents() const {
	QSettings settings = makeSettings();
	const QVariant raw = settings.value(QStringLiteral("recording/skip_events"));
	if (raw.typeId() == QMetaType::QStringList)
		return raw.toStringList();
	const QString s = raw.toString();
	return s.isEmpty() ? QStringList{} : s.split(',', Qt::SkipEmptyParts);
}

void AppSettings::setVerboseLevel(const QString& level) {
	Logger::logf(Logger::Trace, "Settings", "Log verbosity level set to %s", qUtf8Printable(level));
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

#include "app_settings.h"
#include "app_paths.h"
#include "logger.h"

#include <QFile>
#include <QSettings>
#include <QTextStream>

namespace {

QString settingsFilePath() {
	return QString::fromStdString(app_file_path("settings.ini"));
}

QSettings makeSettings() {
	return QSettings(settingsFilePath(), QSettings::IniFormat);
}

// key's value if it's a positive integer, otherwise fallback.
int positiveInt(const char* key, int fallback) {
	bool ok = false;
	const int v = makeSettings().value(QLatin1String(key)).toInt(&ok);
	return (ok && v > 0) ? v : fallback;
}

// key's comma-separated value as a list. QSettings' ini reader returns a
// value containing a comma as a QStringList rather than a QString (and
// QVariant::toString() on a multi-element list is empty), so take both forms.
QStringList commaList(const char* key) {
	const QVariant raw = makeSettings().value(QLatin1String(key));
	if (raw.typeId() == QMetaType::QStringList)
		return raw.toStringList();
	const QString s = raw.toString();
	return s.isEmpty() ? QStringList{} : s.split(',', Qt::SkipEmptyParts);
}

// comment (plain text, lines separated by '\n') as "; "-prefixed INI lines.
QStringList commentLines(const QString& comment) {
	QStringList out;
	if (!comment.isEmpty())
		for (const QString& line : comment.split('\n'))
			out.append("; " + line);
	return out;
}

// Defaults of keys a missing or invalid value falls back to.
constexpr int kDefaultSampleIntervalMs = 500;
constexpr int kDefaultChartsPanelHeight = 400;
constexpr int kDefaultFieldColumnWidth = 140;

// A section's header line, after the comment that goes above it. Sections the
// app writes itself say so; the others have no comment. Written both into the
// default settings.ini and when a setter adds the section to a file that
// lacks it, so the two always match.
QStringList sectionHeaderLines(const QString& section) {
	QString comment;
	if (section == QLatin1String("layout") || section == QLatin1String("data_table"))
		comment = QStringLiteral("Auto-managed by the app.");
	else if (section == QLatin1String("table_column_width"))
		comment = QStringLiteral("Auto-managed by the app. Persisted column widths for the tables in the\n"
		                         "UI that support user resizing.");
	return commentLines(comment) << '[' + section + ']';
}

// Key comments written both into the default settings.ini and when the
// setter adds the key to an existing file that lacks it.
QString recordingEnabledComment() {
	return QStringLiteral("Auto-managed by the app. Whether automatic recording is allowed to start,\n"
	                      "toggled via the Recording indicator in the Live Status panel. Disabling it\n"
	                      "only prevents a new trip from starting; it doesn't stop one already in\n"
	                      "progress. Default: true.");
}

QString chartsPanelHeightComment() {
	return QStringLiteral("Height in pixels of the Charts panel (below the map). The map takes the\n"
	                      "remaining vertical space. Default: %1.").arg(kDefaultChartsPanelHeight);
}

QString fieldColumnWidthComment() {
	return QStringLiteral("Width in pixels of the Field column in the Data Table panel. The Value\n"
	                      "column always stretches to fill the rest. Default: %1.").arg(kDefaultFieldColumnWidth);
}

QString tripHistoryColumnWidthsComment() {
	return QStringLiteral("Column widths in pixels for the Trip History table, as comma-separated\n"
	                      "key=value pairs keyed by TripHistoryModel::Column enum member name (e.g.\n"
	                      "TitleColumn=120). Columns using Stretch sizing are never stored. Unknown\n"
	                      "or missing keys fall back to that column's coded default.");
}

QString hiddenFieldsComment() {
	return QStringLiteral("Comma-separated list of field labels hidden in the Data Table panel via the\n"
	                      "Visible Fields dialog. Absent or empty means all fields are visible.");
}

QString rightPanelWidthComment() {
	return QStringLiteral("Width in pixels of the Live Status panel (top-right) and Data Table panel\n"
	                      "(bottom-right). Both columns share one value so they stay aligned when\n"
	                      "either splitter is dragged. Default: %1.").arg(kRightPanelWidth);
}

// Writes a single key=value in the named INI section, touching only that one
// line. Every other line — comments, blank lines, other keys, other sections —
// is preserved exactly.
//
// Comments are written only when new content is appended to the file:
//   - a section that is absent is added with sectionHeaderLines(), its
//     comment included.
//   - keyComment (plain text, no leading "; ") is written before the
//     key=value line when the key is absent (whether or not the section
//     already existed).
// This means the file stays self-documenting even when keys are added by a
// newer version of the app to an older settings.ini.
void writeIniValue(const QString& section, const QString& key, const QString& value,
                   const QString& keyComment) {
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
			// Ignore a trailing "; comment" or "# comment" on the header line
			// itself (e.g. from manual editing) when matching -- comparing the
			// raw line would fail to recognize an existing section and append
			// a duplicate one at end-of-file instead of updating it. The
			// comment text itself is never touched: only this comparison
			// strips it, the stored line is written back unchanged.
			QString header = t;
			int semi = header.indexOf(';');
			int hash = header.indexOf('#');
			int commentIdx = (semi < 0) ? hash : (hash < 0 ? semi : qMin(semi, hash));
			if (commentIdx >= 0)
				header = header.left(commentIdx).trimmed();
			inSection = (header == sectionHeader);
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
	// separator + section comment) of a *later* section, written before its
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
		lines << sectionHeaderLines(section);
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

	const auto header = [](const char* section) { return sectionHeaderLines(QLatin1String(section)).join('\n'); };
	QTextStream out(&file);
	out.setEncoding(QStringConverter::Utf8);
	out <<
		"; MSFS Flight Data Recorder — settings\n"
		"; Edit while the app is not running. All values are human-readable.\n"
		"\n"
		<< header("ai") << "\n"
		"; Gemini API key for the AI liftoff/landing analysis feature.\n"
		"; Obtain a free key from Google AI Studio (aistudio.google.com), then paste it\n"
		"; here and restart the app. The app never writes this value.\n"
		"; Without a key the Analyze Liftoff and Analyze Landing buttons are disabled.\n"
		"gemini_api_key=\n"
		"\n"
		<< header("recording") << "\n"
		"; Maximum time between telemetry samples written to trip_data, in milliseconds.\n"
		"; Lower values produce finer trajectory and chart resolution at the cost of\n"
		"; a larger database and slower trip load times. Must be a positive integer.\n"
		"; Default: 500  (0.5 s — adequate for all aircraft types including fast jets\n"
		"; at subsonic speeds; go lower only for supersonic recording needs).\n"
		"sample_interval_ms=" << kDefaultSampleIntervalMs << "\n"
		"\n"
		<< commentLines(recordingEnabledComment()).join('\n') << "\n"
		"enabled=true\n"
		"\n"
		<< header("logging") << "\n"
		"; Maximum log level written to msfs_fdr_debug.log.\n"
		"; Levels (inclusive — each includes all levels above it):\n"
		";   FATAL    — unrecoverable errors only\n"
		";   WARNING  — unexpected conditions that don't abort the app\n"
		";   INFO     — operational events (connect, recording start/stop, liftoff, touchdown)\n"
		";   TRACE    — fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)\n"
		";   PROFILE  — performance timing for all subsystems (highest volume; for profiling only)\n"
		"; Default: INFO\n"
		"verbose=INFO\n"
		"\n"
		<< header("layout") << "\n"
		<< commentLines(rightPanelWidthComment()).join('\n') << "\n"
		"right_panel_width=" << kRightPanelWidth << "\n"
		"\n"
		<< commentLines(chartsPanelHeightComment()).join('\n') << "\n"
		"charts_panel_height=" << kDefaultChartsPanelHeight << "\n"
		"\n"
		<< header("data_table") << "\n"
		<< commentLines(hiddenFieldsComment()).join('\n') << "\n"
		"hidden_fields=\n"
		"\n"
		<< header("table_column_width") << "\n"
		<< commentLines(fieldColumnWidthComment()).join('\n') << "\n"
		"data_table_field_column_width=" << kDefaultFieldColumnWidth << "\n"
		"\n"
		<< commentLines(tripHistoryColumnWidthsComment()).join('\n') << "\n"
		"trip_history_column_widths=\n";

	Logger::logf(Logger::Trace, "Settings", "Default settings.ini created at %s", qUtf8Printable(path));
}

}

QString AppSettings::filePath() {
	return settingsFilePath();
}

AppSettings& AppSettings::instance() {
	static bool _ = (ensureSettingsFileExists(), true);
	static AppSettings settings;
	(void)_;
	return settings;
}

QStringList AppSettings::dataTableHiddenFields() const {
	return commaList("data_table/hidden_fields");
}

void AppSettings::setDataTableHiddenFields(const QStringList& fields) {
	Logger::logf(Logger::Trace, "Settings", "Data Table field visibility changed: %d field(s) now hidden", (int)fields.size());
	writeIniValue(
		QStringLiteral("data_table"),
		QStringLiteral("hidden_fields"),
		fields.join(','),
		hiddenFieldsComment()
	);
}

int AppSettings::dataTableFieldColumnWidth() const {
	return positiveInt("table_column_width/data_table_field_column_width", kDefaultFieldColumnWidth);
}

void AppSettings::setDataTableFieldColumnWidth(int w) {
	writeIniValue(
		QStringLiteral("table_column_width"),
		QStringLiteral("data_table_field_column_width"),
		QString::number(w),
		fieldColumnWidthComment()
	);
}

int AppSettings::rightPanelWidth() const {
	return positiveInt("layout/right_panel_width", kRightPanelWidth);
}

void AppSettings::setRightPanelWidth(int w) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("right_panel_width"),
		QString::number(w),
		rightPanelWidthComment()
	);
}

int AppSettings::chartsPanelHeight() const {
	return positiveInt("layout/charts_panel_height", kDefaultChartsPanelHeight);
}

void AppSettings::setChartsPanelHeight(int h) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("charts_panel_height"),
		QString::number(h),
		chartsPanelHeightComment()
	);
}

QMap<QString, int> AppSettings::tripHistoryColumnWidths() const {
	QMap<QString, int> widths;
	for (const QString& part : commaList("table_column_width/trip_history_column_widths")) {
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
		tripHistoryColumnWidthsComment()
	);
}

QString AppSettings::logLevel() {
	return makeSettings().value(QStringLiteral("logging/verbose"), QStringLiteral("INFO")).toString();
}

QString AppSettings::geminiApiKey() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("ai/gemini_api_key")).toString();
}

int AppSettings::sampleIntervalMs() const {
	const int v = positiveInt("recording/sample_interval_ms", 0);
	if (v > 0) {
		Logger::logf(Logger::Trace, "Settings", "sample_interval_ms=%d read from settings.ini", v);
		return v;
	}
	Logger::logf(Logger::Trace, "Settings", "sample_interval_ms missing or invalid in settings.ini; falling back to default %d",
		kDefaultSampleIntervalMs);
	return kDefaultSampleIntervalMs;
}

bool AppSettings::recordingEnabled() const {
	QSettings settings = makeSettings();
	return settings.value(QStringLiteral("recording/enabled"), true).toBool();
}

void AppSettings::setRecordingEnabled(bool enabled) {
	Logger::logf(Logger::Trace, "Settings", "recording/enabled set to %s", enabled ? "true" : "false");
	writeIniValue(
		QStringLiteral("recording"),
		QStringLiteral("enabled"),
		enabled ? QStringLiteral("true") : QStringLiteral("false"),
		recordingEnabledComment()
	);
}

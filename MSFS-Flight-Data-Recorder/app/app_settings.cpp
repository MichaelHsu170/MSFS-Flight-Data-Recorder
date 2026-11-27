#include "app_settings.h"
#include "app_paths.h"
#include "logger.h"

#include <QFile>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSettings>
#include <QStringDecoder>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

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

// The comment at the top of the file. A file's first block is taken for an
// older version of it when it starts with kBannerStart.
const QString kBannerStart = QStringLiteral("MSFS Flight Data Recorder");
const QString kBanner = kBannerStart + QStringLiteral(" -- settings\n"
	"Edit while the app is not running. All values are human-readable.\n"
	"The app updates the comments directly above each setting and section, so leave\n"
	"a blank line between them and notes of your own.");

// The sections, in the default file's order, each with the comment above its
// header (empty for none).
struct Section {
	QString name;
	QString comment;
};

const std::vector<Section>& sectionTable() {
	static const std::vector<Section> sections = {
		{ QStringLiteral("ai"), QStringLiteral(
			"AI analysis of liftoffs and landings (the Analyze button in the map popups).\n"
			"  1. Get an API key from ONE of the providers below.\n"
			"  2. Remove the ; in front of that provider's line and paste the key after its\n"
			"     \"=\". Leave the ; in front of the other two.\n"
			"  3. Save this file and restart the app.\n"
			"A line with a ; in front is not used, so a spare key can be kept there.\n"
			"Analyze is disabled if no key is set, or more than one.\n"
			"To switch provider, put a ; in front of the old key's line and remove the one in\n"
			"front of the new key's. A saved model whose name shows it's another provider's\n"
			"is then cleared, and the new provider's default is used.\n"
			"The app never changes your keys, and puts a ; in front of a line without one.\n"
			"They are stored here as plain text, so don't share this file.") },
		{ QStringLiteral("recording"), QString() },
		{ QStringLiteral("logging"), QString() },
		{ QStringLiteral("layout"), QStringLiteral("Auto-managed by the app.") },
		{ QStringLiteral("data_table"), QStringLiteral("Auto-managed by the app.") },
		{ QStringLiteral("table_column_width"), QStringLiteral(
			"Auto-managed by the app. Persisted column widths for the tables in the\n"
			"UI that support user resizing.") },
	};
	return sections;
}

// The settings, in the default file's order within their section, each with
// the comment above it. The one source for the default file, for a key a
// setter adds and for the startup refresh (refreshComments()).
struct Setting {
	QString section;
	QString key;
	QString defaultValue;
	QString comment;
};

const std::vector<Setting>& settingTable() {
	static const std::vector<Setting> settings = {
		{ QStringLiteral("ai"), QStringLiteral("gemini_api_key"), QString(), QStringLiteral(
			"Google Gemini: aistudio.google.com (some models are free to use).") },
		{ QStringLiteral("ai"), QStringLiteral("openai_api_key"), QString(), QStringLiteral(
			"OpenAI: platform.openai.com. Billed per use, separately from a ChatGPT\n"
			"subscription.") },
		{ QStringLiteral("ai"), QStringLiteral("claude_api_key"), QString(), QStringLiteral(
			"Anthropic Claude: platform.claude.com. Needs an API key, not a Claude app\n"
			"login.") },
		{ QStringLiteral("ai"), QStringLiteral("model"), QString(), QStringLiteral(
			"Auto-managed by the app. The model the analysis uses, chosen in the drop-down\n"
			"beside the Analyze button, or other for the one named in custom_model.\n"
			"Empty: the provider's default.") },
		{ QStringLiteral("ai"), QStringLiteral("custom_model"), QString(), QStringLiteral(
			"Auto-managed by the app. The model name entered for the drop-down's Other\n"
			"choice (model=other).") },
		{ QStringLiteral("ai"), QStringLiteral("progress_evaluation"), QStringLiteral("true"), QStringLiteral(
			"Auto-managed by the app. Whether the AI analysis is a progress evaluation\n"
			"(the whole climb-out or approach) or, false, a contact evaluation (the\n"
			"liftoff or touchdown alone), as the popup's checkbox left it. Default: true.") },
		{ QStringLiteral("recording"), QStringLiteral("sample_interval_ms"), QString::number(kDefaultSampleIntervalMs), QStringLiteral(
			"Maximum time between telemetry samples written to trip_data, in milliseconds.\n"
			"Lower values produce finer trajectory and chart resolution at the cost of\n"
			"a larger database and slower trip load times. Must be a positive integer.\n"
			"Default: %1  (0.5 s -- adequate for all aircraft types including fast jets\n"
			"at subsonic speeds; go lower only for supersonic recording needs).").arg(kDefaultSampleIntervalMs) },
		{ QStringLiteral("recording"), QStringLiteral("enabled"), QStringLiteral("true"), QStringLiteral(
			"Auto-managed by the app. Whether automatic recording is allowed to start,\n"
			"toggled via the Recording indicator in the Live Status panel. Disabling it\n"
			"only prevents a new trip from starting; it doesn't stop one already in\n"
			"progress. Default: true.") },
		{ QStringLiteral("logging"), QStringLiteral("verbose"), QStringLiteral("INFO"), QStringLiteral(
			"Maximum log level written to msfs_fdr_debug.log.\n"
			"Levels (inclusive -- each includes all levels above it):\n"
			"  FATAL    -- unrecoverable errors only\n"
			"  WARNING  -- unexpected conditions that don't abort the app\n"
			"  INFO     -- operational events (connect, recording start/stop, liftoff, touchdown)\n"
			"  TRACE    -- fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)\n"
			"  PROFILE  -- performance timing for all subsystems (highest volume; for profiling only)\n"
			"Default: INFO") },
		{ QStringLiteral("layout"), QStringLiteral("right_panel_width"), QString::number(kRightPanelWidth), QStringLiteral(
			"Width in pixels of the Live Status panel (top-right) and Data Table panel\n"
			"(bottom-right). Both columns share one value so they stay aligned when\n"
			"either splitter is dragged. Default: %1.").arg(kRightPanelWidth) },
		{ QStringLiteral("layout"), QStringLiteral("charts_panel_height"), QString::number(kDefaultChartsPanelHeight), QStringLiteral(
			"Height in pixels of the Charts panel (below the map). The map takes the\n"
			"remaining vertical space. Default: %1.").arg(kDefaultChartsPanelHeight) },
		{ QStringLiteral("data_table"), QStringLiteral("hidden_fields"), QString(), QStringLiteral(
			"Comma-separated list of field labels hidden in the Data Table panel via the\n"
			"Visible Fields dialog. Absent or empty means all fields are visible.") },
		{ QStringLiteral("table_column_width"), QStringLiteral("data_table_field_column_width"),
			QString::number(kDefaultFieldColumnWidth), QStringLiteral(
			"Width in pixels of the Field column in the Data Table panel. The Value\n"
			"column always stretches to fill the rest. Default: %1.").arg(kDefaultFieldColumnWidth) },
		{ QStringLiteral("table_column_width"), QStringLiteral("trip_history_column_widths"), QString(), QStringLiteral(
			"Column widths in pixels for the Trip History table, as comma-separated\n"
			"key=value pairs keyed by TripHistoryModel::Column enum member name (e.g.\n"
			"TitleColumn=120). Columns using Stretch sizing are never stored. Unknown\n"
			"or missing keys fall back to that column's coded default.") },
	};
	return settings;
}

// A section's header line, after the comment that goes above it. Written both
// into the default settings.ini and when a setter adds the section to a file
// that lacks it, so the two always match.
QStringList sectionHeaderLines(const QString& name) {
	QStringList out;
	for (const Section& section : sectionTable())
		if (section.name == name)
			out = commentLines(section.comment);
	return out << '[' + name + ']';
}

QString settingComment(const QString& section, const QString& key) {
	for (const Setting& setting : settingTable())
		if (setting.section == section && setting.key == key)
			return setting.comment;
	return {};
}

bool isComment(const QString& line) {
	const QString t = line.trimmed();
	return t.startsWith(';') || t.startsWith('#');
}

// The name of the section a header line starts, or empty for any other line.
// It and the two keys below are lowercase, as the tables' names are: QSettings
// reads them in any case on Windows.
// A trailing "; comment" or "# comment" (e.g. from manual editing) isn't part
// of the header: comparing the raw line would fail to recognize an existing
// section, and a setter would append a duplicate one.
QString headerSection(const QString& line) {
	static const QRegularExpression re(QStringLiteral("^\\[([^\\]]*)\\]\\s*(?:[;#].*)?$"));
	return re.match(line.trimmed()).captured(1).trimmed().toLower();
}

// The key a setting line sets. A comment's or header's is never a known one.
QString lineKey(const QString& line) {
	return line.contains('=') ? line.section('=', 0, 0).trimmed().toLower() : QString();
}

// The key of a commented-out setting (";enabled=false"), or empty for any
// other line. With a space after the comment sign, as every comment the app
// writes has, only a known key counts, so prose isn't taken for a setting.
QString commentedOutKey(const QString& line) {
	static const QRegularExpression re(QStringLiteral("^[;#]+(\\s*)(\\w+)\\s*="));
	const QRegularExpressionMatch m = re.match(line.trimmed());
	const QString key = m.captured(2).toLower();
	const bool known = std::any_of(settingTable().begin(), settingTable().end(),
		[&key](const Setting& setting) { return setting.key == key; });
	return known || m.captured(1).isEmpty() ? key : QString();
}

// A comment line that isn't a commented-out setting: prose.
bool isProseComment(const QString& line) {
	return isComment(line) && commentedOutKey(line).isEmpty();
}

// The settings a refresh adds where they're missing, so users see where an API
// key goes: the [ai] section's keys. Without a value they're written commented
// out (emptyApiKeyLine()), the way the comments tell users to leave them.
bool isApiKey(const QString& key) {
	return key.endsWith(QLatin1String("_api_key"));
}

QString emptyApiKeyLine(const QString& key) {
	return ';' + key + '=';
}

// Whether a line is an API key without a value, commented out or not.
bool isEmptyApiKey(const QString& line) {
	return isApiKey(isComment(line) ? commentedOutKey(line) : lineKey(line)) && line.section('=', 1).trimmed().isEmpty();
}

// Where a section and one of its keys are in the file's lines, -1 for none. Of
// a section or key there more than once, the last, whose value QSettings reads.
struct Place {
	int header = -1;       // the section's header line
	int last = -1;         // its last line
	int key = -1;          // its line that sets the key
	int commentedOut = -1; // its first line that sets the key, commented out
	bool repeated = false; // the section or the key is there more than once
};

Place findPlace(const QStringList& lines, const QString& section, const QString& key) {
	Place place;
	bool inSection = false;
	for (int i = 0; i < lines.size(); ++i) {
		if (lines[i].trimmed().startsWith('[')) {
			inSection = headerSection(lines[i]) == section;
			if (!inSection)
				continue;
			place.repeated = place.header >= 0;
			place.header = place.last = i;
		} else if (inSection) {
			place.last = i;
			if (lineKey(lines[i]) == key) {
				place.repeated = place.repeated || place.key >= 0;
				place.key = i;
			} else if (place.commentedOut < 0 && commentedOutKey(lines[i]) == key) {
				place.commentedOut = i;
			}
		}
	}
	return place;
}

// Replaces the comment directly above lines[anchor], up to a blank line
// or a commented-out setting, with comment.
void replaceCommentAbove(QStringList& lines, int anchor, const QString& comment, const QString& lineEnd) {
	int top = anchor;
	while (top > 0 && isProseComment(lines[top - 1]))
		--top;
	QStringList updated = commentLines(comment);
	for (QString& line : updated)
		line += lineEnd;
	lines = lines.mid(0, top) + updated + lines.mid(anchor);
}

// lines (each with its own '\r', if any) with the comments of the banner and
// of every known section and setting there brought up to date, [ai]'s missing
// API keys added and its empty ones commented out; none if a section or
// setting is there more than once.
std::optional<QStringList> refreshedLines(QStringList lines) {
	for (const Setting& setting : settingTable()) {
		if (findPlace(lines, setting.section, setting.key).repeated) {
			Logger::logf(Logger::Warning, "Settings", "[%s] or its %s is in settings.ini more than once; comments not updated",
				qUtf8Printable(setting.section), qUtf8Printable(setting.key));
			return std::nullopt;
		}
	}

	const QString lineEnd = lines.first().endsWith('\r') ? QStringLiteral("\r") : QString();
	// The banner, with the blank line that keeps it apart from what follows.
	int bannerEnd = 0;
	if (lines.first().trimmed().mid(1).trimmed().startsWith(kBannerStart))
		while (bannerEnd < lines.size() && isProseComment(lines[bannerEnd]))
			++bannerEnd;
	if (bannerEnd == lines.size() || !lines[bannerEnd].trimmed().isEmpty())
		lines.insert(bannerEnd, lineEnd);
	replaceCommentAbove(lines, bannerEnd, kBanner, lineEnd);

	// The missing API keys, each after the one before it.
	int previous = -1;
	for (const Setting& setting : settingTable()) {
		if (!isApiKey(setting.key))
			continue;
		const Place place = findPlace(lines, setting.section, setting.key);
		if (place.header < 0)
			break;
		if (place.key >= 0 && isEmptyApiKey(lines[place.key]))
			lines[place.key] = emptyApiKeyLine(setting.key) + lineEnd;
		if (place.key >= 0 || place.commentedOut >= 0) {
			previous = place.key >= 0 ? place.key : place.commentedOut;
			continue;
		}
		const int at = previous >= 0 ? previous + 1 : place.header + 1;
		if (at == lines.size() || !lines[at].trimmed().isEmpty())
			lines.insert(at, lineEnd);
		lines.insert(at, emptyApiKeyLine(setting.key) + lineEnd);
		previous = at;
		if (at != place.header + 1) {
			lines.insert(at, lineEnd);
			++previous;
		}
	}

	for (const Section& section : sectionTable()) {
		const int header = findPlace(lines, section.name, QString()).header;
		if (header >= 0 && !section.comment.isEmpty())
			replaceCommentAbove(lines, header, section.comment, lineEnd);
	}
	for (const Setting& setting : settingTable()) {
		const Place place = findPlace(lines, setting.section, setting.key);
		int anchor = place.key >= 0 ? place.key : place.commentedOut;
		if (anchor < 0)
			continue;
		// Spare values commented out above the setting share its comment.
		while (commentedOutKey(lines[anchor - 1]) == setting.key)
			--anchor;
		replaceCommentAbove(lines, anchor, setting.comment, lineEnd);
	}
	return lines;
}

// The lines a refresh must leave as they are: everything but blank lines,
// comments and empty API keys.
QStringList contentLines(const QStringList& lines) {
	QStringList out;
	for (const QString& line : lines)
		if (!line.trimmed().isEmpty() && !isProseComment(line) && !isEmptyApiKey(line))
			out.append(line);
	return out;
}

// settings.ini's lines (each with its own '\r', if any), or none if it can't be
// read or isn't UTF-8, with why.
std::optional<QStringList> readLines(QString& why) {
	QFile file(settingsFilePath());
	if (!file.open(QIODevice::ReadOnly)) {
		why = QStringLiteral("can't read the file");
		return std::nullopt;
	}
	QStringDecoder decoder(QStringDecoder::Utf8, QStringDecoder::Flag::Stateless); // Stateless: a sequence cut off at the end is an error
	const QString text = decoder(file.readAll());
	if (decoder.hasError()) {
		why = QStringLiteral("the file isn't UTF-8");
		return std::nullopt;
	}
	return text.split('\n');
}

// Replaces settings.ini's text as a whole, or not at all, keeping the file
// before as backup if given. Returns why it couldn't, or empty.
QString replaceText(const QString& text, const QString& backup = {}) {
	const QString path = settingsFilePath();
	QSaveFile out(path);
	if (!out.open(QIODevice::WriteOnly))
		return QStringLiteral("can't write the file");
	// The backup before, put back if the file can't be replaced after all.
	const QString old = backup + QStringLiteral(".old");
	if (!backup.isEmpty()) {
		QFile::remove(old);
		if (QFile::exists(backup) && !QFile::rename(backup, old))
			return QStringLiteral("can't write settings.ini.bak");
		if (!QFile::copy(path, backup)) {
			QFile::rename(old, backup);
			return QStringLiteral("can't write settings.ini.bak");
		}
	}
	out.write(text.toUtf8());
	if (!out.commit()) {
		if (!backup.isEmpty()) {
			QFile::remove(backup);
			QFile::rename(old, backup);
		}
		return QStringLiteral("can't write the file");
	}
	if (!backup.isEmpty())
		QFile::remove(old);
	return {};
}

// Brings the comments of an existing settings.ini up to date (refreshedLines()),
// keeping the file it replaces as settings.ini.bak. On any doubt or failure
// the file stays as it is. Returns whether the file is up to date now, so that
// one left as it is isn't changed by anything else at startup either.
bool refreshComments() {
	const auto skip = [](const QString& why) {
		Logger::logf(Logger::Warning, "Settings", "settings.ini's comments not updated: %s", qUtf8Printable(why));
		return false;
	};
	QString why;
	const std::optional<QStringList> lines = readLines(why);
	if (!lines)
		return skip(why);
	const std::optional<QStringList> refreshed = refreshedLines(*lines);
	if (!refreshed)
		return false;
	if (*refreshed == *lines)
		return true;
	if (contentLines(*refreshed) != contentLines(*lines))
		return skip(QStringLiteral("the update would change more than comments and empty API keys"));
	why = replaceText(refreshed->join('\n'), settingsFilePath() + QStringLiteral(".bak"));
	if (!why.isEmpty())
		return skip(why);
	Logger::log(Logger::Info, "Settings", QStringLiteral("settings.ini's comments updated (previous file: settings.ini.bak)"));
	return true;
}

// Writes a single key=value in the named INI section, touching only that one
// line. Every other line -- comments, blank lines, other keys, other sections --
// is preserved exactly (a file mixing line endings gets its first line's). A
// file that can't be read, isn't UTF-8 or can't be replaced stays as it is.
//
// Comments are written only when new content is appended to the file:
//   - a section that is absent is added with sectionHeaderLines(), its
//     comment included.
//   - the key's comment (settingTable()) is written before the
//     key=value line when the key is absent (whether or not the section
//     already existed).
// This means the file stays self-documenting even when keys are added by a
// newer version of the app to an older settings.ini.
void writeIniValue(const QString& section, const QString& key, const QString& value) {
	const QString keyComment = settingComment(section, key);
	const auto skip = [&section, &key](const QString& why) {
		Logger::logf(Logger::Warning, "Settings", "[%s] %s not saved: %s",
			qUtf8Printable(section), qUtf8Printable(key), qUtf8Printable(why));
	};
	QStringList lines;
	if (QFile::exists(settingsFilePath())) {
		QString why;
		const std::optional<QStringList> read = readLines(why);
		if (!read)
			return skip(why);
		lines = *read;
		if (lines.last().isEmpty()) // after the final line's end
			lines.removeLast();
	}
	const QString lineEnd = lines.isEmpty() || lines.first().endsWith('\r') ? QStringLiteral("\r\n") : QStringLiteral("\n");
	for (QString& line : lines)
		if (line.endsWith('\r'))
			line.chop(1);

	const Place place = findPlace(lines, section, key);
	int sectionLastLine = place.last;

	// A trailing blank line + comment here aren't necessarily this section's
	// own trailing content -- they're also exactly what the "section not
	// present" branch below writes as the auto-generated preamble (blank
	// separator + section comment) of a *later* section, written before its
	// own header ever appeared in the file. Trim them back off the end of
	// this section so a later insertion into *this* section can't land
	// inside that preamble and separate it from the header it belongs to. A
	// commented-out setting is the section's own.
	while (sectionLastLine > place.header) {
		const QString t = lines[sectionLastLine].trimmed();
		if (t.isEmpty() || isProseComment(t))
			--sectionLastLine;
		else
			break;
	}

	const QString entry = key + '=' + value;

	if (place.key >= 0) {
		// Key already present -- patch value only, leave comment untouched.
		lines[place.key] = entry;
	} else if (sectionLastLine >= 0) {
		// Section exists but key is missing -- insert key (with comment) after
		// the last line of the section, preserving everything that follows. A
		// blank line separates it from the prior key, matching the grouping
		// convention used between distinct settings elsewhere in this file
		// (see prepareFile()) -- skipped if there's no comment to
		// separate (nothing to visually group) or the prior line is already blank.
		QStringList toInsert;
		if (!keyComment.isEmpty() && !lines[sectionLastLine].trimmed().isEmpty())
			toInsert.append(QString());
		toInsert += commentLines(keyComment);
		toInsert.append(entry);
		for (int j = toInsert.size() - 1; j >= 0; --j)
			lines.insert(sectionLastLine + 1, toInsert[j]);
	} else {
		// Section not present -- append section header and key at end of file.
		if (!lines.isEmpty() && !lines.last().trimmed().isEmpty())
			lines.append(QString());
		lines << sectionHeaderLines(section);
		lines << commentLines(keyComment);
		lines.append(entry);
	}

	const QString why = replaceText(lines.join(lineEnd) + lineEnd);
	if (!why.isEmpty())
		skip(why);
}

// The provider a model's name shows it belongs to, as its API key's setting, or
// empty when the name doesn't show it.
QString modelProviderKey(const QString& model) {
	static const std::vector<std::pair<QRegularExpression, QString>> names = {
		{ QRegularExpression(QStringLiteral("^(?:gemini|gemma)-")), QStringLiteral("gemini_api_key") },
		{ QRegularExpression(QStringLiteral("^(?:gpt-|chatgpt-|chat-|ft:|o\\d)")), QStringLiteral("openai_api_key") },
		{ QRegularExpression(QStringLiteral("^claude-")), QStringLiteral("claude_api_key") },
	};
	for (const auto& [name, key] : names)
		if (name.match(model).hasMatch())
			return key;
	return {};
}

// With exactly one API key set, clears a saved model and custom model that
// belong to another provider (the key was switched since they were chosen).
void clearOtherProvidersModels() {
	const QSettings settings = makeSettings();
	const auto value = [&settings](const QString& key) { return settings.value(QStringLiteral("ai/") + key).toString().trimmed(); };
	QStringList set;
	for (const Setting& setting : settingTable())
		if (isApiKey(setting.key) && !value(setting.key).isEmpty())
			set.append(setting.key);
	if (set.size() != 1)
		return;
	const auto isOthers = [&set](const QString& model) {
		const QString key = modelProviderKey(model);
		return !key.isEmpty() && key != set.first();
	};
	const QString model = value(QStringLiteral("model"));
	const bool clearCustom = isOthers(value(QStringLiteral("custom_model")));
	if (isOthers(model) || (clearCustom && model == QLatin1String("other")))
		writeIniValue(QStringLiteral("ai"), QStringLiteral("model"), QString());
	if (clearCustom)
		writeIniValue(QStringLiteral("ai"), QStringLiteral("custom_model"), QString());
}

}

void AppSettings::prepareFile() {
	const QString path = settingsFilePath();
	if (QFile::exists(path)) {
		Logger::logf(Logger::Trace, "Settings", "Using existing settings.ini at %s", qUtf8Printable(path));
		if (refreshComments())
			clearOtherProvidersModels();
		return;
	}
	Logger::logf(Logger::Trace, "Settings", "No settings.ini found; creating default at %s", qUtf8Printable(path));

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		Logger::logf(Logger::Warning, "Settings",
			"Failed to create default settings.ini at %s", qUtf8Printable(path));
		return;
	}

	QStringList lines = commentLines(kBanner);
	for (const Section& section : sectionTable()) {
		lines << QString() << sectionHeaderLines(section.name);
		bool first = true;
		for (const Setting& setting : settingTable()) {
			if (setting.section != section.name)
				continue;
			if (!first)
				lines << QString();
			first = false;
			lines << commentLines(setting.comment)
				<< (isApiKey(setting.key) ? emptyApiKeyLine(setting.key) : setting.key + '=' + setting.defaultValue);
		}
	}
	file.write((lines.join('\n') + '\n').toUtf8());

	Logger::logf(Logger::Trace, "Settings", "Default settings.ini created at %s", qUtf8Printable(path));
}

QString AppSettings::filePath() {
	return settingsFilePath();
}

AppSettings& AppSettings::instance() {
	static bool _ = (prepareFile(), true);
	static AppSettings settings;
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
		fields.join(',')
	);
}

int AppSettings::dataTableFieldColumnWidth() const {
	return positiveInt("table_column_width/data_table_field_column_width", kDefaultFieldColumnWidth);
}

void AppSettings::setDataTableFieldColumnWidth(int w) {
	writeIniValue(
		QStringLiteral("table_column_width"),
		QStringLiteral("data_table_field_column_width"),
		QString::number(w)
	);
}

int AppSettings::rightPanelWidth() const {
	return positiveInt("layout/right_panel_width", kRightPanelWidth);
}

void AppSettings::setRightPanelWidth(int w) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("right_panel_width"),
		QString::number(w)
	);
}

int AppSettings::chartsPanelHeight() const {
	return positiveInt("layout/charts_panel_height", kDefaultChartsPanelHeight);
}

void AppSettings::setChartsPanelHeight(int h) {
	writeIniValue(
		QStringLiteral("layout"),
		QStringLiteral("charts_panel_height"),
		QString::number(h)
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
		parts.join(',')
	);
}

QString AppSettings::logLevel() {
	return makeSettings().value(QStringLiteral("logging/verbose"), QStringLiteral("INFO")).toString();
}

QString AppSettings::geminiApiKey() const {
	return makeSettings().value(QStringLiteral("ai/gemini_api_key")).toString();
}

QString AppSettings::openaiApiKey() const {
	return makeSettings().value(QStringLiteral("ai/openai_api_key")).toString();
}

QString AppSettings::claudeApiKey() const {
	return makeSettings().value(QStringLiteral("ai/claude_api_key")).toString();
}

QString AppSettings::aiModel() const {
	return makeSettings().value(QStringLiteral("ai/model")).toString();
}

void AppSettings::setAiModel(const QString& model) {
	writeIniValue(QStringLiteral("ai"), QStringLiteral("model"), model);
}

QString AppSettings::aiCustomModel() const {
	return makeSettings().value(QStringLiteral("ai/custom_model")).toString();
}

void AppSettings::setAiCustomModel(const QString& name) {
	writeIniValue(QStringLiteral("ai"), QStringLiteral("custom_model"), name);
}

bool AppSettings::aiProgressEvaluation() const {
	return makeSettings().value(QStringLiteral("ai/progress_evaluation"), true).toBool();
}

void AppSettings::setAiProgressEvaluation(bool progress) {
	writeIniValue(QStringLiteral("ai"), QStringLiteral("progress_evaluation"),
		progress ? QStringLiteral("true") : QStringLiteral("false"));
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
	return makeSettings().value(QStringLiteral("recording/enabled"), true).toBool();
}

void AppSettings::setRecordingEnabled(bool enabled) {
	Logger::logf(Logger::Trace, "Settings", "recording/enabled set to %s", enabled ? "true" : "false");
	writeIniValue(
		QStringLiteral("recording"),
		QStringLiteral("enabled"),
		enabled ? QStringLiteral("true") : QStringLiteral("false")
	);
}

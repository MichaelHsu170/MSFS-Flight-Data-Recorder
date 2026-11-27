// settings.ini (app_settings.cpp): default file, defaults for missing or bad
// values, and in-place edits that keep comments and other sections intact.
#include "test_support.h"

#include "app_settings.h"

#include <QDir>
#include <QFile>
#include <QProcess>
#include <QtTest>

#include <functional>
#include <utility>
#include <vector>

using namespace TestSupport;

namespace {

QString readFile(const QString& path) {
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return QString();
	return QString::fromUtf8(f.readAll());
}

QString readSettingsFile() {
	return readFile(AppSettings::filePath());
}

QString backupPath() {
	return AppSettings::filePath() + QStringLiteral(".bak");
}

// The file's exact bytes, line endings included.
QByteArray readBytes() {
	QFile f(AppSettings::filePath());
	return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

void writeBytes(const QByteArray& bytes) {
	QFile f(AppSettings::filePath());
	QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Truncate));
	f.write(bytes);
}

const QString kBannerText = QStringLiteral(
	"; MSFS Flight Data Recorder -- settings\n"
	"; Edit while the app is not running. All values are human-readable.\n"
	"; The app updates the comments directly above each setting and section, so leave\n"
	"; a blank line between them and notes of your own.\n");

const QString kAiHeader = QStringLiteral(
	"; AI analysis of liftoffs and landings (the Analyze button in the map popups).\n"
	";   1. Get an API key from ONE of the providers below.\n"
	";   2. Remove the ; in front of that provider's line and paste the key after its\n"
	";      \"=\". Leave the ; in front of the other two.\n"
	";   3. Save this file and restart the app.\n"
	"; A line with a ; in front is not used, so a spare key can be kept there.\n"
	"; Analyze is disabled if no key is set, or more than one.\n"
	"; To switch provider, put a ; in front of the old key's line and remove the one in\n"
	"; front of the new key's. A saved model whose name shows it's another provider's\n"
	"; is then cleared, and the new provider's default is used.\n"
	"; The app never changes your keys, and puts a ; in front of a line without one.\n"
	"; They are stored here as plain text, so don't share this file.\n"
	"[ai]\n");

const QString kGeminiKeyComment = QStringLiteral("; Google Gemini: aistudio.google.com (some models are free to use).\n");

const QString kOpenaiKeyComment = QStringLiteral(
	"; OpenAI: platform.openai.com. Billed per use, separately from a ChatGPT\n"
	"; subscription.\n");

const QString kClaudeKeyComment = QStringLiteral(
	"; Anthropic Claude: platform.claude.com. Needs an API key, not a Claude app\n"
	"; login.\n");

void writeSettingsFile(const QString& text) {
	QFile f(AppSettings::filePath());
	QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate));
	f.write(text.toUtf8());
}

// What the refresh adds after a file's only API key, Gemini's.
const QString kOtherKeys = QStringLiteral("\n") + kOpenaiKeyComment + QStringLiteral(";openai_api_key=\n\n")
	+ kClaudeKeyComment + QStringLiteral(";claude_api_key=\n");

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
		QCOMPARE(text, QStringLiteral(
			"; MSFS Flight Data Recorder -- settings\n"
			"; Edit while the app is not running. All values are human-readable.\n"
			"; The app updates the comments directly above each setting and section, so leave\n"
			"; a blank line between them and notes of your own.\n"
			"\n"
			"; AI analysis of liftoffs and landings (the Analyze button in the map popups).\n"
			";   1. Get an API key from ONE of the providers below.\n"
			";   2. Remove the ; in front of that provider's line and paste the key after its\n"
			";      \"=\". Leave the ; in front of the other two.\n"
			";   3. Save this file and restart the app.\n"
			"; A line with a ; in front is not used, so a spare key can be kept there.\n"
			"; Analyze is disabled if no key is set, or more than one.\n"
			"; To switch provider, put a ; in front of the old key's line and remove the one in\n"
			"; front of the new key's. A saved model whose name shows it's another provider's\n"
			"; is then cleared, and the new provider's default is used.\n"
			"; The app never changes your keys, and puts a ; in front of a line without one.\n"
			"; They are stored here as plain text, so don't share this file.\n"
			"[ai]\n"
			"; Google Gemini: aistudio.google.com (some models are free to use).\n"
			";gemini_api_key=\n"
			"\n"
			"; OpenAI: platform.openai.com. Billed per use, separately from a ChatGPT\n"
			"; subscription.\n"
			";openai_api_key=\n"
			"\n"
			"; Anthropic Claude: platform.claude.com. Needs an API key, not a Claude app\n"
			"; login.\n"
			";claude_api_key=\n"
			"\n"
			"; Auto-managed by the app. The model the analysis uses, chosen in the drop-down\n"
			"; beside the Analyze button, or other for the one named in custom_model.\n"
			"; Empty: the provider's default.\n"
			"model=\n"
			"\n"
			"; Auto-managed by the app. The model name entered for the drop-down's Other\n"
			"; choice (model=other).\n"
			"custom_model=\n"
			"\n"
			"; Auto-managed by the app. Whether the AI analysis is a progress evaluation\n"
			"; (the whole climb-out or approach) or, false, a contact evaluation (the\n"
			"; liftoff or touchdown alone), as the popup's checkbox left it. Default: true.\n"
			"progress_evaluation=true\n"
			"\n"
			"[recording]\n"
			"; Maximum time between telemetry samples written to trip_data, in milliseconds.\n"
			"; Lower values produce finer trajectory and chart resolution at the cost of\n"
			"; a larger database and slower trip load times. Must be a positive integer.\n"
			"; Default: 500  (0.5 s -- adequate for all aircraft types including fast jets\n"
			"; at subsonic speeds; go lower only for supersonic recording needs).\n"
			"sample_interval_ms=500\n"
			"\n"
			"; Auto-managed by the app. Whether automatic recording is allowed to start,\n"
			"; toggled via the Recording indicator in the Live Status panel. Disabling it\n"
			"; only prevents a new trip from starting; it doesn't stop one already in\n"
			"; progress. Default: true.\n"
			"enabled=true\n"
			"\n"
			"[logging]\n"
			"; Maximum log level written to msfs_fdr_debug.log.\n"
			"; Levels (inclusive -- each includes all levels above it):\n"
			";   FATAL    -- unrecoverable errors only\n"
			";   WARNING  -- unexpected conditions that don't abort the app\n"
			";   INFO     -- operational events (connect, recording start/stop, liftoff, touchdown)\n"
			";   TRACE    -- fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)\n"
			";   PROFILE  -- performance timing for all subsystems (highest volume; for profiling only)\n"
			"; Default: INFO\n"
			"verbose=INFO\n"
			"\n"
			"; Auto-managed by the app.\n"
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
			"; Auto-managed by the app.\n"
			"[data_table]\n"
			"; Comma-separated list of field labels hidden in the Data Table panel via the\n"
			"; Visible Fields dialog. Absent or empty means all fields are visible.\n"
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
			"trip_history_column_widths=\n"));
	}

	// The check instance() runs at startup updates the comments of an
	// existing file, however partial, adds its missing API keys, and
	// nothing else: values, order,
	// unknown keys and sections, and notes kept apart by a blank line stay.
	void startupUpdatesOnlyTheComments() {
		writeSettingsFile("; my own note\n\n[ai]\n; Old words about the key.\n; More of them.\ngemini_api_key=my-key\n"
			"; my last note\n\n; above recording\n[recording]\n; unknown setting's comment\nskip_events=A,B\n\n"
			"; old\nsample_interval_ms=250\n\n[layout]\ncharts_panel_height=309\n\n; mine\n[custom]\n; mine too\nx=1\n");
		QFile::remove(backupPath());
		const QString before = readSettingsFile();
		AppSettings::prepareFile();
		const QString expected = kBannerText + QStringLiteral("\n; my own note\n\n") + kAiHeader + kGeminiKeyComment
			+ QStringLiteral("gemini_api_key=my-key\n") + kOtherKeys + QStringLiteral("\n; my last note\n\n; above recording\n[recording]\n; unknown setting's comment\nskip_events=A,B\n\n"
			"; Maximum time between telemetry samples written to trip_data, in milliseconds.\n"
			"; Lower values produce finer trajectory and chart resolution at the cost of\n"
			"; a larger database and slower trip load times. Must be a positive integer.\n"
			"; Default: 500  (0.5 s -- adequate for all aircraft types including fast jets\n"
			"; at subsonic speeds; go lower only for supersonic recording needs).\n"
			"sample_interval_ms=250\n\n; Auto-managed by the app.\n[layout]\n"
			"; Height in pixels of the Charts panel (below the map). The map takes the\n"
			"; remaining vertical space. Default: 400.\n"
			"charts_panel_height=309\n\n; mine\n[custom]\n; mine too\nx=1\n");
		QCOMPARE(readSettingsFile(), expected);
		QCOMPARE(readFile(backupPath()), before);
		// A second start finds nothing to do: the file and the backup stay.
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), expected);
		QCOMPARE(readFile(backupPath()), before);
	}

	void startupLeavesADefaultFileAsItIs() {
		removeSettings();
		AppSettings::prepareFile();
		const QString text = readSettingsFile();
		QVERIFY(text.startsWith(kBannerText + QStringLiteral("\n") + kAiHeader));
		QFile::remove(backupPath());
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), text);
		QVERIFY(!QFile::exists(backupPath()));
	}

	// An older banner is replaced, and parted from a section right below it.
	void startupReplacesAnOlderBanner() {
		writeSettingsFile("# MSFS Flight Data Recorder settings, old\n# more\n[custom]\nx=1\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + QStringLiteral("\n[custom]\nx=1\n"));
		writeSettingsFile("; MSFS Flight Data Recorder");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText);
	}

	// Sections and settings are recognized in any case, as they're read.
	void startupRecognizesAnyCase() {
		writeSettingsFile("; old\n[AI]\n; old\nGemini_API_Key=k\n; Claude_API_Key=\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + '\n' + QString(kAiHeader).replace("\n[ai]", "\n[AI]") + kGeminiKeyComment
			+ "Gemini_API_Key=k\n\n" + kOpenaiKeyComment + ";openai_api_key=\n\n" + kClaudeKeyComment + "; Claude_API_Key=\n");
		QCOMPARE(AppSettings::instance().geminiApiKey(), QStringLiteral("k"));
	}

	// Commented-out settings aren't comment text: they're never removed, and
	// the comment goes above a setting's own spare values.
	void startupKeepsCommentedOutSettings() {
		writeSettingsFile("[ai]\n; old\n;gemini_api_key=spare1\n; gemini_api_key = spare2\ngemini_api_key=my-key\n"
			";unknown=1\n; old\ncustom_model=x\n\n[recording]\n; old\n#sample_interval_ms=100\n");
		AppSettings::prepareFile();
		const QString text = readSettingsFile();
		QVERIFY2(text.contains(QStringLiteral("[ai]\n") + kGeminiKeyComment
			+ QStringLiteral(";gemini_api_key=spare1\n; gemini_api_key = spare2\ngemini_api_key=my-key\n") + kOtherKeys + QStringLiteral("\n;unknown=1\n"
			"; Auto-managed by the app. The model name entered for the drop-down's Other\n"
			"; choice (model=other).\ncustom_model=x\n")), qPrintable(text));
		QVERIFY2(text.endsWith(QStringLiteral("; at subsonic speeds; go lower only for supersonic recording needs).\n"
			"#sample_interval_ms=100\n")), qPrintable(text));
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), text);
	}

	void startupKeepsWindowsLineEndings() {
		writeBytes("[ai]\r\n; old\r\ngemini_api_key=my-key\r\n");
		AppSettings::prepareFile();
		const QByteArray expected = (kBannerText + QStringLiteral("\n") + kAiHeader + kGeminiKeyComment + QStringLiteral("gemini_api_key=my-key\n") + kOtherKeys)
			.replace('\n', QStringLiteral("\r\n")).toUtf8();
		QCOMPARE(readBytes(), expected);
		writeBytes("[ai]\n; old\ngemini_api_key=my-key\n");
		AppSettings::prepareFile();
		QCOMPARE(readBytes(), QByteArray(expected).replace("\r\n", "\n"));
	}

	void startupLeavesAFileItCantSafelyUpdate_data() {
		QTest::addColumn<QByteArray>("bytes");
		QTest::newRow("key twice") << QByteArray("[ai]\n; old\nmodel=a\nmodel=b\n");
		QTest::newRow("section twice") << QByteArray("[ai]\n; old\nmodel=a\n[x]\n[ai] ; again\ncustom_model=b\n");
		QTest::newRow("section twice, its settings all in the second") << QByteArray("[recording]\n[x]\n; old\n[recording]\nsample_interval_ms=1\nenabled=true\n");
		QTest::newRow("key twice, other case") << QByteArray("[ai]\n; old\nmodel=a\nModel=b\n");
		QTest::newRow("cut off in a character") << QByteArray("[ai]\n; old\nmodel=a\n[x]\ny=caf\xe9");
		QTest::newRow("not UTF-8") << QByteArray("[ai]\n; old \xe9t\xe9\nmodel=a\n");
		QTest::newRow("key twice, another provider's model") << QByteArray("[ai]\nopenai_api_key=k\nmodel=gemma-4-31b-it\n[layout]\nright_panel_width=1\nright_panel_width=2\n");
		QTest::newRow("section twice, another provider's model") << QByteArray("[ai]\nopenai_api_key=k\nmodel=gemma-4-31b-it\n[x]\n[ai]\n");
		QTest::newRow("not UTF-8, another provider's model") << QByteArray("[ai]\n; old \xe9t\xe9\nopenai_api_key=k\nmodel=gemma-4-31b-it\n");
	}

	void startupLeavesAFileItCantSafelyUpdate() {
		QFETCH(QByteArray, bytes);
		writeBytes(bytes);
		QFile::remove(backupPath());
		AppSettings::prepareFile();
		QCOMPARE(readBytes(), bytes);
		QVERIFY(!QFile::exists(backupPath()));
	}

	// Read-only, locked, or with a backup that can't be replaced: unchanged.
	void startupLeavesAFileItCantWrite() {
		const QByteArray bytes("[ai]\n; old\nopenai_api_key=k\nmodel=gemma-4-31b-it\n");
		const QString path = AppSettings::filePath();
		writeBytes(bytes);
		QFile::remove(backupPath());
		QFile::setPermissions(path, QFile::ReadOwner | QFile::ReadUser);
		AppSettings::prepareFile();
		QFile::setPermissions(path, QFile::ReadOwner | QFile::WriteOwner | QFile::ReadUser | QFile::WriteUser);
		QCOMPARE(readBytes(), bytes);

		HANDLE lock = CreateFileW(reinterpret_cast<LPCWSTR>(QDir::toNativeSeparators(path).utf16()),
			GENERIC_WRITE, FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		QVERIFY(lock != INVALID_HANDLE_VALUE);
		AppSettings::prepareFile();
		CloseHandle(lock);
		QCOMPARE(readBytes(), bytes);

		// A backup another program holds open without sharing its deletion.
		{
			QFile backup(backupPath());
			QVERIFY(backup.open(QIODevice::WriteOnly | QIODevice::Truncate));
			backup.write("before");
		}
		lock = CreateFileW(reinterpret_cast<LPCWSTR>(QDir::toNativeSeparators(backupPath()).utf16()),
			GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		QVERIFY(lock != INVALID_HANDLE_VALUE);
		AppSettings::prepareFile();
		CloseHandle(lock);
		QCOMPARE(readBytes(), bytes);
		QCOMPARE(readFile(backupPath()), QStringLiteral("before"));
	}

	// Another program holding the file open without sharing its deletion
	// blocks the swap; the backup from before stays.
	void startupLeavesAFileItCantReplace() {
		const QByteArray bytes("[ai]\n; old\nopenai_api_key=k\nmodel=gemma-4-31b-it\n");
		writeBytes(bytes);
		{
			QFile backup(backupPath());
			QVERIFY(backup.open(QIODevice::WriteOnly | QIODevice::Truncate));
			backup.write("before");
		}
		HANDLE lock = CreateFileW(reinterpret_cast<LPCWSTR>(QDir::toNativeSeparators(AppSettings::filePath()).utf16()),
			GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		QVERIFY(lock != INVALID_HANDLE_VALUE);
		AppSettings::prepareFile();
		CloseHandle(lock);
		QCOMPARE(readBytes(), bytes);
		QCOMPARE(readFile(backupPath()), QStringLiteral("before"));
		QVERIFY(!QFile::exists(backupPath() + ".old"));
	}

	// A refresh keeps the file before it as the backup, the backup before
	// that replaced, and a backup's leftover from an interrupted refresh gone.
	void startupReplacesTheBackup() {
		const QByteArray bytes("[ai]\n; old\ngemini_api_key=k\n");
		writeBytes(bytes);
		for (const QString& path : { backupPath(), backupPath() + ".old" }) {
			QFile f(path);
			QVERIFY(f.open(QIODevice::WriteOnly | QIODevice::Truncate));
			f.write("earlier");
		}
		AppSettings::prepareFile();
		QVERIFY(readBytes().startsWith(kBannerText.toUtf8()));
		QCOMPARE(readFile(backupPath()).toUtf8(), bytes);
		QVERIFY(!QFile::exists(backupPath() + ".old"));
	}

	// A missing API key is added, commented out, after the one before it or under the
	// header; one commented out counts as there. A file without [ai] gets none.
	void startupAddsMissingApiKeys() {
		writeSettingsFile("[ai]\n;openai_api_key=spare\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + QStringLiteral("\n") + kAiHeader + kGeminiKeyComment
			+ QStringLiteral(";gemini_api_key=\n\n") + kOpenaiKeyComment + QStringLiteral(";openai_api_key=spare\n\n")
			+ kClaudeKeyComment + QStringLiteral(";claude_api_key=\n"));
		writeSettingsFile("[ai]");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + QStringLiteral("\n") + kAiHeader + kGeminiKeyComment
			+ QStringLiteral(";gemini_api_key=\n") + kOtherKeys);
		writeSettingsFile("[custom]\nx=1\n");
		AppSettings::prepareFile();
		QVERIFY(!readSettingsFile().contains(QStringLiteral("_api_key")));
	}

	// An API key without a value is commented out; one with a value (even one
	// ending in =) and an empty setting of another kind or section aren't.
	void startupCommentsOutEmptyApiKeys() {
		writeBytes("[ai]\r\ngemini_api_key=abc=\r\nopenai_api_key=\r\n claude_api_key = \r\nmodel=\r\n[custom]\r\nclaude_api_key=\r\n");
		AppSettings::prepareFile();
		const QByteArray text = readBytes();
		QVERIFY2(text.contains("\r\ngemini_api_key=abc=\r\n"), text.constData());
		QVERIFY2(text.contains("\r\n;openai_api_key=\r\n"), text.constData());
		QVERIFY2(text.contains("\r\n;claude_api_key=\r\n") && !text.contains(" claude_api_key ="), text.constData());
		QVERIFY2(text.contains("\r\nmodel=\r\n"), text.constData());
		QVERIFY2(text.endsWith("[custom]\r\nclaude_api_key=\r\n"), text.constData());
		AppSettings::prepareFile();
		QCOMPARE(readBytes(), text);
	}

	// Indented lines are recognized, and kept as they are unless they're
	// comments the app wrote.
	void startupRecognizesIndentedLines() {
		writeSettingsFile("  ; MSFS Flight Data Recorder, older words\n\n  ; old header\n  [ai]\n  ; old key\n  gemini_api_key=k\n\n  ;openai_api_key=spare\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + '\n' + QString(kAiHeader).replace("\n[ai]", "\n  [ai]") + kGeminiKeyComment
			+ "  gemini_api_key=k\n\n" + kOpenaiKeyComment + "  ;openai_api_key=spare\n\n" + kClaudeKeyComment + ";claude_api_key=\n");
	}

	// Every form of a commented-out setting is kept; prose about one that
	// isn't known is an old comment.
	void startupTellsCommentedOutSettingsFromProse() {
		writeSettingsFile("[ai]\n;;gemini_api_key=a\n\n;gemini_api_key=b\n\n;  openai_api_key=c\n\n; speed = x\nmodel=gemma-4-31b-it\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + '\n' + kAiHeader + kGeminiKeyComment
			+ ";;gemini_api_key=a\n\n;gemini_api_key=b\n\n" + kOpenaiKeyComment + ";  openai_api_key=c\n\n" + kClaudeKeyComment + ";claude_api_key=\n\n"
			"; Auto-managed by the app. The model the analysis uses, chosen in the drop-down\n"
			"; beside the Analyze button, or other for the one named in custom_model.\n"
			"; Empty: the provider's default.\nmodel=gemma-4-31b-it\n");
	}

	// A setting's comment goes above the line that sets it, not above a spare
	// value apart from it; an older top block ends at a commented-out setting.
	void startupKeepsSparesApart() {
		writeSettingsFile("[ai]\n;gemini_api_key=spare\n\ngemini_api_key=k\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + '\n' + kAiHeader + ";gemini_api_key=spare\n\n" + kGeminiKeyComment + "gemini_api_key=k\n" + kOtherKeys);
		writeSettingsFile("; MSFS Flight Data Recorder, older words\n;gemini_api_key=x\n[ai]\ngemini_api_key=k\n");
		AppSettings::prepareFile();
		QCOMPARE(readSettingsFile(), kBannerText + "\n;gemini_api_key=x\n" + kAiHeader + kGeminiKeyComment + "gemini_api_key=k\n" + kOtherKeys);
	}

	// Lines added to a file with Windows line endings have them too.
	void startupAddsWindowsLineEndings() {
		writeBytes("[ai]\r\nmodel=a\r\n");
		AppSettings::prepareFile();
		QVERIFY(readBytes().contains(";claude_api_key=\r\n\r\n; Auto-managed"));
		QCOMPARE(readBytes().count("\r\n"), readBytes().count('\n'));
	}

	// Text other than ASCII in a file it updates stays as it is.
	void startupKeepsUtf8Text() {
		writeBytes("[ai]\n; old\ngemini_api_key=k\n\n; caf\xc3\xa9\n\n[custom]\nx=\xe2\x82\xac\n");
		AppSettings::prepareFile();
		QVERIFY(readBytes().startsWith(kBannerText.toUtf8()));
		QVERIFY(readBytes().endsWith("\n\n; caf\xc3\xa9\n\n[custom]\nx=\xe2\x82\xac\n"));
	}

	// A switched key clears the old model in a file already up to date too.
	void switchingProviderInAnUpToDateFileClearsTheModel() {
		removeSettings();
		AppSettings::prepareFile();
		writeSettingsFile(readSettingsFile().replace("\n;openai_api_key=", "\nopenai_api_key=k").replace("\nmodel=", "\nmodel=gemma-4-31b-it"));
		AppSettings::prepareFile();
		QCOMPARE(AppSettings::instance().aiModel(), QString());
	}

	// A setter sets the copy of a repeated key, or of a repeated section's, that
	// is read back (the last, in any case), adds to a section that is only its
	// header, finds a header with spaces inside its brackets, and doesn't take a
	// line without = for the setting.
	void setterFindsItsPlace_data() {
		QTest::addColumn<QString>("before");
		QTest::addColumn<QString>("after");
		const QString added("\n\n; Auto-managed by the app. The model name entered for the drop-down's Other\n; choice (model=other).\ncustom_model=new\n");
		QTest::newRow("bare header") << "[ai]\n" << "[ai]" + added;
		QTest::newRow("bare header, section after") << "[ai]\n\n[x]\n" << "[ai]" + added + "\n[x]\n";
		QTest::newRow("no =") << "[ai]\ncustom_model\n" << "[ai]\ncustom_model" + added;
		QTest::newRow("header with spaces") << "[ ai ]\ncustom_model=a\n" << "[ ai ]\ncustom_model=new\n";
		QTest::newRow("section twice") << "[ai]\ncustom_model=a\n[x]\ny=1\n[ai]\ncustom_model=b\n" << "[ai]\ncustom_model=a\n[x]\ny=1\n[ai]\ncustom_model=new\n";
		QTest::newRow("section twice, key in the first") << "[ai]\ncustom_model=a\n[x]\ny=1\n[ai]\nmodel=b\n" << "[ai]\ncustom_model=new\n[x]\ny=1\n[ai]\nmodel=b\n";
		QTest::newRow("section twice, key in neither") << "[ai]\nmodel=a\n[x]\ny=1\n[ai]\nmodel=b\n" << "[ai]\nmodel=a\n[x]\ny=1\n[ai]\nmodel=b" + added;
		QTest::newRow("key twice, other case") << "[ai]\ncustom_model=a\nCustom_Model=b\n" << "[ai]\ncustom_model=a\ncustom_model=new\n";
		QTest::newRow("section twice, other case") << "[ai]\ncustom_model=a\n[AI]\ncustom_model=b\n" << "[ai]\ncustom_model=a\n[AI]\ncustom_model=new\n";
		QTest::newRow("key twice") << "[ai]\ncustom_model=a\ncustom_model=b\n" << "[ai]\ncustom_model=a\ncustom_model=new\n";
	}
	void setterFindsItsPlace() {
		QFETCH(QString, before);
		writeSettingsFile(before);
		AppSettings::instance().setAiCustomModel(QStringLiteral("new"));
		QTEST(readSettingsFile(), "after");
		QCOMPARE(AppSettings::instance().aiCustomModel(), QStringLiteral("new"));
	}

	// A setter keeps the file's line endings and text other than ASCII, and
	// ends the file with a line end; a file it creates has Windows ones.
	void setterKeepsTheFilesText() {
		AppSettings& s = AppSettings::instance();
		writeBytes("; caf\xc3\xa9\r\n[ai]\r\ncustom_model=a\r\n\r\n[x]\r\ny=\xe2\x82\xac");
		s.setAiCustomModel(QStringLiteral("new"));
		QCOMPARE(readBytes(), QByteArray("; caf\xc3\xa9\r\n[ai]\r\ncustom_model=new\r\n\r\n[x]\r\ny=\xe2\x82\xac\r\n"));
		writeBytes("[ai]\ncustom_model=a\n\n\n");
		s.setAiCustomModel(QStringLiteral("new"));
		QCOMPARE(readBytes(), QByteArray("[ai]\ncustom_model=new\n\n\n"));
		writeBytes("[x]\r\ny=1\r\n");
		s.setRecordingEnabled(false);
		QVERIFY(readBytes().startsWith("[x]\r\ny=1\r\n\r\n[recording]\r\n;"));
		QVERIFY(readBytes().endsWith("\r\nenabled=false\r\n"));
		removeSettings();
		s.setRecordingEnabled(false);
		QVERIFY(readBytes().startsWith("[recording]\r\n;"));
		QCOMPARE(readBytes().count("\r\n"), readBytes().count('\n'));
	}

	// A file that isn't UTF-8, whose other text saving would change, isn't saved
	// to; one that can't be replaced keeps its text.
	void setterLeavesAFileItCantSafelyChange() {
		AppSettings& s = AppSettings::instance();
		for (const QByteArray latin1 : { "; caf\xe9\n[ai]\ncustom_model=a\n", "[ai]\ncustom_model=a\n[x]\ny=caf\xe9" }) {
			writeBytes(latin1);
			s.setAiCustomModel(QStringLiteral("new"));
			QCOMPARE(readBytes(), latin1);
		}

		const QByteArray bytes("[ai]\ncustom_model=a\n");
		writeBytes(bytes);
		HANDLE lock = CreateFileW(reinterpret_cast<LPCWSTR>(QDir::toNativeSeparators(AppSettings::filePath()).utf16()),
			GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
		QVERIFY(lock != INVALID_HANDLE_VALUE);
		s.setAiCustomModel(QStringLiteral("new"));
		CloseHandle(lock);
		QCOMPARE(readBytes(), bytes);
	}

	// A setting added to a section ending in commented-out ones goes after them.
	void setterAddsAMissingKeyAfterCommentedOutOnes() {
		writeSettingsFile("[ai]\ngemini_api_key=k\n\n; note\n;claude_api_key=\n\n; about other\n[other]\nkeep=me\n");
		AppSettings::instance().setAiCustomModel(QStringLiteral("gemma-x"));
		QCOMPARE(readSettingsFile(), QStringLiteral("[ai]\ngemini_api_key=k\n\n; note\n;claude_api_key=\n\n"
			"; Auto-managed by the app. The model name entered for the drop-down's Other\n"
			"; choice (model=other).\ncustom_model=gemma-x\n\n; about other\n[other]\nkeep=me\n"));
	}

	// With exactly one API key, a saved model or custom model that its name
	// shows to be another provider's is cleared, and model=other with it when
	// the custom model is; anything else is kept.
	void switchingProviderClearsTheOldOnesModels_data() {
		QTest::addColumn<QString>("keys");
		QTest::addColumn<QString>("model");
		QTest::addColumn<QString>("custom");
		QTest::addColumn<QString>("modelAfter");
		QTest::addColumn<QString>("customAfter");
		const QString gemini("gemini_api_key=k\nopenai_api_key=\nclaude_api_key=\n");
		const QString openai("gemini_api_key=\nopenai_api_key=k\n;claude_api_key=spare\n");
		const QString claude("gemini_api_key= \nopenai_api_key=\nclaude_api_key=k\n");
		const QString none("gemini_api_key=\n");
		const QString two("gemini_api_key=k\nopenai_api_key=k\n");
		const QString empty;
		QTest::newRow("same provider") << gemini << "gemma-4-31b-it" << "gemini-3.1-flash-lite" << "gemma-4-31b-it" << "gemini-3.1-flash-lite";
		QTest::newRow("to OpenAI") << openai << "gemma-4-31b-it" << "gemini-3.1-flash-lite" << empty << empty;
		QTest::newRow("to Claude") << claude << "gpt-6.2-sol" << "o3" << empty << empty;
		QTest::newRow("to Gemini") << gemini << "claude-sonnet-5-5" << "chatgpt-x" << empty << empty;
		QTest::newRow("OpenAI's other names") << claude << "chat-latest" << "ft:gpt-5:me" << empty << empty;
		QTest::newRow("Claude kept") << claude << "claude-sonnet-5-5" << "claude-opus-5-5" << "claude-sonnet-5-5" << "claude-opus-5-5";
		QTest::newRow("OpenAI kept") << openai << "gpt-6.2-sol" << "o3" << "gpt-6.2-sol" << "o3";
		QTest::newRow("other, cleared") << openai << "other" << "gemini-3.1-flash-lite" << empty << empty;
		QTest::newRow("other, kept") << openai << "other" << "gpt-5.5" << "other" << "gpt-5.5";
		QTest::newRow("model only") << openai << "gemma-4-31b-it" << "gpt-5.5" << empty << "gpt-5.5";
		QTest::newRow("custom only") << openai << "gpt-6.2-sol" << "claude-opus-5-5" << "gpt-6.2-sol" << empty;
		QTest::newRow("unknown name") << openai << "other" << "mystery-1" << "other" << "mystery-1";
		QTest::newRow("other with Claude's") << claude << "other" << "claude-opus-5-5" << "other" << "claude-opus-5-5";
		QTest::newRow("OpenAI's name inside another") << claude << "x-gpt-4" << "x-o3" << "x-gpt-4" << "x-o3";
		QTest::newRow("Gemini's and Claude's inside others") << openai << "x-gemini-1" << "x-claude-1" << "x-gemini-1" << "x-claude-1";
		QTest::newRow("a quoted blank is no key") << QString("gemini_api_key=\" \"\nopenai_api_key=k\n") << "gemma-4-31b-it" << empty << empty << empty;
		QTest::newRow("no key") << none << "gpt-6.2-sol" << "claude-opus-5-5" << "gpt-6.2-sol" << "claude-opus-5-5";
		QTest::newRow("two keys") << two << "claude-sonnet-5-5" << "claude-opus-5-5" << "claude-sonnet-5-5" << "claude-opus-5-5";
	}

	void switchingProviderClearsTheOldOnesModels() {
		QFETCH(QString, keys);
		QFETCH(QString, model);
		QFETCH(QString, custom);
		writeSettingsFile(QStringLiteral("[ai]\n") + keys + QStringLiteral("model=") + model + QStringLiteral("\ncustom_model=") + custom + '\n');
		AppSettings::prepareFile();
		QTEST(AppSettings::instance().aiModel(), "modelAfter");
		QTEST(AppSettings::instance().aiCustomModel(), "customAfter");
	}

	void defaultFileReadsAsDefaults() {
		removeSettings();
		AppSettings::prepareFile();
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.dataTableHiddenFields(), QStringList());
		QCOMPARE(s.dataTableFieldColumnWidth(), 140);
		QCOMPARE(s.rightPanelWidth(), 260);
		QCOMPARE(s.chartsPanelHeight(), 400);
		QVERIFY(s.tripHistoryColumnWidths().isEmpty());
		QCOMPARE(s.geminiApiKey(), QString());
		QCOMPARE(s.openaiApiKey(), QString());
		QCOMPARE(s.claudeApiKey(), QString());
		QCOMPARE(s.aiModel(), QString());
		QCOMPARE(s.aiCustomModel(), QString());
		QCOMPARE(s.aiProgressEvaluation(), true);
		QVERIFY(readSettingsFile().contains(QStringLiteral("\nprogress_evaluation=true\n")));
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
		QCOMPARE(s.aiModel(), QString());
		QCOMPARE(s.aiProgressEvaluation(), true);
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
		writeSettingsFile("[ai]\ngemini_api_key=KEY123\nopenai_api_key=KEY456\nclaude_api_key=KEY789\n[recording]\nsample_interval_ms=250\nenabled=false\n");
		AppSettings& s = AppSettings::instance();
		QCOMPARE(s.geminiApiKey(), QStringLiteral("KEY123"));
		QCOMPARE(s.openaiApiKey(), QStringLiteral("KEY456"));
		QCOMPARE(s.claudeApiKey(), QStringLiteral("KEY789"));
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

	// Each setter adds its missing section under the same header block the
	// default file has (see firstUseCreatesTheDefaultFile).
	void setterAddsAMissingSectionUnderItsDefaultHeader() {
		AppSettings& s = AppSettings::instance();
		const QString autoManaged = QStringLiteral("; Auto-managed by the app.\n");
		const QString columnWidths = QStringLiteral("; Auto-managed by the app. Persisted column widths for the tables in the\n"
			"; UI that support user resizing.\n");
		const std::vector<std::pair<std::function<void()>, QString>> cases = {
			{ [&s] { s.setRightPanelWidth(300); }, autoManaged + "[layout]\n" },
			{ [&s] { s.setChartsPanelHeight(500); }, autoManaged + "[layout]\n" },
			{ [&s] { s.setDataTableHiddenFields({ "G Force" }); }, autoManaged + "[data_table]\n" },
			{ [&s] { s.setDataTableFieldColumnWidth(180); }, columnWidths + "[table_column_width]\n" },
			{ [&s] { s.setTripHistoryColumnWidths({ { "TitleColumn", 120 } }); }, columnWidths + "[table_column_width]\n" },
			{ [&s] { s.setRecordingEnabled(false); }, QStringLiteral("[recording]\n") },
			{ [&s] { s.setAiModel(QStringLiteral("other")); }, kAiHeader },
			{ [&s] { s.setAiCustomModel(QStringLiteral("gemini-3-pro")); }, kAiHeader },
			{ [&s] { s.setAiProgressEvaluation(false); }, kAiHeader },
		};
		for (const auto& [set, header] : cases) {
			removeSettings();
			set();
			const QString text = readSettingsFile();
			QVERIFY2(text.startsWith(header), qPrintable(text));
		}
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
		const QByteArray bytes("[layout]\nright_panel_width=260\n[other]\nkeep=me\n");
		writeBytes(bytes);
		// Reading is denied, but the file could still be replaced.
		const QString path = QDir::toNativeSeparators(AppSettings::filePath());
		QCOMPARE(QProcess::execute("icacls", { path, "/deny", "*S-1-1-0:(RD)" }), 0);
		const bool unreadable = !QFile(path).open(QIODevice::ReadOnly);
		AppSettings::instance().setRightPanelWidth(300);
		QCOMPARE(QProcess::execute("icacls", { path, "/remove:d", "*S-1-1-0" }), 0);
		QVERIFY(unreadable);
		QCOMPARE(readBytes(), bytes);
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

	// The model choice is saved into [ai] next to the key, each with its
	// comment when the key is new, and read back verbatim.
	void aiModelRoundTrip() {
		writeSettingsFile("[ai]\ngemini_api_key=KEY123\n\n[other]\nkeep=me\n");
		AppSettings& s = AppSettings::instance();
		s.setAiModel(QStringLiteral("other"));
		s.setAiCustomModel(QStringLiteral("gemini-3-pro"));
		const QString text = readSettingsFile();
		QVERIFY2(text.startsWith(QStringLiteral("[ai]\ngemini_api_key=KEY123\n")), qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("; Auto-managed by the app. The model the analysis uses")), qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("; Auto-managed by the app. The model name entered for the drop-down's Other")), qPrintable(text));
		QVERIFY2(text.indexOf(QStringLiteral("custom_model=gemini-3-pro\n")) < text.indexOf(QStringLiteral("[other]")), qPrintable(text));
		QVERIFY2(text.endsWith(QStringLiteral("[other]\nkeep=me\n")), qPrintable(text));
		QCOMPARE(s.aiModel(), QStringLiteral("other"));
		QCOMPARE(s.aiCustomModel(), QStringLiteral("gemini-3-pro"));
		s.setAiModel(QStringLiteral("gemma-4-26b-a4b-it"));
		QCOMPARE(s.aiModel(), QStringLiteral("gemma-4-26b-a4b-it"));
		QCOMPARE(s.aiCustomModel(), QStringLiteral("gemini-3-pro"));
	}

	// The evaluation choice is saved into [ai] with its comment when the key
	// is new, and read back.
	void aiProgressEvaluationRoundTrip() {
		writeSettingsFile("[ai]\ngemini_api_key=KEY123\n");
		AppSettings& s = AppSettings::instance();
		s.setAiProgressEvaluation(false);
		const QString text = readSettingsFile();
		QVERIFY2(text.contains(QStringLiteral("; Auto-managed by the app. Whether the AI analysis is a progress evaluation\n")),
			qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("\nprogress_evaluation=false\n")), qPrintable(text));
		QCOMPARE(s.aiProgressEvaluation(), false);
		s.setAiProgressEvaluation(true);
		QCOMPARE(s.aiProgressEvaluation(), true);
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

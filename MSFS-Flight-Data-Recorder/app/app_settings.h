#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

// Default width of the right-side panels (live status and data table) --
// rightPanelWidth() when unset -- and also the widest they can be dragged.
inline constexpr int kRightPanelWidth = 260;

// Thin QSettings wrapper -- an INI file in the app's folder (app_file_path(),
// alongside flight_data.db), not the registry, so settings travel with a
// portable install.
class AppSettings {
public:
	static AppSettings& instance();

	// Path of settings.ini: the current working directory in Debug builds,
	// the executable's directory in Release builds.
	static QString filePath();

	// Writes a fully-documented default settings.ini when there's none. In an
	// existing file, only brings the comments directly above its settings and
	// sections, and at its top, up to date, adds [ai]'s missing API keys and
	// comments out its empty ones, keeping the previous file as
	// settings.ini.bak; and then, with exactly one API key set, clears a saved
	// model and custom model that their names show to be another provider's (and
	// model=other with the custom model), unless the refresh left the file as it
	// is because it couldn't safely update it (refreshComments() returning false).
	// instance() calls it the first time it's used in a process.
	static void prepareFile();

	// [logging] verbose: the log level name (Logger::levelFromString()),
	// "INFO" if unset. Static and read-only (never creates settings.ini), so
	// main() can read it before QApplication exists and logging starts.
	static QString logLevel();

	// Field names (DataTablePanel's row labels) the user has unchecked in the
	// Visible Fields dialog -- everything not listed here stays visible.
	QStringList dataTableHiddenFields() const;
	void setDataTableHiddenFields(const QStringList& fields);

	// Width in pixels of the Data Table panel's "Field" column (the Value
	// column always stretches to fill the rest).
	int dataTableFieldColumnWidth() const;
	void setDataTableFieldColumnWidth(int w);

	// Width of the right-side panels (live status and data table). Both columns
	// share one value so resizing either one persists for both.
	int rightPanelWidth() const;
	void setRightPanelWidth(int w);

	// Height in pixels of the charts panel (below the map). The map takes the
	// remaining vertical space.
	int chartsPanelHeight() const;
	void setChartsPanelHeight(int h);

	// Column widths (pixels) of the Trip History table, keyed by
	// TripHistoryModel::Column's enum member name (via Qt's Q_ENUM
	// reflection) rather than ordinal position, so a column being inserted,
	// removed, or reordered later can't silently misassign a saved width
	// onto the wrong column -- an unrecognized or missing key just falls
	// back to that column's coded default. Columns using Stretch sizing are
	// never stored.
	QMap<QString, int> tripHistoryColumnWidths() const;
	void setTripHistoryColumnWidths(const QMap<QString, int>& widths);

	// API keys of the liftoff/landing analysis feature in the map popup's
	// providers, set manually under [ai] in settings.ini (gemini_api_key,
	// openai_api_key, claude_api_key). The page (map.html) analyzes only with
	// exactly one of them set, which decides the provider.
	QString geminiApiKey() const;
	QString openaiApiKey() const;
	QString claudeApiKey() const;

	// The AI analysis model as the map popup's drop-down left it, under [ai]
	// in settings.ini: model is a model's name or other (the name in
	// custom_model). Read verbatim: the page (map.html) owns the choices and
	// treats an empty or unknown model as the provider's default.
	QString aiModel() const;
	void setAiModel(const QString& model);
	QString aiCustomModel() const;
	void setAiCustomModel(const QString& name);
	// Whether the AI analysis is a progress evaluation (with the trajectory)
	// or a contact evaluation, as the popup's checkbox left it: [ai]
	// progress_evaluation in settings.ini. Default: true.
	bool aiProgressEvaluation() const;
	void setAiProgressEvaluation(bool progress);

	// Maximum milliseconds between telemetry samples written to trip_data.
	// Read from [recording] sample_interval_ms in settings.ini.
	// If unset, empty, or not a positive integer, defaults to 500.
	int sampleIntervalMs() const;

	// Whether automatic recording is allowed to start. Toggled via the
	// Recording indicator in the Live Status panel (a no-op while a trip is
	// already recording); disabling it only prevents a *new* trip from
	// starting, it doesn't stop one in progress. Read from [recording]
	// enabled in settings.ini. Default: true.
	bool recordingEnabled() const;
	void setRecordingEnabled(bool enabled);

private:
	AppSettings() = default;
};

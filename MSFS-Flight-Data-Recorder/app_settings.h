#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

// Thin QSettings wrapper -- an INI file next to the executable (alongside
// flight_data.db), not the registry, so settings travel with a portable
// install.
class AppSettings {
public:
	static AppSettings& instance();

	// Field names (DataTablePanel's row labels) the user has unchecked in the
	// "Fields…" dialog -- everything not listed here stays visible.
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

	// Gemini API key used by the liftoff/landing analysis feature in the map popup.
	// Set manually under [ai] gemini_api_key in settings.ini.
	QString geminiApiKey() const;
	void setGeminiApiKey(const QString& key);

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

	// Log verbosity level written to [logging] verbose in settings.ini.
	// Valid values: "FATAL", "WARNING", "INFO" (default), "TRACE", "PROFILE".
	// INFO includes user-visible events; TRACE also writes fine-grained
	// diagnostic detail; PROFILE also writes timing breakdowns.
	QString verboseLevel() const;
	void setVerboseLevel(const QString& level);

private:
	AppSettings() = default;
};

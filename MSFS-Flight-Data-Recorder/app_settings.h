#pragma once

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

	// Width of the right-side panels (live status and data table). Both columns
	// share one value so resizing either one persists for both.
	int rightPanelWidth() const;
	void setRightPanelWidth(int w);

	// Height in pixels of the charts panel (below the map). The map takes the
	// remaining vertical space.
	int chartsPanelHeight() const;
	void setChartsPanelHeight(int h);

	// Gemini API key used by the touchdown analysis feature in the map popup.
	// Set manually under [ai] gemini_api_key in settings.ini.
	QString geminiApiKey() const;
	void setGeminiApiKey(const QString& key);

	// Maximum milliseconds between telemetry samples written to trip_data.
	// Read from [recording] sample_interval_ms in settings.ini.
	// If unset, empty, or not a positive integer, defaults to 500.
	int sampleIntervalMs() const;

	// Log verbosity level written to [logging] verbose in settings.ini.
	// Valid values: "FATAL", "WARNING", "INFO" (default), "PROFILE".
	// INFO includes user-visible events; PROFILE also writes timing breakdowns.
	QString verboseLevel() const;
	void setVerboseLevel(const QString& level);

private:
	AppSettings() = default;
};

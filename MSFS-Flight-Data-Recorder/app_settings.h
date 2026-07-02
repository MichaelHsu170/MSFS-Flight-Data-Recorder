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

	// Splitter states (QSplitter::saveState / restoreState byte arrays).
	QByteArray topSplitterState() const;
	void setTopSplitterState(const QByteArray& state);
	QByteArray trajectoryDataTableSplitterState() const;
	void setTrajectoryDataTableSplitterState(const QByteArray& state);
	QByteArray trajectoryMapChartsSplitterState() const;
	void setTrajectoryMapChartsSplitterState(const QByteArray& state);

private:
	AppSettings() = default;
};

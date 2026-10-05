#include "map_bridge.h"
#include "db_connection.h"
#include "db_history.h"
#include "logger.h"

MapBridge::MapBridge(QObject* parent) : QObject(parent) {}

void MapBridge::markerMoved(int index, int version) {
	emit cursorIndexChanged(index, version);
}

void MapBridge::rangeChanged(int startIndex, int endIndex, int version) {
	emit visibleRangeChanged(startIndex, endIndex, version);
}

void MapBridge::overviewSegmentClicked(int tripId) {
	emit overviewTripClicked(tripId);
}

bool MapBridge::saveLiftoffAnalysisReport(int rowId, const QString& report) {
	return saveReport(CONTACT_TABLE::LIFTOFFS, rowId, report);
}

bool MapBridge::saveTouchdownAnalysisReport(int rowId, const QString& report) {
	return saveReport(CONTACT_TABLE::TOUCHDOWNS, rowId, report);
}

bool MapBridge::saveReport(CONTACT_TABLE table, int rowId, const QString& report) {
	DbConnection sql = DbConnection::readWrite();
	if (!sql) {
		Logger::logf(Logger::Warning, "DB", "saveAnalysisReport(%s %d): failed to open read-write connection; analysis was not saved",
			contactLabel(table), rowId);
		return false;
	}
	return saveAnalysisReport(sql.get(), table, rowId, report);
}

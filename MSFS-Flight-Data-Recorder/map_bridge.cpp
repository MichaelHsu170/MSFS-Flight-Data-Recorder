#include "map_bridge.h"
#include "db.h"
#include "logger.h"
#include "sqlite3.h"

MapBridge::MapBridge(QObject* parent) : QObject(parent) {}

void MapBridge::markerMoved(int index) {
	emit cursorIndexChanged(index);
}

void MapBridge::rangeChanged(int startIndex, int endIndex) {
	emit visibleRangeChanged(startIndex, endIndex);
}

void MapBridge::overviewSegmentClicked(int tripId) {
	emit overviewTripClicked(tripId);
}

void MapBridge::saveLiftoffAnalysisReport(int rowId, const QString& report) {
	if (rowId <= 0) {
		Logger::logf(Logger::Trace, "DB", "saveLiftoffAnalysisReport: ignoring invalid liftoff id %d", rowId);
		return;
	}
	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::logf(Logger::Warning, "DB", "saveLiftoffAnalysisReport(liftoff %d): failed to open read-write connection; analysis was not saved", rowId);
		return;
	}
	const char* q = "UPDATE trip_liftoffs SET analysis_report = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, q, -1, &stmt, nullptr) == SQLITE_OK) {
		QByteArray utf8 = report.toUtf8();
		sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
		sqlite3_bind_int(stmt, 2, rowId);
		if (sqlite3_step(stmt) != SQLITE_DONE)
			Logger::logf(Logger::Warning, "DB", "saveLiftoffAnalysisReport(liftoff %d): update failed: %s", rowId, sqlite3_errmsg(sql));
		else
			Logger::logf(Logger::Trace, "DB", "saveLiftoffAnalysisReport(liftoff %d): analysis report saved (%d chars)", rowId, utf8.size());
		sqlite3_finalize(stmt);
	} else {
		Logger::logf(Logger::Warning, "DB", "saveLiftoffAnalysisReport(liftoff %d): prepare failed: %s", rowId, sqlite3_errmsg(sql));
	}
	sqlite3_close(sql);
}

void MapBridge::saveTouchdownAnalysisReport(int rowId, const QString& report) {
	if (rowId <= 0) {
		Logger::logf(Logger::Trace, "DB", "saveTouchdownAnalysisReport: ignoring invalid touchdown id %d", rowId);
		return;
	}
	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::logf(Logger::Warning, "DB", "saveTouchdownAnalysisReport(touchdown %d): failed to open read-write connection; analysis was not saved", rowId);
		return;
	}
	const char* q = "UPDATE trip_touchdowns SET analysis_report = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, q, -1, &stmt, nullptr) == SQLITE_OK) {
		QByteArray utf8 = report.toUtf8();
		sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
		sqlite3_bind_int(stmt, 2, rowId);
		if (sqlite3_step(stmt) != SQLITE_DONE)
			Logger::logf(Logger::Warning, "DB", "saveTouchdownAnalysisReport(touchdown %d): update failed: %s", rowId, sqlite3_errmsg(sql));
		else
			Logger::logf(Logger::Trace, "DB", "saveTouchdownAnalysisReport(touchdown %d): analysis report saved (%d chars)", rowId, utf8.size());
		sqlite3_finalize(stmt);
	} else {
		Logger::logf(Logger::Warning, "DB", "saveTouchdownAnalysisReport(touchdown %d): prepare failed: %s", rowId, sqlite3_errmsg(sql));
	}
	sqlite3_close(sql);
}

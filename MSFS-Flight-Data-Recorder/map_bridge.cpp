#include "map_bridge.h"
#include "db.h"
#include "sqlite3.h"

MapBridge::MapBridge(QObject* parent) : QObject(parent) {}

void MapBridge::markerMoved(int index) {
	emit cursorIndexChanged(index);
}

void MapBridge::rangeChanged(int startIndex, int endIndex) {
	emit visibleRangeChanged(startIndex, endIndex);
}

void MapBridge::saveAnalysisReport(int rowId, const QString& report) {
	if (rowId <= 0) return;
	sqlite3* sql = connect_db_readwrite();
	if (!sql) return;
	const char* q = "UPDATE trip_touchdowns SET analysis_report = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, q, -1, &stmt, nullptr) == SQLITE_OK) {
		QByteArray utf8 = report.toUtf8();
		sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
		sqlite3_bind_int(stmt, 2, rowId);
		sqlite3_step(stmt);
		sqlite3_finalize(stmt);
	}
	sqlite3_close(sql);
}

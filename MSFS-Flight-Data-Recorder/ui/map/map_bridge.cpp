#include "map_bridge.h"
#include "contact_analysis.h"
#include "db_connection.h"
#include "db_history.h"
#include "logger.h"
#include "map_script.h"

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

namespace {

// contacts' one with rowId's trajectory as JSON; null without one.
template <typename Contact>
QVariant profileJson(const TripDataset& dataset, const std::vector<Contact>& contacts, int rowId, bool touchdown) {
	for (const Contact& contact : contacts) {
		if (contact.rowId != rowId)
			continue;
		const std::optional<ContactProfile> profile = contactProfile(dataset, contact, touchdown);
		return profile ? QVariant(mapProfileJson(*profile).toVariantMap()) : QVariant();
	}
	return QVariant();
}

}

QVariant MapBridge::liftoffProfile(int rowId) const {
	return dataset_ ? profileJson(*dataset_, dataset_->liftoffPoints, rowId, false) : QVariant();
}

QVariant MapBridge::touchdownProfile(int rowId) const {
	return dataset_ ? profileJson(*dataset_, dataset_->touchdowns, rowId, true) : QVariant();
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

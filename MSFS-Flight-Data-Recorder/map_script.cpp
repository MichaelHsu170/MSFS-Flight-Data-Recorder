#include "map_script.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace {

QString call(const char* function, const QJsonDocument& argument) {
	return QStringLiteral("%1(%2);").arg(QString::fromLatin1(function),
		QString::fromUtf8(argument.toJson(QJsonDocument::Compact)));
}

// The fields liftoff and touchdown markers share (their popups show the same
// runway/attitude/wind details).
template <typename Point>
QJsonObject runwayContactToJson(const Point& t) {
	QJsonObject obj;
	obj["lat"] = t.latitude;
	obj["lng"] = t.longitude;
	obj["icao"] = t.icao;
	obj["airportName"] = t.airportName;
	obj["runway"] = t.runway;
	obj["runwayHeading"] = t.runwayHeading;
	obj["airspeed"] = t.airspeed;
	obj["verticalSpeed"] = t.verticalSpeed;
	obj["pitchDegrees"] = t.pitchDegrees;
	obj["bankDegrees"] = t.bankDegrees;
	obj["headingDegrees"] = t.headingDegrees;
	obj["distanceLength"] = t.distanceLength;
	obj["distanceWidth"] = t.distanceWidth;
	obj["distanceLengthPercent"] = t.distanceLengthPercent;
	obj["distanceWidthPercent"] = t.distanceWidthPercent;
	obj["windDirection"] = t.windDirection;
	obj["windVelocity"] = t.windVelocity;
	obj["zuluTime"] = t.zuluTime;
	obj["localTime"] = t.localTime;
	obj["rowId"] = t.rowId;
	obj["analysisReport"] = t.analysisReport;
	return obj;
}

}

QString mapSetStringJs(const QString& jsVariable, const QString& value) {
	// Encoded inside a one-element array: a bare JSON string isn't a valid
	// QJsonDocument.
	QJsonArray arr;
	arr.append(QJsonValue(value));
	return QStringLiteral("%1=%2[0];").arg(jsVariable,
		QString::fromUtf8(QJsonDocument(arr).toJson(QJsonDocument::Compact)));
}

QString mapSetTrajectoryJs(const std::vector<std::pair<double, double>>& coords) {
	QJsonArray lats, lngs, idxs;
	for (int i : decimatedIndices(0, (int)coords.size() - 1, MAP_MAX_TRAJECTORY_POINTS)) {
		lats.append(coords[i].first);
		lngs.append(coords[i].second);
		idxs.append(i);
	}
	QJsonObject data;
	data[QStringLiteral("lats")] = lats;
	data[QStringLiteral("lngs")] = lngs;
	data[QStringLiteral("idxs")] = idxs;
	return call("setTrajectory", QJsonDocument(data));
}

QString mapAppendPointsJs(const std::vector<std::pair<double, double>>& coords) {
	QJsonArray pts;
	for (const auto& [lat, lng] : coords) {
		QJsonObject obj;
		obj["lat"] = lat;
		obj["lng"] = lng;
		pts.append(obj);
	}
	return call("appendPoints", QJsonDocument(pts));
}

QString mapSetLiftoffsJs(const std::vector<LiftoffPoint>& liftoffs) {
	QJsonArray arr;
	for (const LiftoffPoint& t : liftoffs)
		arr.append(runwayContactToJson(t));
	return call("setLiftoffs", QJsonDocument(arr));
}

QString mapSetTouchdownsJs(const std::vector<TouchdownPoint>& touchdowns) {
	QJsonArray arr;
	for (const TouchdownPoint& t : touchdowns) {
		QJsonObject obj = runwayContactToJson(t);
		obj["gForce"] = t.gForce;
		arr.append(obj);
	}
	return call("setTouchdowns", QJsonDocument(arr));
}

QString mapSetEventsJs(const std::vector<TripEvent>& events) {
	QJsonArray arr;
	for (const TripEvent& e : events) {
		QJsonObject obj;
		obj["lat"] = e.latitude;
		obj["lng"] = e.longitude;
		obj["event"] = e.event;
		obj["zuluTime"] = e.zuluTime;
		arr.append(obj);
	}
	return call("setEvents", QJsonDocument(arr));
}

QString mapSetEventsVisibleJs(bool visible) {
	return visible ? QStringLiteral("setEventsVisible(true);") : QStringLiteral("setEventsVisible(false);");
}

QString mapSetOverviewJs(const std::vector<TripSummary>& trips) {
	QJsonArray segments;
	for (const TripSummary& t : trips) {
		QJsonObject seg;
		seg[QStringLiteral("fromLat")] = t.departureLat;
		seg[QStringLiteral("fromLng")] = t.departureLng;
		seg[QStringLiteral("toLat")]   = t.destinationLat;
		seg[QStringLiteral("toLng")]   = t.destinationLng;
		seg[QStringLiteral("groupId")]   = t.groupId;
		seg[QStringLiteral("groupName")] = t.groupId != 0 ? t.groupName : QStringLiteral("Ungrouped");
		seg[QStringLiteral("groupRank")] = t.groupRank;
		seg[QStringLiteral("tripId")]  = t.id;
		seg[QStringLiteral("fromIcao")] = t.departureIcao;
		seg[QStringLiteral("toIcao")]   = t.destinationIcao;
		segments.append(seg);
	}
	return call("setOverview", QJsonDocument(segments));
}

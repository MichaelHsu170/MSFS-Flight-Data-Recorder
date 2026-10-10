#include "map_script.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>

namespace {

QString call(const char* function, const QJsonDocument& argument) {
	return QStringLiteral("%1(%2);").arg(QString::fromLatin1(function),
		QString::fromUtf8(argument.toJson(QJsonDocument::Compact)));
}

// Sets obj[key] to value unless it's NaN (not recorded): map.html leaves an
// absent value out of the popup and the prompt instead of showing 0.
void setKnown(QJsonObject& obj, const char* key, double value) {
	if (!std::isnan(value))
		obj[QLatin1String(key)] = value;
}

void setKnown(QJsonObject& obj, const char* key, const QString& value) {
	if (!value.isEmpty())
		obj[QLatin1String(key)] = value;
}

QJsonObject conditionsToJson(const ContactConditions& c) {
	QJsonObject obj;
	setKnown(obj, "headingTrue", c.headingTrue);
	setKnown(obj, "headingMagnetic", c.headingMagnetic);
	setKnown(obj, "trackTrue", c.trackTrue);
	setKnown(obj, "magneticVariation", c.magneticVariation);
	setKnown(obj, "crabAngle", c.crabAngle);
	setKnown(obj, "sunElevation", c.sunElevation);
	setKnown(obj, "temperatureC", c.temperatureC);
	setKnown(obj, "stationPressureHg", c.stationPressureHg);
	setKnown(obj, "altimeterHg", c.altimeterHg);
	setKnown(obj, "altimeterMb", c.altimeterMb);
	setKnown(obj, "pressureAltitudeFt", c.pressureAltitudeFt);
	setKnown(obj, "densityAltitudeFt", c.densityAltitudeFt);
	setKnown(obj, "visibilityM", c.visibilityM);
	setKnown(obj, "seaLevelPressureMb", c.seaLevelPressureMb);
	setKnown(obj, "precipitation", c.precipitation);
	setKnown(obj, "precipitationRate", c.precipitationRate);
	setKnown(obj, "inCloud", c.inCloud);
	setKnown(obj, "fieldElevationFt", c.fieldElevationFt);
	setKnown(obj, "windMinKt", c.windMinKt);
	setKnown(obj, "windMaxKt", c.windMaxKt);
	setKnown(obj, "windDirectionFrom", c.windDirectionFrom);
	setKnown(obj, "windDirectionTo", c.windDirectionTo);
	setKnown(obj, "surfaceType", c.surfaceType);
	setKnown(obj, "surfaceCondition", c.surfaceCondition);
	return obj;
}

QJsonArray profilePointsToJson(const std::vector<ProfilePoint>& points) {
	QJsonArray arr;
	for (const ProfilePoint& p : points) {
		QJsonObject obj;
		setKnown(obj, "gate", p.gate);
		obj["seconds"] = p.seconds;
		setKnown(obj, "heightFt", p.heightFt);
		setKnown(obj, "distanceNm", p.distanceNm);
		setKnown(obj, "terrainFt", p.terrainFt);
		setKnown(obj, "airspeed", p.airspeed);
		setKnown(obj, "verticalSpeed", p.verticalSpeed);
		setKnown(obj, "pitch", p.pitch);
		setKnown(obj, "bank", p.bank);
		setKnown(obj, "headingTrue", p.headingTrue);
		setKnown(obj, "trackTrue", p.trackTrue);
		setKnown(obj, "thresholdFt", p.thresholdFt);
		setKnown(obj, "centerlineFt", p.centerlineFt);
		setKnown(obj, "pathAngle", p.pathAngle);
		arr.append(obj);
	}
	return arr;
}

QJsonObject profileToJson(const ContactProfile& profile) {
	QJsonObject obj;
	obj["gates"] = profilePointsToJson(profile.gates);
	obj["series"] = profilePointsToJson(profile.series);
	setKnown(obj, "highestTerrainFt", profile.highestTerrainFt);
	setKnown(obj, "highestTerrainNm", profile.highestTerrainNm);
	return obj;
}

// The fields liftoff and touchdown markers share (their popups show the same
// runway/attitude/wind details).
QJsonObject runwayContactToJson(const RunwayContactPoint& t) {
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
	// The used runway end's facility data, each absent when not stored.
	setKnown(obj, "runwayLengthFt", t.runwayData.lengthFt);
	setKnown(obj, "runwayWidthFt", t.runwayData.widthFt);
	setKnown(obj, "thresholdOffsetFt", t.runwayData.thresholdOffsetFt);
	setKnown(obj, "runwaySlope", t.runwayData.slopeDegrees);
	// Without a sample at the contact there's nothing to derive: both absent.
	if (t.conditions.fromSample) {
		obj["conditions"] = conditionsToJson(t.conditions);
		obj["profile"] = profileToJson(t.profile);
	}
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

QString mapSetTrajectoryJs(const std::vector<std::pair<double, double>>& coords, int version) {
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
	data[QStringLiteral("version")] = version;
	return call("setTrajectory", QJsonDocument(data));
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

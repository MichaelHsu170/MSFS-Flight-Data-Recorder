// Map script module (map_script.cpp) on its own: made-up trip data in, the
// JavaScript calls map.html receives out.
#include "map_script.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtTest>

namespace {

// The JSON argument of a "function(<json>);" call, after checking the shape.
QJsonDocument argumentOf(const QString& js, const QString& function) {
	const QString prefix = function + QStringLiteral("(");
	if (!js.startsWith(prefix) || !js.endsWith(QStringLiteral(");")))
		return {};
	return QJsonDocument::fromJson(js.mid(prefix.size(), js.size() - prefix.size() - 2).toUtf8());
}

std::vector<std::pair<double, double>> line(int count) {
	std::vector<std::pair<double, double>> coords;
	for (int i = 0; i < count; ++i)
		coords.emplace_back(40.0 + i * 0.001, -70.0 - i * 0.001);
	return coords;
}

template <typename Point>
void fillRunwayContact(Point& p) {
	p.latitude = 47.5;
	p.longitude = 8.5;
	p.icao = QStringLiteral("LSZH");
	p.airportName = QStringLiteral("Zürich \"Kloten\"");
	p.runway = QStringLiteral("16");
	p.runwayHeading = 155;
	p.airspeed = 140;
	p.verticalSpeed = -120;
	p.pitchDegrees = 4.5;
	p.bankDegrees = -1.25;
	p.headingDegrees = 154;
	p.distanceLength = 1200;
	p.distanceWidth = -3.5;
	p.distanceLengthPercent = 32.5;
	p.distanceWidthPercent = -5;
	p.windDirection = 200;
	p.windVelocity = 12;
	p.zuluTime = QStringLiteral("2026-03-04T10:00:00.000+00:00_3");
	p.localTime = QStringLiteral("2026-03-04T11:00:00.000+01:00_3");
	p.rowId = 42;
	p.analysisReport = QStringLiteral("report </script>");
}

void verifyRunwayContact(const QJsonObject& o) {
	QCOMPARE(o["lat"].toDouble(), 47.5);
	QCOMPARE(o["lng"].toDouble(), 8.5);
	QCOMPARE(o["icao"].toString(), QStringLiteral("LSZH"));
	QCOMPARE(o["airportName"].toString(), QStringLiteral("Zürich \"Kloten\""));
	QCOMPARE(o["runway"].toString(), QStringLiteral("16"));
	QCOMPARE(o["runwayHeading"].toDouble(), 155.0);
	QCOMPARE(o["airspeed"].toDouble(), 140.0);
	QCOMPARE(o["verticalSpeed"].toDouble(), -120.0);
	QCOMPARE(o["pitchDegrees"].toDouble(), 4.5);
	QCOMPARE(o["bankDegrees"].toDouble(), -1.25);
	QCOMPARE(o["headingDegrees"].toDouble(), 154.0);
	QCOMPARE(o["distanceLength"].toDouble(), 1200.0);
	QCOMPARE(o["distanceWidth"].toDouble(), -3.5);
	QCOMPARE(o["distanceLengthPercent"].toDouble(), 32.5);
	QCOMPARE(o["distanceWidthPercent"].toDouble(), -5.0);
	QCOMPARE(o["windDirection"].toDouble(), 200.0);
	QCOMPARE(o["windVelocity"].toDouble(), 12.0);
	QCOMPARE(o["zuluTime"].toString(), QStringLiteral("2026-03-04T10:00:00.000+00:00_3"));
	QCOMPARE(o["localTime"].toString(), QStringLiteral("2026-03-04T11:00:00.000+01:00_3"));
	QCOMPARE(o["rowId"].toInt(), 42);
	QCOMPARE(o["analysisReport"].toString(), QStringLiteral("report </script>"));
}

}

class TstMapScript : public QObject {
	Q_OBJECT

private slots:
	void setStringEscapesAnyText() {
		const QString value = QStringLiteral("a'b\"c\\d\ne</script>ü");
		const QString js = mapSetStringJs(QStringLiteral("window._x"), value);
		QVERIFY(js.startsWith(QStringLiteral("window._x=[")));
		QVERIFY(js.endsWith(QStringLiteral("][0];")));
		const QJsonDocument doc = QJsonDocument::fromJson(js.mid(10, js.size() - 10 - 4).toUtf8());
		QCOMPARE(doc.array().at(0).toString(), value);
		QVERIFY(!js.contains(QLatin1Char('\n')));
		QCOMPARE(mapSetStringJs(QStringLiteral("window._y"), QString()), QStringLiteral("window._y=[\"\"][0];"));
	}

	void shortTrajectoryIsSentWhole() {
		const QJsonObject data = argumentOf(mapSetTrajectoryJs(line(5), 9), QStringLiteral("setTrajectory")).object();
		QCOMPARE(data["version"].toInt(), 9);
		QCOMPARE(data["lats"].toArray().size(), 5);
		QCOMPARE(data["lngs"].toArray().size(), 5);
		QCOMPARE(data["idxs"].toArray(), (QJsonArray{ 0, 1, 2, 3, 4 }));
		QCOMPARE(data["lats"].toArray().at(3).toDouble(), 40.003);
		QCOMPARE(data["lngs"].toArray().at(3).toDouble(), -70.003);
	}

	void longTrajectoryIsThinnedKeepingIndicesAndEnds() {
		const auto coords = line(7001);
		const QJsonObject data = argumentOf(mapSetTrajectoryJs(coords, 1), QStringLiteral("setTrajectory")).object();
		const QJsonArray idxs = data["idxs"].toArray();
		const QJsonArray lats = data["lats"].toArray();
		QVERIFY(idxs.size() <= MAP_MAX_TRAJECTORY_POINTS + 1);
		QVERIFY(idxs.size() > MAP_MAX_TRAJECTORY_POINTS / 2);
		QCOMPARE(lats.size(), idxs.size());
		QCOMPARE(data["lngs"].toArray().size(), idxs.size());
		QCOMPARE(idxs.first().toInt(), 0);
		QCOMPARE(idxs.last().toInt(), 7000);
		// Each point is the sample its index names.
		for (int i = 0; i < idxs.size(); i += 250)
			QCOMPARE(lats.at(i).toDouble(), coords[idxs.at(i).toInt()].first);
	}

	void emptyTrajectory() {
		const QJsonObject data = argumentOf(mapSetTrajectoryJs({}, 1), QStringLiteral("setTrajectory")).object();
		QVERIFY(data.contains("lats"));
		QVERIFY(data["lats"].toArray().isEmpty());
		QVERIFY(data["idxs"].toArray().isEmpty());
	}

	void liftoffsCarryEveryPopupField() {
		LiftoffPoint lo;
		fillRunwayContact(lo);
		const QJsonArray arr = argumentOf(mapSetLiftoffsJs({ lo, lo }), QStringLiteral("setLiftoffs")).array();
		QCOMPARE(arr.size(), 2);
		const QJsonObject o = arr.at(0).toObject();
		verifyRunwayContact(o);
		QVERIFY(!o.contains("gForce"));
		QCOMPARE(o.size(), 21);
	}

	void touchdownsAlsoCarryGForce() {
		TouchdownPoint td;
		fillRunwayContact(td);
		td.gForce = 1.37;
		const QJsonArray arr = argumentOf(mapSetTouchdownsJs({ td }), QStringLiteral("setTouchdowns")).array();
		QCOMPARE(arr.size(), 1);
		const QJsonObject o = arr.at(0).toObject();
		verifyRunwayContact(o);
		QCOMPARE(o["gForce"].toDouble(), 1.37);
		QCOMPARE(o.size(), 22);
	}

	// A contact with a sample also carries its conditions, each value only
	// when known (no NaN reaches the page).
	void contactsWithASampleCarryConditions() {
		TouchdownPoint td;
		fillRunwayContact(td);
		td.conditions.fromSample = true;
		td.conditions.headingTrue = 268.5;
		td.conditions.crabAngle = -2.5;
		td.conditions.surfaceType = QStringLiteral("asphalt");
		td.conditions.inCloud = 0;  // known: "not in cloud" is sent, not left out
		td.conditions.precipitation = QStringLiteral("rain");
		td.conditions.seaLevelPressureMb = 1008;
		const QJsonObject o = argumentOf(mapSetTouchdownsJs({ td }), QStringLiteral("setTouchdowns")).array().at(0).toObject();
		QCOMPARE(o.size(), 23);

		const QJsonObject c = o["conditions"].toObject();
		QCOMPARE(c.keys(), QStringList({ "crabAngle", "headingTrue", "inCloud", "precipitation", "seaLevelPressureMb", "surfaceType" }));
		QCOMPARE(c["inCloud"].toDouble(), 0.0);
		QCOMPARE(c["precipitation"].toString(), QStringLiteral("rain"));
		QCOMPARE(c["seaLevelPressureMb"].toDouble(), 1008.0);
		QCOMPARE(c["headingTrue"].toDouble(), 268.5);
		QCOMPARE(c["crabAngle"].toDouble(), -2.5);
		QCOMPARE(c["surfaceType"].toString(), QStringLiteral("asphalt"));
	}

	// A final trajectory (MapBridge's, for the prompt) carries each value
	// only when known; a point's seconds and on-ground state always, its
	// per-engine values when one engine's is known (an unknown one's null).
	void profileCarriesKnownValues() {
		ContactProfile contactProfile;
		ProfilePoint gate;
		gate.gate = QStringLiteral("500 ft");
		gate.seconds = -30;
		gate.heightFt = 510;
		ProfilePoint last;  // at the contact: seconds 0 is still sent
		last.heightFt = 0;
		contactProfile.gates = { gate };
		contactProfile.series = { last };
		contactProfile.highestTerrainFt = 420;
		const QJsonObject profile = mapProfileJson(contactProfile);
		QCOMPARE(profile.keys(), QStringList({ "gates", "highestTerrainFt", "series" }));
		QCOMPARE(profile["highestTerrainFt"].toDouble(), 420.0);
		const QJsonObject g = profile["gates"].toArray().at(0).toObject();
		QCOMPARE(g.keys(), QStringList({ "gate", "heightFt", "onGround", "seconds" }));
		QCOMPARE(g["gate"].toString(), QStringLiteral("500 ft"));
		QCOMPARE(g["seconds"].toDouble(), -30.0);
		QCOMPARE(g["heightFt"].toDouble(), 510.0);
		const QJsonArray series = profile["series"].toArray();
		QCOMPARE(series.size(), 1);
		QCOMPARE(series.at(0).toObject().keys(), QStringList({ "heightFt", "onGround", "seconds" }));
		QCOMPARE(series.at(0).toObject()["seconds"].toDouble(), 0.0);

		// Each point's place on the runway, when known.
		ProfilePoint onRunway;
		onRunway.thresholdFt = -120.5;
		onRunway.centerlineFt = 4.25;
		contactProfile.series = { onRunway };
		const QJsonObject point = mapProfileJson(contactProfile)["series"].toArray().at(0).toObject();
		QCOMPARE(point.keys(), QStringList({ "centerlineFt", "onGround", "seconds", "thresholdFt" }));
		QCOMPARE(point["thresholdFt"].toDouble(), -120.5);
		QCOMPARE(point["centerlineFt"].toDouble(), 4.25);

		// Engine values: none known is absent, one of two known is sent with
		// the other's null.
		ProfilePoint engines;
		engines.throttle = { std::nan(""), std::nan("") };
		engines.power = { std::nan(""), 52.5 };
		contactProfile.series = { engines };
		const QJsonObject enginePoint = mapProfileJson(contactProfile)["series"].toArray().at(0).toObject();
		QCOMPARE(enginePoint.keys(), QStringList({ "onGround", "power", "seconds" }));
		QCOMPARE(enginePoint["power"].toArray(), (QJsonArray{ QJsonValue(), 52.5 }));
	}

	// Every condition and trajectory value reaches the page under the name
	// map.html reads it by.
	void everyValueHasTheNameThePageReads() {
		TouchdownPoint td;
		ContactConditions& c = td.conditions;
		c.fromSample = true;
		c.headingTrue = 1;
		c.headingMagnetic = 2;
		c.trackTrue = 3;
		c.magneticVariation = 4;
		c.crabAngle = 5;
		c.sunElevation = 6;
		c.temperatureC = 7;
		c.stationPressureHg = 8;
		c.altimeterHg = 9;
		c.altimeterMb = 10;
		c.pressureAltitudeFt = 11;
		c.densityAltitudeFt = 12;
		c.visibilityM = 13;
		c.seaLevelPressureMb = 14;
		c.precipitation = QStringLiteral("snow");
		c.precipitationRate = 15;
		c.inCloud = 1;
		c.fieldElevationFt = 16;
		c.windMinKt = 17;
		c.windMaxKt = 18;
		c.windDirectionFrom = 19;
		c.windDirectionTo = 20;
		c.surfaceType = QStringLiteral("grass");
		c.surfaceCondition = QStringLiteral("icy");
		c.runwayHeadingMagnetic = 21;
		const QJsonObject o = argumentOf(mapSetTouchdownsJs({ td }), QStringLiteral("setTouchdowns")).array().at(0).toObject();
		QCOMPARE(o["conditions"].toObject(), (QJsonObject{ { "headingTrue", 1 }, { "trackTrue", 3 },
			{ "magneticVariation", 4 }, { "crabAngle", 5 }, { "runwayHeadingMagnetic", 21 }, { "sunElevation", 6 }, { "temperatureC", 7 },
			{ "stationPressureHg", 8 }, { "altimeterHg", 9 }, { "altimeterMb", 10 }, { "pressureAltitudeFt", 11 },
			{ "densityAltitudeFt", 12 }, { "visibilityM", 13 }, { "seaLevelPressureMb", 14 }, { "precipitation", "snow" },
			{ "precipitationRate", 15 }, { "inCloud", 1 }, { "fieldElevationFt", 16 }, { "windMinKt", 17 },
			{ "windMaxKt", 18 }, { "windDirectionFrom", 19 }, { "windDirectionTo", 20 }, { "surfaceType", "grass" },
			{ "surfaceCondition", "icy" } }));

		ProfilePoint p;
		p.gate = QStringLiteral("2 nm");
		p.seconds = -40;
		p.heightFt = 640;
		p.distanceNm = 2;
		p.terrainFt = 30;
		p.airspeed = 135;
		p.verticalSpeed = -720;
		p.pitch = 1.5;
		p.bank = -3;
		p.headingTrue = 271;
		p.trackTrue = 268;
		p.thresholdFt = -12000;
		p.centerlineFt = 25;
		p.pathAngle = 3.1;
		p.onGround = true;
		p.throttle = { -4, 30 };
		p.power = { 53, 54 };
		ContactProfile contactProfile;
		contactProfile.gates = { p };
		contactProfile.highestTerrainFt = 420;
		contactProfile.powerName = QStringLiteral("N1");
		contactProfile.powerUnit = QStringLiteral("%");
		contactProfile.highestTerrainNm = 3.5;
		const QJsonObject profile = mapProfileJson(contactProfile);
		QCOMPARE(profile["gates"].toArray().at(0).toObject(), (QJsonObject{ { "gate", "2 nm" }, { "seconds", -40 },
			{ "heightFt", 640 }, { "distanceNm", 2 }, { "terrainFt", 30 }, { "airspeed", 135 }, { "verticalSpeed", -720 },
			{ "pitch", 1.5 }, { "bank", -3 }, { "headingTrue", 271 }, { "trackTrue", 268 }, { "thresholdFt", -12000 },
			{ "centerlineFt", 25 }, { "pathAngle", 3.1 }, { "onGround", true }, { "throttle", QJsonArray{ -4, 30 } },
			{ "power", QJsonArray{ 53, 54 } } }));
		QCOMPARE(profile["powerName"].toString(), QStringLiteral("N1"));
		QCOMPARE(profile["powerUnit"].toString(), QStringLiteral("%"));
		QCOMPARE(profile["highestTerrainFt"].toDouble(), 420.0);
		QCOMPARE(profile["highestTerrainNm"].toDouble(), 3.5);
	}

	// The runway end's stored data is sent when known; the start and exact
	// heading stay in C++.
	void contactsCarryTheirRunwayDataWhenKnown() {
		LiftoffPoint lo;
		fillRunwayContact(lo);
		lo.runwayData.lengthFt = 9800;
		lo.runwayData.widthFt = 148;
		lo.runwayData.thresholdOffsetFt = 0;
		lo.runwayData.slopeDegrees = -0.25;
		lo.runwayData.startLatitude = 47.4;
		lo.runwayData.startLongitude = 8.6;
		lo.runwayData.trueHeading = 155.5;
		lo.conditions.fromSample = true;
		const QJsonObject o = argumentOf(mapSetLiftoffsJs({ lo }), QStringLiteral("setLiftoffs")).array().at(0).toObject();
		QCOMPARE(o["runwayLengthFt"].toDouble(), 9800.0);
		QCOMPARE(o["runwayWidthFt"].toDouble(), 148.0);
		// 0 (no displaced threshold) is a known value, sent, not left out.
		QCOMPARE(o["thresholdOffsetFt"], QJsonValue(0.0));
		QCOMPARE(o["runwaySlope"].toDouble(), -0.25);
		QCOMPARE(o.size(), 21 + 4 + 1);  // the shared fields, the runway data and conditions
	}

	void emptyMarkerListsAreStillCalls() {
		QCOMPARE(mapSetLiftoffsJs({}), QStringLiteral("setLiftoffs([]);"));
		QCOMPARE(mapSetTouchdownsJs({}), QStringLiteral("setTouchdowns([]);"));
		QCOMPARE(mapSetEventsJs({}), QStringLiteral("setEvents([]);"));
		QCOMPARE(mapSetOverviewJs({}), QStringLiteral("setOverview([]);"));
	}

	void eventsCarryPositionNameAndTime() {
		TripEvent e;
		e.latitude = 1.5;
		e.longitude = -2.5;
		e.event = QStringLiteral("GEAR_UP");
		e.zuluTime = QStringLiteral("2026-03-04T10:00:00.000+00:00_3");
		const QJsonArray arr = argumentOf(mapSetEventsJs({ e }), QStringLiteral("setEvents")).array();
		const QJsonObject o = arr.at(0).toObject();
		QCOMPARE(o["lat"].toDouble(), 1.5);
		QCOMPARE(o["lng"].toDouble(), -2.5);
		QCOMPARE(o["event"].toString(), QStringLiteral("GEAR_UP"));
		QCOMPARE(o["zuluTime"].toString(), e.zuluTime);
		QCOMPARE(o.size(), 4);
	}

	void eventsVisibleFlag() {
		QCOMPARE(mapSetEventsVisibleJs(true), QStringLiteral("setEventsVisible(true);"));
		QCOMPARE(mapSetEventsVisibleJs(false), QStringLiteral("setEventsVisible(false);"));
	}

	void overviewHasOneSegmentPerTripWithItsGroup() {
		TripSummary a;
		a.id = 7;
		a.departureLat = 1; a.departureLng = 2; a.destinationLat = 3; a.destinationLng = 4;
		a.departureIcao = QStringLiteral("AAAA");
		a.destinationIcao = QStringLiteral("BBBB");
		a.groupId = 3;
		a.groupName = QStringLiteral("Tour");
		a.groupRank = 1;
		TripSummary b = a;
		b.id = 8;
		b.groupId = 0;
		b.groupName = QStringLiteral("ignored");
		b.groupRank = 0;
		const QJsonArray segs = argumentOf(mapSetOverviewJs({ a, b }), QStringLiteral("setOverview")).array();
		QCOMPARE(segs.size(), 2);
		const QJsonObject s = segs.at(0).toObject();
		QCOMPARE(s["tripId"].toInt(), 7);
		QCOMPARE(s["fromLat"].toDouble(), 1.0);
		QCOMPARE(s["fromLng"].toDouble(), 2.0);
		QCOMPARE(s["toLat"].toDouble(), 3.0);
		QCOMPARE(s["toLng"].toDouble(), 4.0);
		QCOMPARE(s["fromIcao"].toString(), QStringLiteral("AAAA"));
		QCOMPARE(s["toIcao"].toString(), QStringLiteral("BBBB"));
		QCOMPARE(s["groupId"].toInt(), 3);
		QCOMPARE(s["groupName"].toString(), QStringLiteral("Tour"));
		QCOMPARE(s["groupRank"].toInt(), 1);
		QCOMPARE(s.size(), 10);
		QCOMPARE(segs.at(1).toObject()["groupName"].toString(), QStringLiteral("Ungrouped"));
	}
};

QTEST_GUILESS_MAIN(TstMapScript)
#include "tst_map_script.moc"

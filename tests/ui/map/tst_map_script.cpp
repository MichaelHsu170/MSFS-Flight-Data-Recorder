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

	// A contact with a sample also carries its conditions and final
	// trajectory, each value only when known (no NaN reaches the page).
	void contactsWithASampleCarryConditionsAndProfile() {
		TouchdownPoint td;
		fillRunwayContact(td);
		td.conditions.fromSample = true;
		td.conditions.headingTrue = 268.5;
		td.conditions.crabAngle = -2.5;
		td.conditions.surfaceType = QStringLiteral("asphalt");
		td.conditions.inCloud = 0;  // known: "not in cloud" is sent, not left out
		td.conditions.precipitation = QStringLiteral("rain");
		td.conditions.seaLevelPressureMb = 1008;
		ProfilePoint gate;
		gate.gate = QStringLiteral("500 ft");
		gate.seconds = -30;
		gate.heightFt = 510;
		ProfilePoint last;  // at the contact: seconds 0 is still sent
		last.heightFt = 0;
		td.profile.gates = { gate };
		td.profile.series = { last };
		td.profile.highestTerrainFt = 420;
		const QJsonObject o = argumentOf(mapSetTouchdownsJs({ td }), QStringLiteral("setTouchdowns")).array().at(0).toObject();
		QCOMPARE(o.size(), 24);

		const QJsonObject c = o["conditions"].toObject();
		QCOMPARE(c.keys(), QStringList({ "crabAngle", "headingTrue", "inCloud", "precipitation", "seaLevelPressureMb", "surfaceType" }));
		QCOMPARE(c["inCloud"].toDouble(), 0.0);
		QCOMPARE(c["precipitation"].toString(), QStringLiteral("rain"));
		QCOMPARE(c["seaLevelPressureMb"].toDouble(), 1008.0);
		QCOMPARE(c["headingTrue"].toDouble(), 268.5);
		QCOMPARE(c["crabAngle"].toDouble(), -2.5);
		QCOMPARE(c["surfaceType"].toString(), QStringLiteral("asphalt"));

		const QJsonObject profile = o["profile"].toObject();
		QCOMPARE(profile.keys(), QStringList({ "gates", "highestTerrainFt", "series" }));
		QCOMPARE(profile["highestTerrainFt"].toDouble(), 420.0);
		const QJsonObject g = profile["gates"].toArray().at(0).toObject();
		QCOMPARE(g.keys(), QStringList({ "gate", "heightFt", "seconds" }));
		QCOMPARE(g["gate"].toString(), QStringLiteral("500 ft"));
		QCOMPARE(g["seconds"].toDouble(), -30.0);
		QCOMPARE(g["heightFt"].toDouble(), 510.0);
		const QJsonArray series = profile["series"].toArray();
		QCOMPARE(series.size(), 1);
		QCOMPARE(series.at(0).toObject().keys(), QStringList({ "heightFt", "seconds" }));
		QCOMPARE(series.at(0).toObject()["seconds"].toDouble(), 0.0);
	}

	// The runway end's stored data and each point's place on the runway are
	// sent when known; the start and exact heading stay in C++.
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
		ProfilePoint p;
		p.thresholdFt = -120.5;
		p.centerlineFt = 4.25;
		lo.profile.series = { p };
		const QJsonObject o = argumentOf(mapSetLiftoffsJs({ lo }), QStringLiteral("setLiftoffs")).array().at(0).toObject();
		QCOMPARE(o["runwayLengthFt"].toDouble(), 9800.0);
		QCOMPARE(o["runwayWidthFt"].toDouble(), 148.0);
		QCOMPARE(o["thresholdOffsetFt"].toDouble(), 0.0);
		QCOMPARE(o["runwaySlope"].toDouble(), -0.25);
		QCOMPARE(o.size(), 21 + 4 + 2);  // the shared fields, the runway data, conditions and profile
		const QJsonObject point = o["profile"].toObject()["series"].toArray().at(0).toObject();
		QCOMPARE(point.keys(), QStringList({ "centerlineFt", "seconds", "thresholdFt" }));
		QCOMPARE(point["thresholdFt"].toDouble(), -120.5);
		QCOMPARE(point["centerlineFt"].toDouble(), 4.25);
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

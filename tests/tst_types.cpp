// types.h: geodesic math, coordinate/time formatting, runway codes, AIRPORT
// ownership, and the two DB write queues.
#include "types.h"

#include <QtTest>

#include <thread>

class TstTypes : public QObject {
	Q_OBJECT

private:
	static COORDINATE at(double lat, double lon) {
		COORDINATE c;
		c.latitude = lat;
		c.longitude = lon;
		return c;
	}

private slots:
	void coordinateDefaultsToInvalidSentinel() {
		COORDINATE c;
		QCOMPARE(c.latitude, 360.0);
		QCOMPARE(c.longitude, 360.0);
	}

	void distanceOneDegreeOfLongitudeAtEquator() {
		// 6371 km * pi / 180
		QVERIFY(qAbs(at(0, 0).distanceInKm2Coordinate(at(0, 1)) - 111.19492664) < 1e-6);
	}

	void distanceLondonToParis() {
		QVERIFY(qAbs(at(51.5074, -0.1278).distanceInKm2Coordinate(at(48.8566, 2.3522)) - 343.56) < 0.1);
	}

	void distanceToSelfIsZero() {
		QCOMPARE(at(43, 1).distanceInKm2Coordinate(at(43, 1)), 0.0);
	}

	void bearingCardinalDirections() {
		QVERIFY(qAbs(at(0, 0).bearing2Coordinate(at(0, 1)) - 90) < 1e-9);
		QVERIFY(qAbs(at(0, 0).bearing2Coordinate(at(-1, 0)) - 180) < 1e-9);
		QVERIFY(qAbs(at(0, 0).bearing2Coordinate(at(0, -1)) - 270) < 1e-9);
		// Due north is reported as 360, never 0 (wrap_bearing()).
		QVERIFY(qAbs(at(0, 0).bearing2Coordinate(at(1, 0)) - 360) < 1e-9);
	}

	void wrapBearingIntoZeroExclusiveTo360() {
		QCOMPARE(wrap_bearing(0), 360.0);
		QCOMPARE(wrap_bearing(-90), 270.0);
		QCOMPARE(wrap_bearing(360), 360.0);
		QCOMPARE(wrap_bearing(370), 10.0);
		QCOMPARE(wrap_bearing(123.5), 123.5);
	}

	void bearingDifferenceTakesTheShorterWayRound() {
		QCOMPARE(bearing_difference(10, 350), 20.0);
		QCOMPARE(bearing_difference(350, 10), 20.0);
		QCOMPARE(bearing_difference(90, 270), 180.0);
		QCOMPARE(bearing_difference(100, 40), 60.0);
		QCOMPARE(bearing_difference(360, 360), 0.0);
	}

	void destinationAlongEquatorAndMeridian() {
		COORDINATE east = at(0, 0).destinationWithDistanceAndBearing(111.19492664, 90);
		QVERIFY(qAbs(east.latitude) < 1e-9);
		QVERIFY(qAbs(east.longitude - 1) < 1e-6);
		COORDINATE north = at(0, 0).destinationWithDistanceAndBearing(111.19492664, 360);
		QVERIFY(qAbs(north.latitude - 1) < 1e-6);
		QVERIFY(qAbs(north.longitude) < 1e-9);
	}

	void destinationOffTheEquatorMatchesReference() {
		// The worked example from Chris Veness's "Calculate distance, bearing
		// and more between Latitude/Longitude points" (movable-type.co.uk),
		// also with R = 6371 km: from 53°19'14"N 001°43'47"W on bearing
		// 096°01'18" for 124.8 km lands at 53°11'18"N 000°08'00"E. The
		// published answer is to the arcsecond (0.00028°).
		COORDINATE start = at(53 + 19 / 60.0 + 14 / 3600.0, -(1 + 43 / 60.0 + 47 / 3600.0));
		COORDINATE end = start.destinationWithDistanceAndBearing(124.8, 96 + 1 / 60.0 + 18 / 3600.0);
		QVERIFY2(qAbs(end.latitude - (53 + 11 / 60.0 + 18 / 3600.0)) < 0.0003, qPrintable(QString::number(end.latitude, 'f', 6)));
		QVERIFY2(qAbs(end.longitude - 8 / 60.0) < 0.0003, qPrintable(QString::number(end.longitude, 'f', 6)));
	}

	void destinationRoundTripsAtRunwayScale() {
		// At the few-kilometre scale the recorder uses it for (runway ends,
		// margin rectangles), destination -> distance/bearing agrees closely.
		COORDINATE origin = at(43.6, 1.4);
		COORDINATE target = origin.destinationWithDistanceAndBearing(1.5, 123);
		QVERIFY(qAbs(origin.distanceInKm2Coordinate(target) - 1.5) < 0.001);
		QVERIFY(qAbs(origin.bearing2Coordinate(target) - 123) < 0.1);
	}

	void dmsFormatting() {
		COORDINATE c = at(43.5, -1.25);
		QCOMPARE(QString::fromStdString(c.coordinate_decimal_to_dms(COORDINATE::LATITUDE)), QString::fromUtf8("43°30'00.0\"N"));
		QCOMPARE(QString::fromStdString(c.coordinate_decimal_to_dms(COORDINATE::LONGITUDE)), QString::fromUtf8("1°15'00.0\"W"));
		// Rounded to a tenth of a second, carrying into the minute and degree
		// rather than showing 60.0 seconds.
		COORDINATE t = at(0.9999999, 0);
		QCOMPARE(QString::fromStdString(t.coordinate_decimal_to_dms(COORDINATE::LATITUDE)), QString::fromUtf8("1°00'00.0\"N"));
		COORDINATE s = at(-10.5000278, 0);
		QCOMPARE(QString::fromStdString(s.coordinate_decimal_to_dms(COORDINATE::LATITUDE)), QString::fromUtf8("10°30'00.1\"S"));
	}

	void dateTimeFormatting() {
		DATETIME t;
		t.year = 2026;
		t.month_of_year = 1;
		t.day_of_month = 2;
		t.day_of_week = 5;
		t.time_day = 36000.5;
		// timezone_offset is SimConnect's UTC minus local: 3600 is UTC-1.
		t.timezone_offset = 3600;
		QCOMPARE(QString::fromStdString(t.format_date_time()), QStringLiteral("2026-01-02T10:00:00.500-01:00_5"));
		t.timezone_offset = -5400;
		t.time_day = 3 * 3600 + 4 * 60 + 5.25;
		QCOMPARE(QString::fromStdString(t.format_date_time()), QStringLiteral("2026-01-02T03:04:05.250+01:30_5"));
		t.timezone_offset = 0;
		QCOMPARE(QString::fromStdString(t.format_date_time()), QStringLiteral("2026-01-02T03:04:05.250+00:00_5"));
	}

	void runwayCodes_data() {
		QTest::addColumn<int>("number");
		QTest::addColumn<int>("designator");
		QTest::addColumn<QString>("expected");
		QTest::newRow("no designator") << 9 << 0 << "09";
		QTest::newRow("left") << 9 << 1 << "09L";
		QTest::newRow("right") << 27 << 2 << "27R";
		QTest::newRow("center") << 36 << 3 << "36C";
		QTest::newRow("water") << 18 << 4 << "18W";
		QTest::newRow("A") << 1 << 5 << "01A";
		QTest::newRow("B") << 1 << 6 << "01B";
		QTest::newRow("compass N") << 37 << 0 << "N";
		QTest::newRow("compass NW with designator") << 44 << 1 << "NWL";
		QTest::newRow("zero") << 0 << 0 << "";
		QTest::newRow("out of range") << 45 << 0 << "";
	}

	void runwayCodes() {
		QFETCH(int, number);
		QFETCH(int, designator);
		QFETCH(QString, expected);
		RUNWAY rwy;
		rwy.numbers[0] = number;
		rwy.designators[0] = designator;
		QCOMPARE(QString::fromStdString(rwy.runway_code_generator(true)), expected);
	}

	void runwayCodeSecondaryEnd() {
		RUNWAY rwy;
		rwy.numbers[0] = 9;
		rwy.numbers[1] = 27;
		rwy.designators[0] = 1;
		rwy.designators[1] = 2;
		QCOMPARE(QString::fromStdString(rwy.runway_code_generator(false)), QStringLiteral("27R"));
	}

	void copyCstrCutsToFitAndZeroFills() {
		char buf[6];
		memset(buf, 'x', sizeof(buf));
		copy_cstr(buf, "LFBOXYZ");
		QCOMPARE(QByteArray(buf, sizeof(buf)), QByteArray("LFBOX\0", 6));
		memset(buf, 'x', sizeof(buf));
		copy_cstr(buf, "LF");
		QCOMPARE(QByteArray(buf, sizeof(buf)), QByteArray("LF\0\0\0\0", 6));
	}

	void airportCopyIsDeepAndClearResets() {
		AIRPORT src;
		copy_cstr(src.name, "Test Field");
		copy_cstr(src.icao, "LFBO");
		copy_cstr(src.region, "LF");
		src.magvar = 1.5f;
		src.n_runways = 1;
		src.runways = (RUNWAY*)calloc(1, sizeof(RUNWAY));
		src.runways[0].numbers[0] = 14;
		src.runways[0].designators[0] = 2;
		src.runway_act.index = 0;
		src.runway_act.is_primary = true;

		AIRPORT dst;
		dst.copy(&src);
		QVERIFY(dst.runways != nullptr);
		QVERIFY(dst.runways != src.runways);
		QCOMPARE(QString(dst.icao), QStringLiteral("LFBO"));
		QCOMPARE(QString(dst.name), QStringLiteral("Test Field"));
		QCOMPARE(QString::fromStdString(dst.runway_code_generator()), QStringLiteral("14R"));

		dst.clear();
		QVERIFY(dst.runways == nullptr);
		QCOMPARE(dst.n_runways, 0);
		QCOMPARE(dst.runway_act.index, -1);
		QCOMPARE(dst.runway_act.heading, -1);
		QCOMPARE(dst.runway_act.distances[0], -1.0);
		QCOMPARE(QString::fromStdString(dst.runway_code_generator()), QString());
	}

	void sampleQueueDrainsBeforeStopping() {
		SampleWriteQueue q;
		q.push(nullptr, 1);
		q.push(nullptr, 2);
		q.stop();
		SAMPLE_QUEUE_ITEM item;
		QVERIFY(q.pop(item));
		QCOMPARE(item.trip_id, 1);
		QVERIFY(q.pop(item));
		QCOMPARE(item.trip_id, 2);
		QVERIFY(!q.pop(item));
		q.reset();
		q.push(nullptr, 3);
		QVERIFY(q.pop(item));
		QCOMPARE(item.trip_id, 3);
	}

	void sampleQueuePopBlocksUntilPush() {
		SampleWriteQueue q;
		std::thread producer([&q] {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			q.push(nullptr, 7);
		});
		SAMPLE_QUEUE_ITEM item;
		QVERIFY(q.pop(item));
		QCOMPARE(item.trip_id, 7);
		producer.join();
	}

	void eventQueueKeepsInsertAndDeleteOrder() {
		EventWriteQueue q;
		q.push(4, "GEAR_UP", "z", "l", 11);
		q.push_delete({ 11 });
		q.stop();
		EVENT_QUEUE_ITEM item;
		QVERIFY(q.pop(item));
		QVERIFY(item.kind == EVENT_QUEUE_ITEM::Kind::Insert);
		QCOMPARE(item.trip_id, 4);
		QCOMPARE(QString::fromStdString(item.event), QStringLiteral("GEAR_UP"));
		QCOMPARE(item.seq, 11ull);
		QVERIFY(q.pop(item));
		QVERIFY(item.kind == EVENT_QUEUE_ITEM::Kind::Delete);
		QCOMPARE(item.delete_seqs.size(), size_t(1));
		QVERIFY(!q.pop(item));
	}
};

QTEST_APPLESS_MAIN(TstTypes)
#include "tst_types.moc"

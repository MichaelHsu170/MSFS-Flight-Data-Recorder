// KML export (kml_export.cpp): document structure, placemark contents,
// escaping, event grouping, and write failures.
#include "kml_export.h"

#include <QFile>
#include <QTemporaryDir>
#include <QtTest>

namespace {

TripSamplePoint sample(double lat, double lon, int altFt, const char* zulu) {
	TripSamplePoint p;
	p.latitude = lat;
	p.longitude = lon;
	p.altitude = altFt;
	p.zuluTime = QString::fromLatin1(zulu);
	return p;
}

TripDataset fullDataset() {
	TripDataset d;
	d.tripId = 7;
	d.points = { sample(43.0, 1.0, 1000, "2026-01-02T10:00:00.000+00:00_5"), sample(43.1, 1.1, 2000, "2026-01-02T10:00:01.000+00:00_5") };
	LiftoffPoint lo;
	lo.latitude = 43.0;
	lo.longitude = 1.0;
	lo.icao = "TEST";
	lo.airportName = "Test Field";
	lo.runway = "09";
	lo.runwayHeading = 91;
	lo.airspeed = 150;
	lo.verticalSpeed = 500;
	lo.pitchDegrees = 8.5;
	lo.bankDegrees = -1.5;
	lo.headingDegrees = 92;
	lo.distanceLength = 5905.5;
	lo.distanceWidth = 9.8;
	lo.distanceLengthPercent = 0.6;
	lo.distanceWidthPercent = 0.14;
	lo.windDirection = 270;
	lo.windVelocity = 12;
	lo.zuluTime = "2026-01-02T10:00:00.000+00:00_5";
	lo.localTime = "2026-01-02T11:00:00.000+01:00_5";
	d.liftoffPoints = { lo };
	TouchdownPoint td;
	td.latitude = 43.1;
	td.longitude = 1.1;
	td.icao = "DEST";
	td.runway = "27";
	td.runwayHeading = -1;
	td.airspeed = 130;
	td.verticalSpeed = -180;
	td.gForce = 1.3;
	td.distanceLength = -164;
	td.distanceWidth = -3.2;
	td.distanceLengthPercent = -0.02;
	td.distanceWidthPercent = -0.04;
	td.zuluTime = "2026-01-02T10:00:01.000+00:00_5";
	d.touchdowns = { td };
	TripEvent a, b, c;
	a.event = "GEAR_UP";
	a.zuluTime = "2026-01-02T10:00:00.000+00:00_5";
	a.latitude = 43.0;
	a.longitude = 1.0;
	b = a;
	b.event = "FLAPS_UP & <more>";
	c = a;
	c.event = "SPOILERS_ARM_ON";
	c.latitude = 43.05;
	d.events = { a, b, c };
	return d;
}

}

class TstKml : public QObject {
	Q_OBJECT

private:
	QTemporaryDir dir_;

	QString exportToString(const TripDataset& d) {
		const QString path = dir_.filePath("trip.kml");
		QString error;
		if (!exportTripDatasetToKmlFile(d, path, &error))
			return QStringLiteral("EXPORT FAILED: ") + error;
		QFile f(path);
		if (!f.open(QIODevice::ReadOnly))
			return QStringLiteral("OPEN FAILED: ") + f.errorString();
		return QString::fromUtf8(f.readAll());
	}

private slots:
	void documentHeaderAndName() {
		const QString kml = exportToString(fullDataset());
		QVERIFY(kml.startsWith("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\""));
		QVERIFY(kml.contains("<name>TEST-DEST</name>"));
		QVERIFY(kml.trimmed().endsWith("</Document>\n</kml>") || kml.trimmed().endsWith("</kml>"));
	}

	void flightPathAndTrackInMeters() {
		const QString kml = exportToString(fullDataset());
		QVERIFY(kml.contains("<coordinates>1.000000,43.000000,304.8 1.100000,43.100000,609.6 </coordinates>"));
		QVERIFY(kml.contains("<when>2026-01-02T10:00:00.000Z</when>"));
		QVERIFY(kml.contains("<gx:coord>1.100000 43.100000 609.6</gx:coord>"));
	}

	void liftoffPlacemark() {
		const QString kml = exportToString(fullDataset());
		QVERIFY(kml.contains("<Folder><name>Liftoffs</name>"));
		QVERIFY(kml.contains("<Placemark><name>Liftoff</name><styleUrl>#liftoffStyle</styleUrl><TimeStamp><when>2026-01-02T10:00:00.000Z</when></TimeStamp>"));
		for (const char* row : { "<b>Airport:</b> TEST (Test Field)<br/>", "<b>Runway:</b> 09 (91°)<br/>", "<b>Airspeed:</b> 150 kt<br/>",
				"<b>V/S:</b> 500 ft/min<br/>", "<b>Pitch:</b> 8.5°<br/>", "<b>Bank:</b> -1.5°<br/>", "<b>Heading:</b> 92°<br/>",
				"<b>Threshold:</b> 5906 ft (60%)<br/>", "<b>Centerline:</b> 10 ft R (14%)<br/>", "<b>Wind:</b> 270° / 12 kt<br/>",
				"<b>Zulu:</b> 2026-01-02T10:00:00.000+00:00_5<br/>", "<b>Local:</b> 2026-01-02T11:00:00.000+01:00_5<br/>" })
			QVERIFY2(kml.contains(QString::fromUtf8(row)), row);
		QVERIFY(!kml.contains("<b>G-Force:</b> 0"));
	}

	void touchdownPlacemark() {
		const QString kml = exportToString(fullDataset());
		for (const char* row : { "<b>Airport:</b> DEST<br/>", "<b>Runway:</b> 27<br/>", "<b>G-Force:</b> 1.30 G<br/>",
				"<b>Threshold:</b> -164 ft (-2%)<br/>", "<b>Centerline:</b> 3 ft L (4%)<br/>" })
			QVERIFY2(kml.contains(QString::fromUtf8(row)), row);
		QVERIFY(!kml.contains("<b>Local:</b> <br/>"));
	}

	void noRunwayMeansNoThresholdRows() {
		TripDataset d = fullDataset();
		d.liftoffPoints[0].runway.clear();
		d.touchdowns.clear();
		const QString kml = exportToString(d);
		QVERIFY(!kml.contains("Threshold"));
		QVERIFY(!kml.contains("Centerline"));
		QVERIFY(kml.contains("<b>Wind:</b>"));
	}

	void eventsAtTheSameSpotShareAPlacemarkAndAreEscaped() {
		const QString kml = exportToString(fullDataset());
		QVERIFY(kml.contains("<name>Events (2)</name>"));
		QVERIFY(kml.contains("<name>SPOILERS_ARM_ON</name>"));
		// Placemark names are XML-escaped; CDATA descriptions are not.
		QVERIFY(kml.contains("<b>FLAPS_UP & <more>:</b>"));
		QCOMPARE(kml.count("<styleUrl>#eventStyle</styleUrl>"), 2);
	}

	void placemarkNamesAreXmlEscaped() {
		TripDataset d;
		d.tripId = 1;
		TripEvent e;
		e.event = "A & <B>";
		d.events = { e };
		QVERIFY(exportToString(d).contains("<name>A &amp; &lt;B&gt;</name>"));
	}

	void emptyTripHasOnlyTheDocument() {
		TripDataset d;
		d.tripId = 3;
		const QString kml = exportToString(d);
		QVERIFY(kml.contains("<name>Trip 3</name>"));
		QVERIFY(!kml.contains("<Folder>"));
	}

	void unparseableTimesAreLeftOut() {
		TripDataset d;
		d.tripId = 1;
		d.points = { sample(1, 2, 0, "bad"), sample(1, 2, 0, "2026-01-02T10:00:00.000+00:00_5") };
		const QString kml = exportToString(d);
		QCOMPARE(kml.count("<when>"), 1);
		QCOMPARE(kml.count("<gx:coord>"), 1);
		// Both points stay in the static path.
		QVERIFY(kml.contains("<coordinates>2.000000,1.000000,0.0 2.000000,1.000000,0.0 </coordinates>"));
	}

	void writeFailureIsReported() {
		QString error;
		QVERIFY(!exportTripDatasetToKmlFile(fullDataset(), dir_.filePath("missing/dir/trip.kml"), &error));
		QVERIFY(!error.isEmpty());
		QVERIFY(!exportTripDatasetToKmlFile(fullDataset(), dir_.filePath("missing/dir/trip.kml")));
	}
};

QTEST_APPLESS_MAIN(TstKml)
#include "tst_kml.moc"

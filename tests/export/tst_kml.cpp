// KML export (kml_export.cpp): document structure, placemark contents,
// escaping, event grouping, and write failures; and from kml_export_dialog.cpp,
// the error box both export menus show, a cancelled save dialog and a
// database that can't be opened.
#include "test_support.h"

#include "kml_export.h"
#include "kml_export_dialog.h"

#include <QFile>
#include <QMessageBox>
#include <QTemporaryDir>
#include <QXmlStreamReader>
#include <QtTest>

#include <cmath>

using namespace TestSupport;

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
	lo.actualVerticalSpeed = 519.6;
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
	td.touchdownRate = 262.5;
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

	// Answers exportTripToKml()'s save dialog with answerDialog, then returns
	// the text of the error box it shows ("" if none within timeoutMs).
	static QString exportErrorShown(const std::function<void(QWidget*)>& answerDialog, int timeoutMs) {
		QWidget parent;
		QString error;
		onNextModal(answerDialog);
		exportTripToKml(&parent, QStringLiteral("trip"), 1);
		onNextModal([&error](QWidget* box) {
			error = static_cast<QMessageBox*>(box)->text();
			clickDialogButton(box, "OK");
		});
		waitFor([&error] { return !error.isEmpty(); }, timeoutMs);
		return error;
	}

private slots:
	void initTestCase() {
		isolateFiles();
		QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs); // a test can't answer a native dialog
	}
	void cleanup() { cancelPendingModals(); } // no dialog action outlives its test
	void documentHeaderAndName() {
		const QString kml = exportToString(fullDataset());
		QVERIFY(kml.startsWith("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\""));
		QVERIFY(kml.contains("<name>TEST-DEST</name>"));
		QVERIFY(kml.endsWith("</Document>\n</kml>\n"));
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
		QVERIFY(kml.contains("<Placemark><name>Liftoff</name><description>"));
		QVERIFY(kml.contains("<TimeStamp><when>2026-01-02T10:00:00.000Z</when></TimeStamp><styleUrl>#liftoffStyle</styleUrl><Point>"));
		for (const char* row : { "<b>Airport:</b> TEST (Test Field)<br/>", "<b>Runway:</b> 09 (91° T)<br/>", "<b>Airspeed:</b> 150 kt<br/>",
				"<b>V/S:</b> 520 ft/min<br/>", "<b>Pitch:</b> 8.5°<br/>", "<b>Bank:</b> -1.5°<br/>", "<b>Heading:</b> 92° M<br/>",
				"<b>Threshold:</b> 5906 ft (60%)<br/>", "<b>Centerline:</b> 10 ft R (14%)<br/>", "<b>Wind:</b> 270° T / 12 kt<br/>",
				"<b>Zulu:</b> 2026-01-02T10:00:00.000+00:00_5<br/>", "<b>Local:</b> 2026-01-02T11:00:00.000+01:00_5<br/>" })
			QVERIFY2(kml.contains(QString::fromUtf8(row)), row);
		QVERIFY(!kml.contains("<b>G-Force:</b> 0"));
	}

	void touchdownPlacemark() {
		const QString kml = exportToString(fullDataset());
		for (const char* row : { "<b>Airport:</b> DEST<br/>", "<b>Runway:</b> 27<br/>", "<b>G-Force:</b> 1.30 G<br/>",
				"<b>V/S:</b> -180 ft/min (indicated)<br/>", "<b>Touchdown rate:</b> 263 ft/min<br/>",
				"<b>Threshold:</b> -164 ft (-2%)<br/>", "<b>Centerline:</b> 3 ft L (4%)<br/>" })
			QVERIFY2(kml.contains(QString::fromUtf8(row)), row);
		QVERIFY(!kml.contains("<b>Local:</b> <br/>"));
		// A liftoff has no touchdown rate; a touchdown without one, no row.
		QCOMPARE(kml.count(QStringLiteral("<b>Touchdown rate:</b>")), 1);
		TripDataset dataset = fullDataset();
		dataset.touchdowns[0].touchdownRate = std::nan("");
		QVERIFY(!exportToString(dataset).contains(QStringLiteral("<b>Touchdown rate:</b>")));
	}

	// With the contact's sample the heading shows magnetic, then true in
	// brackets, and the runway heading magnetic, then true (the true ones
	// compare with the wind).
	void headingRowsAreMagneticThenTrueWithASample() {
		TripDataset d = fullDataset();
		d.liftoffPoints[0].conditions.headingTrue = 96.6;
		d.liftoffPoints[0].conditions.runwayHeadingMagnetic = 86;
		const QString kml = exportToString(d);
		QVERIFY(kml.contains(QString::fromUtf8("<b>Heading:</b> 92° M (97° T)<br/>")));
		QVERIFY(kml.contains(QString::fromUtf8("<b>Runway:</b> 09 (86° M, 91° T)<br/>")));
	}

	// The crab angle from the contact's sample, rounded, with its side; no
	// row without a sample.
	void crabRowSaysWhichSide_data() {
		QTest::addColumn<double>("crabAngle");
		QTest::addColumn<QString>("row");
		QTest::newRow("left") << -2.5 << QString::fromUtf8("<b>Crab:</b> 3° L<br/>");
		QTest::newRow("right") << 1.6 << QString::fromUtf8("<b>Crab:</b> 2° R<br/>");
		QTest::newRow("none") << 0.4 << QString::fromUtf8("<b>Crab:</b> 0°<br/>");
		QTest::newRow("no sample") << std::nan("") << QString();
	}
	void crabRowSaysWhichSide() {
		QFETCH(double, crabAngle);
		QFETCH(QString, row);
		TripDataset d = fullDataset();
		d.liftoffPoints.clear();
		d.touchdowns[0].conditions.crabAngle = crabAngle;
		const QString kml = exportToString(d);
		if (row.isEmpty())
			QVERIFY(!kml.contains(QStringLiteral("<b>Crab:</b>")));
		else
			QVERIFY2(kml.contains(row), qPrintable(row));
	}

	// The KML 2.2 schema (ogckml22.xsd, AbstractFeatureGroup then Placemark)
	// fixes a Placemark's child order: name, description, TimeStamp,
	// styleUrl, then the geometry. A file out of that order fails validation.
	void placemarkChildrenFollowTheSchemaOrder() {
		const QStringList schemaOrder = { "name", "description", "TimeStamp", "styleUrl", "LineString", "Track", "Point" };
		QXmlStreamReader xml(exportToString(fullDataset()));
		int placemarks = 0;
		while (!xml.atEnd()) {
			if (xml.readNext() != QXmlStreamReader::StartElement || xml.name() != QLatin1String("Placemark"))
				continue;
			++placemarks;
			int last = -1;
			while (xml.readNextStartElement()) {
				const int at = schemaOrder.indexOf(xml.name().toString());
				QVERIFY2(at > last, qPrintable(xml.name().toString() + " out of order in Placemark " + QString::number(placemarks)));
				last = at;
				xml.skipCurrentElement();
			}
		}
		QVERIFY2(!xml.hasError(), qPrintable(xml.errorString()));
		QVERIFY(placemarks >= 4); // path, track, liftoff, touchdown at least
	}

	void noAirportMeansNoAirportRow() {
		TripDataset d = fullDataset();
		d.liftoffPoints[0].icao.clear();
		d.liftoffPoints[0].runway.clear();
		d.touchdowns.clear();
		const QString kml = exportToString(d);
		QVERIFY(kml.contains("<Placemark><name>Liftoff</name>"));
		QVERIFY(!kml.contains("<b>Airport:</b>"));
		QVERIFY(!kml.contains("Test Field"));
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
		// The CDATA description is HTML, so the name is escaped there too.
		QVERIFY(kml.contains("<b>FLAPS_UP &amp; &lt;more&gt;:</b>"));
		QCOMPARE(kml.count("<styleUrl>#eventStyle</styleUrl>"), 2);
	}

	void descriptionValuesShowLiterallyAndKeepTheCdataClosed() {
		TripDataset d = fullDataset();
		d.liftoffPoints[0].airportName = "A]]><b>&B";
		d.touchdowns.clear();
		d.events.clear();
		const QString kml = exportToString(d);
		QVERIFY(kml.contains("<b>Airport:</b> TEST (A]]&gt;&lt;b&gt;&amp;B)<br/>"));
		// The only "]]>" is the one that closes the liftoff's description.
		QCOMPARE(kml.count("]]>"), 1);
		QVERIFY(kml.contains("<br/>]]></description>"));
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

	// An earlier export still open elsewhere (e.g. in Google Earth) can't be
	// replaced: the export fails and that file keeps what it held.
	void aFailedExportLeavesTheEarlierFileAsItWas() {
		const QString path = dir_.filePath("earlier.kml");
		QFile earlier(path);
		QVERIFY(earlier.open(QIODevice::WriteOnly));
		earlier.write("earlier export");
		earlier.close();
		QVERIFY(earlier.open(QIODevice::ReadOnly)); // held open while exporting

		QString error;
		QVERIFY(!exportTripDatasetToKmlFile(fullDataset(), path, &error));
		QVERIFY(!error.isEmpty());
		QCOMPARE(earlier.readAll(), QByteArray("earlier export"));
	}

	void failureBoxIsAnErrorNamingTheFileAndTheReason() {
		QMessageBox::Icon icon = QMessageBox::NoIcon;
		QString text;
		onNextModal([&](QWidget* dialog) {
			if (auto* box = qobject_cast<QMessageBox*>(dialog)) {
				icon = box->icon();
				text = box->text();
			}
			clickDialogButton(dialog, QStringLiteral("OK"));
		});
		showKmlExportFailed(nullptr, QStringLiteral("C:/out/trip.kml"), QStringLiteral("Access denied"));
		QCOMPARE(icon, QMessageBox::Critical);
		QVERIFY(text.contains(QStringLiteral("C:/out/trip.kml")));
		QVERIFY(text.contains(QStringLiteral("Access denied")));
	}

	// Cancelling the save dialog ends the export: no error box (there's no
	// database here, which an export would report).
	void cancellingTheSaveDialogExportsNothing() {
		removeDatabase();
		const QString error = exportErrorShown([](QWidget* dialog) { static_cast<QDialog*>(dialog)->reject(); }, 500);
		QCOMPARE(error, QString());
	}

	void exportingWithNoDatabaseSaysSoAndWritesNothing() {
		removeDatabase();
		const QString path = dir_.filePath("no_database.kml");
		const QString error = exportErrorShown([&path](QWidget* dialog) { saveFileDialogAs(dialog, path); }, 10000);
		QCOMPARE(error, QStringLiteral("Failed to export KML to %1.\nCould not open the trip database.").arg(path));
		QVERIFY(!QFile::exists(path));
	}
};

QTEST_MAIN(TstKml)
#include "tst_kml.moc"

#include "kml_export.h"
#include "logger.h"

#include <QDateTime>
#include <QElapsedTimer>
#include <QFile>
#include <QIODevice>
#include <QtGlobal>

namespace {

constexpr double kFeetToMeters = 0.3048;

QString xmlEscape(const QString& s) {
	QString out;
	out.reserve(s.size());
	for (const QChar c : s) {
		if (c == QLatin1Char('&')) out += QStringLiteral("&amp;");
		else if (c == QLatin1Char('<')) out += QStringLiteral("&lt;");
		else if (c == QLatin1Char('>')) out += QStringLiteral("&gt;");
		else out += c;
	}
	return out;
}

// "lon,lat,alt" -- the tuple format <coordinates> (LineString/Point) expects.
QString formatCoordinate(double lon, double lat, double altMeters) {
	return QStringLiteral("%1,%2,%3").arg(lon, 0, 'f', 6).arg(lat, 0, 'f', 6).arg(altMeters, 0, 'f', 1);
}

// "lon lat alt" -- the space-separated format <gx:coord> expects (distinct
// from <coordinates> above).
QString formatGxCoordinate(double lon, double lat, double altMeters) {
	return QStringLiteral("%1 %2 %3").arg(lon, 0, 'f', 6).arg(lat, 0, 'f', 6).arg(altMeters, 0, 'f', 1);
}

// zuluTime -> KML <when>/<TimeStamp> UTC timestamp string, empty if unparseable.
QString whenFromZulu(const QString& zuluTime) {
	const QDateTime dt = parseZuluTime(zuluTime);
	return dt.isValid() ? dt.toUTC().toString(Qt::ISODateWithMs) : QString();
}

// One "<b>key:</b> value<br/>" line for a CDATA-wrapped <description> body --
// content lands inside a CDATA section, so it's written as literal HTML/text,
// not XML-escaped (that would visibly double-escape "&", "<", ">").
QString descRow(const QString& key, const QString& value) {
	return QStringLiteral("<b>%1:</b> %2<br/>").arg(key, value);
}

QString airportLabel(const QString& icao, const QString& airportName) {
	if (icao.isEmpty())
		return QString();
	return airportName.isEmpty() ? icao : QStringLiteral("%1 (%2)").arg(icao, airportName);
}

// Shared by liftoff/touchdown descriptions: airport/runway/airspeed/V-S
// fields both popups show. gForce/pitch/bank/heading are appended by the caller.
QString facilityDescriptionCommon(const QString& icao, const QString& airportName, const QString& runway,
	int runwayHeading, int airspeed, int verticalSpeed) {
	QString html;
	const QString airport = airportLabel(icao, airportName);
	if (!airport.isEmpty())
		html += descRow(QStringLiteral("Airport"), airport);
	if (!runway.isEmpty()) {
		const QString rwy = runwayHeading >= 0 ? QStringLiteral("%1 (%2°)").arg(runway).arg(runwayHeading) : runway;
		html += descRow(QStringLiteral("Runway"), rwy);
	}
	html += descRow(QStringLiteral("Airspeed"), QStringLiteral("%1 kt").arg(airspeed));
	html += descRow(QStringLiteral("V/S"), QStringLiteral("%1 ft/min").arg(verticalSpeed));
	return html;
}

QString thresholdAndWindDescription(const QString& runway, double distanceLength, double distanceWidth,
	double distanceLengthPercent, double distanceWidthPercent, int windDirection, int windVelocity) {
	QString html;
	// Gated on a matched runway, not "distanceLength >= 0": a NULL distance_length
	// column (no runway matched) reads back as 0 via sqlite3_column_double, which
	// would otherwise show a meaningless "0 ft (0%)" as if a runway had been found.
	if (!runway.isEmpty()) {
		html += descRow(QStringLiteral("Threshold"),
			QStringLiteral("%1 ft (%2%)").arg(qRound(distanceLength)).arg(qRound(distanceLengthPercent * 100)));
		const QChar side = distanceWidth >= 0 ? QChar('R') : QChar('L');
		html += descRow(QStringLiteral("Centerline"),
			QStringLiteral("%1 ft %2 (%3%)").arg(qRound(qAbs(distanceWidth))).arg(side).arg(qRound(qAbs(distanceWidthPercent) * 100)));
	}
	html += descRow(QStringLiteral("Wind"), QStringLiteral("%1° / %2 kt").arg(windDirection).arg(windVelocity));
	return html;
}

// Shared by liftoff/touchdown descriptions.
QString attitudeDescription(double pitchDegrees, double bankDegrees, int headingDegrees) {
	QString html;
	html += descRow(QStringLiteral("Pitch"), QStringLiteral("%1°").arg(pitchDegrees, 0, 'f', 1));
	html += descRow(QStringLiteral("Bank"), QStringLiteral("%1°").arg(bankDegrees, 0, 'f', 1));
	html += descRow(QStringLiteral("Heading"), QStringLiteral("%1°").arg(headingDegrees));
	return html;
}

// Shared by liftoff/touchdown descriptions.
QString timestampDescription(const QString& zuluTime, const QString& localTime) {
	QString html = descRow(QStringLiteral("Zulu"), zuluTime);
	if (!localTime.isEmpty())
		html += descRow(QStringLiteral("Local"), localTime);
	return html;
}

QString liftoffDescription(const LiftoffPoint& t) {
	QString html = facilityDescriptionCommon(t.icao, t.airportName, t.runway, t.runwayHeading, t.airspeed, t.verticalSpeed);
	html += attitudeDescription(t.pitchDegrees, t.bankDegrees, t.headingDegrees);
	html += thresholdAndWindDescription(t.runway, t.distanceLength, t.distanceWidth,
		t.distanceLengthPercent, t.distanceWidthPercent, t.windDirection, t.windVelocity);
	html += timestampDescription(t.zuluTime, t.localTime);
	return html;
}

QString touchdownDescription(const TouchdownPoint& t) {
	QString html = facilityDescriptionCommon(t.icao, t.airportName, t.runway, t.runwayHeading, t.airspeed, t.verticalSpeed);
	html += descRow(QStringLiteral("G-Force"), QStringLiteral("%1 G").arg(t.gForce, 0, 'f', 2));
	html += attitudeDescription(t.pitchDegrees, t.bankDegrees, t.headingDegrees);
	html += thresholdAndWindDescription(t.runway, t.distanceLength, t.distanceWidth,
		t.distanceLengthPercent, t.distanceWidthPercent, t.windDirection, t.windVelocity);
	html += timestampDescription(t.zuluTime, t.localTime);
	return html;
}

// One or more TripEvents resolved to (nearly) the same position, combined
// into a single placemark -- see groupNearbyEvents() below.
struct EventGroup {
	double latitude = 0;
	double longitude = 0;
	std::vector<const TripEvent*> events;
};

// Groups events within ~11 m of each other (same or adjacent sample) into one
// placemark, mirroring resources/map.html's setEvents(). Without this,
// closely-timed events resolve to the same/adjacent sample position and their
// <name> labels stack illegibly on top of each other in Google Earth.
std::vector<EventGroup> groupNearbyEvents(const std::vector<TripEvent>& events) {
	constexpr double kThreshold = 0.0001;
	std::vector<EventGroup> groups;
	for (const TripEvent& e : events) {
		bool merged = false;
		for (EventGroup& g : groups) {
			if (qAbs(g.latitude - e.latitude) < kThreshold && qAbs(g.longitude - e.longitude) < kThreshold) {
				g.events.push_back(&e);
				merged = true;
				break;
			}
		}
		if (!merged)
			groups.push_back(EventGroup{ e.latitude, e.longitude, { &e } });
	}
	return groups;
}

// Appends one Point <Placemark> -- name/styleUrl, an optional <TimeStamp>
// (skipped if zuluTime doesn't parse), a CDATA <description>, and a
// clampToGround <Point>. Shared by the Liftoffs/Touchdowns/Events folder
// loops below, which differ only in the per-point values passed in here.
void appendPointPlacemark(QString& kml, const QString& name, const QString& styleUrl,
	const QString& zuluTime, const QString& descriptionHtml, double longitude, double latitude) {
	kml += QStringLiteral("<Placemark><name>%1</name><styleUrl>%2</styleUrl>").arg(xmlEscape(name), styleUrl);
	const QString when = whenFromZulu(zuluTime);
	if (!when.isEmpty())
		kml += QStringLiteral("<TimeStamp><when>%1</when></TimeStamp>").arg(when);
	kml += QStringLiteral("<description><![CDATA[%1]]></description>").arg(descriptionHtml);
	kml += QStringLiteral("<Point><altitudeMode>clampToGround</altitudeMode><coordinates>%1</coordinates></Point>")
		.arg(formatCoordinate(longitude, latitude, 0.0));
	kml += QStringLiteral("</Placemark>\n");
}

// "KJFK-KLAX" / "KJFK" / "Trip <id>" -- shared airportPairName() derivation
// (trip_dataset.h), also used for the default Save Image/KML file name.
QString deriveTripName(const TripDataset& dataset) {
	return airportPairName(dataset.liftoffPoints, dataset.touchdowns, QStringLiteral("Trip %1").arg(dataset.tripId));
}

QString buildTripKml(const TripDataset& dataset) {
	QString kml;
	// Trajectory coordinates/track dominate the output for a long flight
	// (tens of thousands of points, each contributing a handful of tags) --
	// reserving generously up front avoids repeated reallocation/copying as
	// the string grows.
	kml.reserve(2000 + (int)dataset.points.size() * 160
		+ (int)(dataset.liftoffPoints.size() + dataset.touchdowns.size() + dataset.events.size()) * 512);

	kml += QStringLiteral(
		"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
		"<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n"
		"<Document>\n");
	kml += QStringLiteral("<name>%1</name>\n").arg(xmlEscape(deriveTripName(dataset)));

	// No custom Icon hrefs -- those would point at maps.google.com/earth.google.com
	// image URLs, which only Google Earth is guaranteed to fetch and which need
	// internet access to load at all. <color> alone tints each viewer's own
	// default marker/pushpin instead, keeping liftoff/touchdown/event visually
	// distinct without depending on an externally hosted image.
	kml += QStringLiteral(
		"<Style id=\"pathStyle\"><LineStyle><color>ff0000ff</color><width>3</width></LineStyle></Style>\n"
		"<Style id=\"trackStyle\"><LineStyle><color>ff0000ff</color><width>3</width></LineStyle></Style>\n"
		"<Style id=\"liftoffStyle\"><IconStyle><color>ff00cc00</color></IconStyle></Style>\n"
		"<Style id=\"touchdownStyle\"><IconStyle><color>ff0000ff</color></IconStyle></Style>\n"
		"<Style id=\"eventStyle\"><IconStyle><color>ff00aaff</color><scale>0.8</scale></IconStyle></Style>\n");

	if (!dataset.points.empty()) {
		kml += QStringLiteral("<Folder><name>Trajectory</name>\n");

		// Static 3D path.
		kml += QStringLiteral("<Placemark><name>Flight Path</name><styleUrl>#pathStyle</styleUrl>"
			"<LineString><altitudeMode>absolute</altitudeMode><coordinates>");
		for (const TripSamplePoint& p : dataset.points) {
			kml += formatCoordinate(p.longitude, p.latitude, p.altitude * kFeetToMeters);
			kml += QLatin1Char(' ');
		}
		kml += QStringLiteral("</coordinates></LineString></Placemark>\n");

		// Time-animated track -- one <when>/<gx:coord> pair per point (skipping
		// any point whose zuluTime doesn't parse, keeping the two lists paired).
		kml += QStringLiteral("<Placemark><name>Flight Track</name><styleUrl>#trackStyle</styleUrl>"
			"<gx:Track><altitudeMode>absolute</altitudeMode>\n");
		QString coordLines;
		coordLines.reserve((int)dataset.points.size() * 40);
		for (const TripSamplePoint& p : dataset.points) {
			const QString when = whenFromZulu(p.zuluTime);
			if (when.isEmpty())
				continue;
			kml += QStringLiteral("<when>%1</when>\n").arg(when);
			coordLines += QStringLiteral("<gx:coord>%1</gx:coord>\n").arg(formatGxCoordinate(p.longitude, p.latitude, p.altitude * kFeetToMeters));
		}
		kml += coordLines;
		kml += QStringLiteral("</gx:Track></Placemark>\n</Folder>\n");
	}

	if (!dataset.liftoffPoints.empty()) {
		kml += QStringLiteral("<Folder><name>Liftoffs</name>\n");
		for (const LiftoffPoint& t : dataset.liftoffPoints)
			appendPointPlacemark(kml, QStringLiteral("Liftoff"), QStringLiteral("#liftoffStyle"),
				t.zuluTime, liftoffDescription(t), t.longitude, t.latitude);
		kml += QStringLiteral("</Folder>\n");
	}

	if (!dataset.touchdowns.empty()) {
		kml += QStringLiteral("<Folder><name>Touchdowns</name>\n");
		for (const TouchdownPoint& t : dataset.touchdowns)
			appendPointPlacemark(kml, QStringLiteral("Touchdown"), QStringLiteral("#touchdownStyle"),
				t.zuluTime, touchdownDescription(t), t.longitude, t.latitude);
		kml += QStringLiteral("</Folder>\n");
	}

	if (!dataset.events.empty()) {
		kml += QStringLiteral("<Folder><name>Events</name>\n");
		for (const EventGroup& g : groupNearbyEvents(dataset.events)) {
			const QString name = g.events.size() == 1
				? g.events.front()->event
				: QStringLiteral("Events (%1)").arg(g.events.size());
			QString desc;
			for (const TripEvent* e : g.events)
				desc += descRow(e->event, e->zuluTime);
			appendPointPlacemark(kml, name, QStringLiteral("#eventStyle"),
				g.events.front()->zuluTime, desc, g.longitude, g.latitude);
		}
		kml += QStringLiteral("</Folder>\n");
	}

	kml += QStringLiteral("</Document>\n</kml>\n");
	return kml;
}

}

bool exportTripDatasetToKmlFile(const TripDataset& dataset, const QString& fileName, QString* errorMessage) {
	QElapsedTimer t; t.start();
	const QString kml = buildTripKml(dataset);
	Logger::logf(Logger::Profile, "KML", "buildTripKml: %lld ms  (%zu pts, %zu liftoffs, %zu touchdowns, %zu events)",
		t.nsecsElapsed() / 1000000, dataset.points.size(), dataset.liftoffPoints.size(), dataset.touchdowns.size(), dataset.events.size());

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
		if (errorMessage) *errorMessage = file.errorString();
		return false;
	}
	const QByteArray bytes = kml.toUtf8();
	const qint64 written = file.write(bytes);
	const QString writeError = file.errorString();
	file.close();
	if (written != bytes.size()) {
		if (errorMessage) *errorMessage = written < 0 ? writeError : QStringLiteral("Disk full or write error (file may be incomplete).");
		return false;
	}
	return true;
}

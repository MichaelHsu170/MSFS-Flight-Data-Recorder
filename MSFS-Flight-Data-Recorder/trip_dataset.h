#pragma once

#include <QString>
#include <utility>
#include <vector>

// One row of trip_data, decoded into engineering units/booleans. Same shape
// whether it came from db_history::queryTripData() (history) or was appended
// live from a FLIGHT_DATA_RECORD while recording (Recording feature) — the
// Trajectory View doesn't care which.
struct TripSamplePoint {
	double latitude = 0;
	double longitude = 0;
	int altitude = 0;
	int airspeed = 0;
	int groundSpeed = 0;
	int verticalSpeed = 0;
	double n1_1 = 0;
	double n1_2 = 0;
	double n2_1 = 0;
	double n2_2 = 0;
	double gearHandlePosition = 0;
	int gearPosition[3] = { 0, 0, 0 };
	bool gearOnGround[3] = { false, false, false };
	int brakeIndicator = 0;
	double flapsHandleIndex = 0;
	double spoilersHandlePosition = 0;
	double fuelTotalQuantityWeight = 0;
	QString zuluTime;
	QString localTime;

	// Every other trip_data column, label -> formatted value, in a fixed order
	// shared by both producers (recorder_bridge.cpp's live path and
	// db_history.cpp's historical path build this from the same X-macro list
	// in trip_data_fields.h) so DataTablePanel can render "every field" without
	// caring which path the point came from.
	std::vector<std::pair<QString, QString>> allFields;
};

// One landing logged during the trip (trip_touchdowns), shown as a marker on
// the map. A trip can have more than one touchdown (go-arounds, bounces).
struct TouchdownPoint {
	double latitude = 0;
	double longitude = 0;
	QString icao;
	QString runway;
	int airspeed = 0;
	int verticalSpeed = 0;
	double gForce = 0;
	double pitchDegrees = 0;
	double bankDegrees = 0;
	int headingDegrees = 0;
	QString airportName;
	double distanceLength = -1;   // feet from threshold, -1 = unknown
	double distanceWidth = 0;     // feet from centerline (+right/-left)
	double distanceLengthPercent = -1;  // 0-1 fraction of runway length
	double distanceWidthPercent = 0;    // 0-1 fraction of runway half-width
	int windDirection = 0;   // degrees true, 0 if unknown
	int windVelocity = 0;    // knots
	QString zuluTime;
	QString localTime;
};

// One discrete cockpit event (gear/flaps/spoilers/etc. toggled) logged during
// the trip (trip_events). trip_events only stores a timestamp, not a
// position -- latitude/longitude/sampleIndex are resolved after loading by
// matching zuluTime against the nearest TripSamplePoint (see
// TripHistoryPanel::onRowActivated).
struct TripEvent {
	QString event;
	QString zuluTime;
	double latitude = 0;
	double longitude = 0;
	int sampleIndex = -1;
};

struct TripDataset {
	int tripId = -1;
	QString aircraftTitle;  // trips.title — human-readable name like "Airbus A320neo FlyByWire"
	std::vector<TripSamplePoint> points;
	std::vector<TouchdownPoint> touchdowns;
	std::vector<TripEvent> events;
};

enum class TripStatus {
	Completed,
	Open,
	Live,
};

struct TripSummary {
	int id = 0;
	QString title;
	QString atcAirline;
	QString atcFlightNumber;
	QString departureIcao;
	QString departureName;
	QString departureRegion;
	QString departureRwy;
	QString destinationIcao;
	QString destinationName;
	QString destinationRegion;
	QString destinationRwy;
	QString departureZuluTime;
	QString destinationZuluTime;
	TripStatus status = TripStatus::Completed;
};

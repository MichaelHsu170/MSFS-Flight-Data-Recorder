#pragma once

#include "fake_simconnect.h"
#include "recorder_bridge.h"

#include <QList>
#include <QString>
#include <QVariant>
#include <QVariantMap>
#include <QWidget>

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

// Shared helpers for the test executables: file isolation, inbound SimConnect
// packet builders, a made-up airport "world" that answers the recorder's
// facility lookups, and FlightDriver, which pushes made-up sim data through a
// real RecorderBridge wired to the fake SimConnect.
namespace TestSupport {

// Switches the working directory to a fresh temporary directory (once per
// process) and deletes flight_data.db/settings.ini at the paths the app
// itself resolves (resolve_db_path(), AppSettings::filePath()).
void isolateFiles();
void removeDatabase();
void removeSettings();

// Polls cond (processing Qt events in between) until it returns true or
// timeoutMs elapses. Returns cond()'s final value.
bool waitFor(const std::function<bool()>& cond, int timeoutMs = 10000);

// Runs action on the next modal dialog or popup menu to appear (e.g. one the
// code under test opens with exec()), from the event loop that exec() runs.
// Polls for up to 5 s; action receives the dialog/menu widget.
void onNextModal(const std::function<void(QWidget*)>& action);
// Chooses the action with this text from a popup QMenu (via the keyboard, so
// QMenu::exec() returns it like a real click would).
void chooseMenuItem(QWidget* menu, const QString& text);
// Clicks the button with this text in a dialog (QMessageBox, QInputDialog...).
void clickDialogButton(QWidget* dialog, const QString& text);

// Runs sql on a fresh read-only connection to flight_data.db.
QList<QVariantMap> queryRows(const QString& sql);
QVariant queryValue(const QString& sql);

// A sample with every field zeroed except what the recorder needs to treat
// the aircraft as sitting in a loaded flight: on the ground at the test
// airport, engines off, 2026-01-02 10:00:00Z.
FLIGHT_DATA_RECORD makeRecord();

std::vector<char> recvPacket(DWORD id, size_t size);
std::vector<char> eventPacket(DWORD eventId, DWORD data);
std::vector<char> samplePacket(const FLIGHT_DATA_RECORD& record);
std::vector<char> exceptionPacket(DWORD exception, DWORD sendId);

struct RunwaySpec {
	double latitude = 0;
	double longitude = 0;
	float heading = 0;        // true heading of the primary end, degrees
	float lengthM = 3000;
	float widthM = 45;
	int primaryNumber = 9;
	int secondaryNumber = 27;
	int primaryDesignator = 0; // 0 none, 1 L, 2 R, 3 C, ...
	int secondaryDesignator = 0;
	float primaryThresholdM = 0;
	float secondaryThresholdM = 0;
	int thresholdEnable = 0;   // PAVEMENT ENABLE flag for both ends
	bool sendPavement = true;  // whether PAVEMENT child records are sent at all
};

struct AirportSpec {
	std::string ident;
	std::string region;
	std::string name;
	double latitude = 0;
	double longitude = 0;
	float magvar = 0;
	std::vector<RunwaySpec> runways;
};

std::vector<char> airportListPacket(const std::vector<AirportSpec>& airports, DWORD entryNumber, DWORD outOf);
std::vector<char> facilityAirportPacket(const AirportSpec& airport);
std::vector<char> facilityRunwayPacket(DWORD itemIndex, DWORD uniqueRequestId, const RunwaySpec& runway);
std::vector<char> facilityPavementPacket(DWORD parentUniqueRequestId, float lengthM, float widthM, int enable);
std::vector<char> facilityEndPacket();

// The point distanceM along the runway's primary heading from its primary
// (start_points[0]) end, offset rightM to the right of the centerline --
// computed with the app's own COORDINATE geometry (types.h).
COORDINATE pointOnRunway(const RunwaySpec& runway, double distanceM, double rightM = 0);

class FlightDriver {
public:
	// Resets the fake, creates a RecorderBridge (which connects immediately)
	// and delivers the OPEN packet plus "Sim running".
	FlightDriver();
	~FlightDriver();

	RecorderBridge& bridge() { return *bridge_; }
	STATUS& status() { return *bridge_->status(); }

	// The sim state sent by tick(); edit freely between ticks.
	FLIGHT_DATA_RECORD record;

	// Runs one RecorderBridge::pollDispatch(), delivering queued packets.
	void pump();
	void send(std::vector<char> packet);
	void simEvent(DWORD eventId, DWORD data = 1);
	// Advances both clocks by seconds, sends record as one sample, pumps.
	void tick(double seconds = 0.5);
	void ticks(int count, double seconds = 0.5);

	void setOnGround(bool onGround);
	void setEngines(bool running);
	void moveTo(const COORDINATE& position);
	void setHeading(double magneticDegrees);

	// Engines on while on the ground, one tick: starts a trip. Returns its id.
	int startTrip();
	// Engines off on the ground, one tick, then waits for tripEnded.
	void endTrip();

	// Answers every facility request the recorder has made so far (and any it
	// makes while being answered) from airports.
	std::vector<AirportSpec> airports;
	int airportListChunkSize = 0; // 0 = whole list in one AIRPORT_LIST packet
	// SendIDs of facility-list requests the "sim" rejected with an exception:
	// never answered, like the real simulator.
	std::set<DWORD> rejectedRequests;
	void serviceLookups();

private:
	std::unique_ptr<RecorderBridge> bridge_;
	size_t listRequestsServed_ = 0;
	size_t dataRequestsServed_ = 0;
};

}

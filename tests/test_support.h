#pragma once

#include "fake_simconnect.h"
#include "recorder_bridge.h"

#include <QList>
#include <QSignalSpy>
#include <QString>
#include <QStringList>
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
// itself resolves (db_file_path(), AppSettings::filePath()).
void isolateFiles();
void removeDatabase();
void removeSettings();

// Polls cond (processing Qt events in between) until it returns true or
// timeoutMs elapses. Returns cond()'s final value.
bool waitFor(const std::function<bool()>& cond, int timeoutMs = 10000);

// Whether one of lines contains every one of parts. Tests match a message's
// key facts (ids, counts, names), not its exact wording.
bool anyLineWith(const QStringList& lines, const QStringList& parts);
// The last line in log (a spy on a QString signal such as
// RecorderBridge::logMessage) that contains every one of parts, or an empty
// string.
QString lastLogWith(const QSignalSpy& log, const QStringList& parts);
// Whether the log file at logPath (one the test passed to Logger::init) has a
// line of level levelTag (as the file shows it: "FATAL", "WARN ", ...) that
// contains every one of parts.
bool lineLogged(const QString& logPath, const char* levelTag, const QStringList& parts);
// lineLogged() for a Warning line.
bool warningLogged(const QString& logPath, const QStringList& parts);

// Runs js on the page of the QWebEngineView inside owner (e.g. a MapWidget)
// and waits up to 5 s for its result; an invalid QVariant if none came.
QVariant evalPageJs(QWidget* owner, const QString& js);
// The map page's Leaflet zoom level and the point count of its trajectory
// line, read back from the page itself.
int mapZoom(QWidget* owner);
int mapTrajectoryPointCount(QWidget* owner);
// Whether the map page shows the box from (lat1, lng1) to (lat2, lng2) the
// way its fit does once it has settled: centred, at the closest zoom that
// fits it inside 20 px of padding. Polled with QTRY_VERIFY to wait out the
// fit's animation.
bool mapFittedTo(QWidget* owner, double lat1, double lng1, double lat2, double lng2);
// How many elements with this CSS class (a marker's divIcon class) the map
// page shows.
int mapElementCount(QWidget* owner, const char* className);

// Runs action on the next modal dialog or popup menu to appear (e.g. one the
// code under test opens with exec()), from the event loop that exec() runs.
// Polls for up to 5 s; action receives the dialog/menu widget.
void onNextModal(const std::function<void(QWidget*)>& action);
// Drops every onNextModal() action that hasn't run yet: its captures may not
// outlive the test that set it up (e.g. one that failed before the dialog
// came). Call from the test class's cleanup().
void cancelPendingModals();
// Chooses the action with this text from a popup QMenu (via the keyboard, so
// QMenu::exec() returns it like a real click would).
void chooseMenuItem(QWidget* menu, const QString& text);
// Clicks the button with this text in a dialog (QMessageBox, QInputDialog...).
void clickDialogButton(QWidget* dialog, const QString& text);
// Saves to path from a Qt file dialog (QFileDialog::getSaveFileName() with
// Qt::AA_DontUseNativeDialogs set; a native dialog can't be answered) and
// returns the file name it suggested; an empty string if dialog isn't one.
QString saveFileDialogAs(QWidget* dialog, const QString& path);
// Sends w a left-button press or release (type) at pos, in w's coordinates,
// with the button state a real click has at that point.
void sendLeftButton(QWidget* w, QEvent::Type type, const QPoint& pos);

// Puts back what was on the clipboard (on the desktop, the user's own) when
// it goes out of scope, so a test can use the clipboard and fail midway.
class ClipboardGuard {
public:
	ClipboardGuard();
	~ClipboardGuard();
	ClipboardGuard(const ClipboardGuard&) = delete;
	ClipboardGuard& operator=(const ClipboardGuard&) = delete;
private:
	QList<QPair<QString, QByteArray>> saved_; // format, data
};

// Runs sql on a fresh read-only connection to flight_data.db.
QList<QVariantMap> queryRows(const QString& sql);
QVariant queryValue(const QString& sql);
// The trips row with this id, column name -> value. A missing row fails the
// current test, so a NULL column can't pass for a trip that isn't there.
QVariantMap tripRow(int id);
// Opens flight_data.db read-write, creating it if missing (unlike the app's
// connect_db_readwrite()), so a test can build a database in any state;
// nullptr if that fails. Caller must sqlite3_close().
sqlite3* openDatabaseFile();
// Runs sql on db; a failure fails the current test.
void exec(sqlite3* db, const char* sql);
// Runs sql on a fresh read-write connection to flight_data.db (one beside the
// code under test's own); a failure fails the current test.
void exec(const char* sql);
// Adds a trips row titled "Trip <id>" with only the required columns and
// these filled, through exec(): groupId 0 is no group, a null
// destinationZulu an open trip.
void addTrip(int id, int groupId = 0, const char* departureZulu = "z", const char* destinationZulu = nullptr);
// Adds a group through the app's insertGroup() on a fresh read-write
// connection and returns its id; a failure fails the current test.
int addGroup(const char* name);
// Creates flight_data.db with only a trip_data from before engine_speed/
// engine_load: N1/N2 of engines 1-2 in their own columns, 5 rows.
void createLegacyTripData();
// Adds count twin-jet rows of trip 2 to createLegacyTripData()'s table.
void addLegacyJetRows(int count);
// Creates flight_data.db with only a trip_data from before trip_engine_data,
// 5 rows of trip 1: engine power in the engine_speed/engine_load BLOBs,
// eng_oil_pressure of engines 1-2 in their own columns, and eng_failed and
// general_eng_starter of engines 1-2 as bool_group bits (row 1 only).
void createBlobTripData();

// A sample with every field zeroed except what the recorder needs to treat
// the aircraft as sitting in a loaded flight: on the ground at the test
// airport, a twin with its engines off, 2026-01-02 10:00:00Z.
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
	// Resets the fake, creates and starts a RecorderBridge (which connects)
	// and delivers the OPEN packet plus "Sim running". The event flood
	// filter's clock is replaced by one that only tick() moves.
	FlightDriver();
	// Destroys the bridge first: its event filter's clock reads eventClock_,
	// which the default member order would destroy before it.
	~FlightDriver();

	RecorderBridge& bridge() { return *bridge_; }
	STATUS& status() { return *bridge_->status(); }

	// The sim state sent by tick(); edit freely between ticks.
	FLIGHT_DATA_RECORD record;

	// Runs one RecorderBridge::pollDispatch(), delivering queued packets.
	void pump();
	void send(std::vector<char> packet);
	void simEvent(DWORD eventId, DWORD data = 1);
	// Advances the sim clocks in record and the event filter's clock by
	// seconds, sends record as one sample, pumps.
	void tick(double seconds = 0.5);
	void ticks(int count, double seconds = 0.5);

	void setOnGround(bool onGround);
	// Sets ENG COMBUSTION of all SIM_ENGINE_INDEXES engines.
	void setEngines(bool running);
	void moveTo(const COORDINATE& position);
	void setHeading(double magneticDegrees);

	// Engines on while on the ground, one tick: starts a trip. Returns its id.
	int startTrip();
	// Engines off on the ground, one tick, then waits until the trip's last
	// samples are written (RecorderBridge::isTripFlushing()).
	void endTrip();

	// Answers every facility request the recorder has made so far (and any it
	// makes while being answered) from airports; fails the test if the
	// requests never stop coming.
	std::vector<AirportSpec> airports;
	int airportListChunkSize = 0; // 0 = whole list in one AIRPORT_LIST packet
	// SendIDs of facility-list requests the "sim" rejected with an exception:
	// never answered, like the real simulator.
	std::set<DWORD> rejectedRequests;
	void serviceLookups();

private:
	std::unique_ptr<RecorderBridge> bridge_;
	EventFloodFilter::TimePoint eventClock_ = std::chrono::steady_clock::now();
	size_t listRequestsServed_ = 0;
	size_t dataRequestsServed_ = 0;
};

}

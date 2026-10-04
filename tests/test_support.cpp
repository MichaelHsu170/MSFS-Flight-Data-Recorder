#include "test_support.h"

#include "app_settings.h"
#include "db.h"

#include <QAbstractButton>
#include <QApplication>
#include <QCoreApplication>
#include <QMenu>
#include <QTimer>
#include <QtTest>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QMetaObject>
#include <QTemporaryDir>
#include <QThread>
#include <QWebEnginePage>
#include <QWebEngineView>

#include <algorithm>
#include <cstring>

namespace TestSupport {

void isolateFiles() {
	static QTemporaryDir dir;
	QDir::setCurrent(dir.path());
	removeDatabase();
	removeSettings();
}

void removeDatabase() {
	QFile::remove(QString::fromStdString(db_file_path()));
}

void removeSettings() {
	QFile::remove(AppSettings::filePath());
}

bool waitFor(const std::function<bool()>& cond, int timeoutMs) {
	QElapsedTimer timer;
	timer.start();
	while (!cond()) {
		if (timer.elapsed() > timeoutMs)
			return cond();
		QCoreApplication::processEvents();
		QThread::msleep(5);
	}
	return true;
}

QString lastLogWith(const QSignalSpy& log, const QStringList& parts) {
	QString found;
	for (const QList<QVariant>& args : log) {
		const QString line = args.value(0).toString();
		if (std::all_of(parts.begin(), parts.end(), [&line](const QString& part) { return line.contains(part); }))
			found = line;
	}
	return found;
}

bool warningLogged(const QString& logPath, const QStringList& parts) {
	QFile f(logPath);
	if (!f.open(QIODevice::ReadOnly))
		return false;
	const QStringList lines = QString::fromUtf8(f.readAll()).split('\n');
	return std::any_of(lines.begin(), lines.end(), [&parts](const QString& line) {
		return line.contains(QStringLiteral("[WARN ]"))
			&& std::all_of(parts.begin(), parts.end(), [&line](const QString& part) { return line.contains(part); });
	});
}

QVariant evalPageJs(QWidget* owner, const QString& js) {
	// Shared so a callback arriving after the timeout writes somewhere valid.
	auto result = std::make_shared<std::pair<bool, QVariant>>(false, QVariant());
	owner->findChild<QWebEngineView*>()->page()->runJavaScript(js,
		[result](const QVariant& v) { *result = { true, v }; });
	if (!waitFor([result] { return result->first; }, 5000))
		return QVariant();
	return result->second;
}

int mapZoom(QWidget* owner) {
	return evalPageJs(owner, QStringLiteral("leafletMapInstance.getZoom()")).toInt();
}

// The trajectory is the blue polyline (canvas-drawn, so not in the DOM):
// runs body with it as l, -1/null if absent.
static QVariant evalOnTrajectory(QWidget* owner, const char* body) {
	return evalPageJs(owner, QStringLiteral(
		"(function(){var r=null;leafletMapInstance.eachLayer(function(l){"
		"if(l instanceof L.Polyline&&l.options.color==='blue'){var a=l.getLatLngs();%1}});return r;})()")
		.arg(QLatin1String(body)));
}

int mapTrajectoryPointCount(QWidget* owner) {
	const QVariant v = evalOnTrajectory(owner, "r=a.length;");
	return v.isValid() ? v.toInt() : -1;
}

int mapElementCount(QWidget* owner, const char* className) {
	return evalPageJs(owner, QStringLiteral("document.querySelectorAll('.%1').length")
		.arg(QLatin1String(className))).toInt();
}

void onNextModal(const std::function<void(QWidget*)>& action) {
	auto* timer = new QTimer();
	auto* elapsed = new QElapsedTimer();
	elapsed->start();
	QObject::connect(timer, &QTimer::timeout, [timer, elapsed, action] {
		QWidget* target = QApplication::activePopupWidget();
		if (!target)
			target = QApplication::activeModalWidget();
		if (!target && elapsed->elapsed() < 5000)
			return;
		timer->stop();
		timer->deleteLater();
		delete elapsed;
		if (target)
			action(target);
	});
	timer->start(10);
}

void chooseMenuItem(QWidget* menuWidget, const QString& text) {
	QMenu* menu = qobject_cast<QMenu*>(menuWidget);
	if (!menu)
		return;
	for (QAction* action : menu->actions()) {
		if (action->text() == text) {
			menu->setActiveAction(action);
			QTest::keyClick(menu, Qt::Key_Return);
			return;
		}
	}
	menu->close();
}

void clickDialogButton(QWidget* dialog, const QString& text) {
	for (QAbstractButton* button : dialog->findChildren<QAbstractButton*>()) {
		if (button->text().remove('&') == text) {
			button->click();
			return;
		}
	}
}

QList<QVariantMap> queryRows(const QString& sql) {
	QList<QVariantMap> rows;
	sqlite3* db = connect_db_readonly();
	if (!db)
		return rows;
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(db, sql.toUtf8().constData(), -1, &stmt, nullptr) == SQLITE_OK) {
		while (sqlite3_step(stmt) == SQLITE_ROW) {
			QVariantMap row;
			for (int i = 0; i < sqlite3_column_count(stmt); ++i) {
				const QString name = QString::fromUtf8(sqlite3_column_name(stmt, i));
				switch (sqlite3_column_type(stmt, i)) {
				case SQLITE_INTEGER: row[name] = (qlonglong)sqlite3_column_int64(stmt, i); break;
				case SQLITE_FLOAT: row[name] = sqlite3_column_double(stmt, i); break;
				case SQLITE_NULL: row[name] = QVariant(); break;
				default: row[name] = QString::fromUtf8(reinterpret_cast<const char*>(sqlite3_column_text(stmt, i))); break;
				}
			}
			rows.append(row);
		}
	}
	sqlite3_finalize(stmt);
	sqlite3_close(db);
	return rows;
}

void exec(sqlite3* db, const char* sql) {
	char* err = nullptr;
	if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
		const QString message = QString::fromUtf8(err ? err : "?");
		sqlite3_free(err);
		QFAIL(qPrintable(message + " in: " + sql));
	}
}

void exec(const char* sql) {
	sqlite3* db = connect_db_readwrite();
	QVERIFY(db);
	exec(db, sql);
	sqlite3_close(db);
}

void addTrip(int id, int groupId, const char* departureZulu, const char* destinationZulu) {
	const QString sql = QStringLiteral("INSERT INTO trips (id,title,atc_airline,atc_flight_number,atc_id,atc_model,atc_type,"
		"departure_latitude,departure_longitude,departure_zulu_time,departure_local_time,destination_zulu_time,group_id) "
		"VALUES (%1,'Trip %1','A','1','I','M','T',0,0,'%2','l',%3,%4);")
		.arg(id).arg(departureZulu).arg(destinationZulu ? QStringLiteral("'%1'").arg(destinationZulu) : QStringLiteral("NULL"))
		.arg(groupId ? QString::number(groupId) : QStringLiteral("NULL"));
	exec(sql.toUtf8().constData());
}

void createLegacyTripData() {
	sqlite3* db = nullptr;
	QCOMPARE(sqlite3_open(db_file_path().c_str(), &db), SQLITE_OK);
	// retired_field stands for a column the current code no longer names.
	exec(db, "CREATE TABLE trip_data (trip INTEGER NOT NULL, retired_field REAL,"
		" engine_type INTEGER NOT NULL, number_of_engines INTEGER NOT NULL,"
		" turb_eng_n1_1 REAL NOT NULL, turb_eng_n1_2 REAL NOT NULL, turb_eng_n2_1 REAL NOT NULL, turb_eng_n2_2 REAL NOT NULL);");
	exec(db, "INSERT INTO trip_data (trip, retired_field, engine_type, number_of_engines, turb_eng_n1_1, turb_eng_n1_2, turb_eng_n2_1, turb_eng_n2_2) VALUES"
		" (1, 10, 1, 2, 85.5, 90.25, 95, 96.5),"  // twin jet
		" (1, 20, 1, 1, 85.5, 0, 95, 0),"         // single jet
		" (1, 30, 1, 4, 85.5, 90.25, 95, 96.5),"  // quad jet: only engines 1-2 were recorded
		" (1, 40, 1, 0, 85.5, 90.25, 95, 96.5),"  // no engines
		" (1, 50, 0, 1, 85.5, 0, 95, 0);");       // piston: its N1/N2 meant nothing
	sqlite3_close(db);
}

void addLegacyJetRows(int count) {
	exec(QByteArray("WITH RECURSIVE n(i) AS (SELECT 1 UNION ALL SELECT i + 1 FROM n WHERE i < ")
		.append(QByteArray::number(count)).append(")"
		" INSERT INTO trip_data (trip, engine_type, number_of_engines, turb_eng_n1_1, turb_eng_n1_2, turb_eng_n2_1, turb_eng_n2_2)"
		" SELECT 2, 1, 2, 85.5, 90.25, 95, 96.5 FROM n;").constData());
}

QVariant queryValue(const QString& sql) {
	const QList<QVariantMap> rows = queryRows(sql);
	if (rows.isEmpty() || rows.first().isEmpty())
		return QVariant();
	return rows.first().first();
}

FLIGHT_DATA_RECORD makeRecord() {
	FLIGHT_DATA_RECORD r;
	memset(static_cast<void*>(&r), 0, sizeof(r));
	r.surface_type = 4;
	r.sim_on_ground = 1;
	r.number_of_engines = 2;
	r.plane_coordinate.latitude = 43.0;
	r.plane_coordinate.longitude = 1.0;
	r.plane_heading_degrees_magnetic = 90;
	for (DATETIME* t : { &r.time_zulu, &r.time_local }) {
		t->year = 2026;
		t->month_of_year = 1;
		t->day_of_month = 2;
		t->day_of_week = 5;
		t->time_day = 36000;
		t->timezone_offset = 0;
	}
	strcpy(r.title, "Test Aircraft");
	strcpy(r.atc_airline, "TESTAIR");
	strcpy(r.atc_flight_number, "123");
	strcpy(r.atc_id, "N123TA");
	strcpy(r.atc_model, "A320");
	strcpy(r.atc_type, "AIRBUS");
	return r;
}

std::vector<char> recvPacket(DWORD id, size_t size) {
	std::vector<char> packet(size, 0);
	SIMCONNECT_RECV* recv = reinterpret_cast<SIMCONNECT_RECV*>(packet.data());
	recv->dwSize = static_cast<DWORD>(size);
	recv->dwID = id;
	return packet;
}

std::vector<char> eventPacket(DWORD eventId, DWORD data) {
	std::vector<char> packet = recvPacket(SIMCONNECT_RECV_ID_EVENT, sizeof(SIMCONNECT_RECV_EVENT));
	SIMCONNECT_RECV_EVENT* evt = reinterpret_cast<SIMCONNECT_RECV_EVENT*>(packet.data());
	evt->uEventID = eventId;
	evt->dwData = data;
	return packet;
}

// Byte offset of the variable-length payload that starts at a packet's
// trailing "Data" member.
template <typename T, typename M>
static size_t payloadOffset(M T::* member) {
	T probe;
	return reinterpret_cast<const char*>(&(probe.*member)) - reinterpret_cast<const char*>(&probe);
}

std::vector<char> samplePacket(const FLIGHT_DATA_RECORD& record) {
	// Same byte count decode_flight_sample() copies: everything except the unsent
	// trailing time_zulu.timezone_offset.
	const size_t payload = sizeof(FLIGHT_DATA_RECORD) - sizeof(double);
	const size_t offset = payloadOffset(&SIMCONNECT_RECV_SIMOBJECT_DATA::dwData);
	std::vector<char> packet = recvPacket(SIMCONNECT_RECV_ID_SIMOBJECT_DATA, offset + payload);
	SIMCONNECT_RECV_SIMOBJECT_DATA* data = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(packet.data());
	data->dwRequestID = REQUEST_FLIGHT;
	data->dwDefineID = DEFINITION_FLIGHT;
	memcpy(packet.data() + offset, &record, payload);
	return packet;
}

std::vector<char> exceptionPacket(DWORD exception, DWORD sendId) {
	std::vector<char> packet = recvPacket(SIMCONNECT_RECV_ID_EXCEPTION, sizeof(SIMCONNECT_RECV_EXCEPTION));
	SIMCONNECT_RECV_EXCEPTION* except = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(packet.data());
	except->dwException = exception;
	except->dwSendID = sendId;
	return packet;
}

std::vector<char> airportListPacket(const std::vector<AirportSpec>& airports, DWORD entryNumber, DWORD outOf) {
	const size_t offset = payloadOffset(&SIMCONNECT_RECV_AIRPORT_LIST::rgData);
	const size_t count = airports.size();
	std::vector<char> packet = recvPacket(SIMCONNECT_RECV_ID_AIRPORT_LIST,
		offset + sizeof(SIMCONNECT_DATA_FACILITY_AIRPORT) * (count > 0 ? count : 1));
	SIMCONNECT_RECV_AIRPORT_LIST* list = reinterpret_cast<SIMCONNECT_RECV_AIRPORT_LIST*>(packet.data());
	list->dwRequestID = REQUEST_AIRPORTS;
	list->dwArraySize = static_cast<DWORD>(count);
	list->dwEntryNumber = entryNumber;
	list->dwOutOf = outOf;
	for (size_t i = 0; i < count; ++i) {
		SIMCONNECT_DATA_FACILITY_AIRPORT entry;
		memset(&entry, 0, sizeof(entry));
		strncpy(entry.Ident, airports[i].ident.c_str(), sizeof(entry.Ident) - 1);
		strncpy(entry.Region, airports[i].region.c_str(), sizeof(entry.Region) - 1);
		entry.Latitude = airports[i].latitude;
		entry.Longitude = airports[i].longitude;
		memcpy(packet.data() + offset + i * sizeof(entry), &entry, sizeof(entry));
	}
	return packet;
}

static std::vector<char> facilityDataPacket(DWORD type, DWORD uniqueRequestId, DWORD parentUniqueRequestId, DWORD itemIndex,
	const std::vector<char>& payload) {
	const size_t offset = payloadOffset(&SIMCONNECT_RECV_FACILITY_DATA::Data);
	std::vector<char> packet = recvPacket(SIMCONNECT_RECV_ID_FACILITY_DATA, offset + payload.size());
	SIMCONNECT_RECV_FACILITY_DATA* data = reinterpret_cast<SIMCONNECT_RECV_FACILITY_DATA*>(packet.data());
	data->UserRequestId = REQUEST_RUNWAYS;
	data->UniqueRequestId = uniqueRequestId;
	data->ParentUniqueRequestId = parentUniqueRequestId;
	data->Type = type;
	data->ItemIndex = itemIndex;
	memcpy(packet.data() + offset, payload.data(), payload.size());
	return packet;
}

template <typename T>
static void appendBytes(std::vector<char>& out, const T& value) {
	const char* bytes = reinterpret_cast<const char*>(&value);
	out.insert(out.end(), bytes, bytes + sizeof(T));
}

std::vector<char> facilityAirportPacket(const AirportSpec& airport) {
	// Field order of facility_lookup_request_candidate(): NAME64, MAGVAR, N_RUNWAYS.
	std::vector<char> payload;
	char name[64] = {};
	strncpy(name, airport.name.c_str(), sizeof(name) - 1);
	payload.insert(payload.end(), name, name + sizeof(name));
	appendBytes(payload, airport.magvar);
	appendBytes(payload, static_cast<int>(airport.runways.size()));
	return facilityDataPacket(SIMCONNECT_FACILITY_DATA_AIRPORT, 1, 0, 0, payload);
}

std::vector<char> facilityRunwayPacket(DWORD itemIndex, DWORD uniqueRequestId, const RunwaySpec& runway) {
	// LENGTH, WIDTH, HEADING, PRIMARY_NUMBER, SECONDARY_NUMBER,
	// PRIMARY_DESIGNATOR, SECONDARY_DESIGNATOR, LATITUDE, LONGITUDE.
	std::vector<char> payload;
	appendBytes(payload, runway.lengthM);
	appendBytes(payload, runway.widthM);
	appendBytes(payload, runway.heading);
	appendBytes(payload, runway.primaryNumber);
	appendBytes(payload, runway.secondaryNumber);
	appendBytes(payload, runway.primaryDesignator);
	appendBytes(payload, runway.secondaryDesignator);
	appendBytes(payload, runway.latitude);
	appendBytes(payload, runway.longitude);
	return facilityDataPacket(SIMCONNECT_FACILITY_DATA_RUNWAY, uniqueRequestId, 1, itemIndex, payload);
}

std::vector<char> facilityPavementPacket(DWORD parentUniqueRequestId, float lengthM, float widthM, int enable) {
	std::vector<char> payload;
	appendBytes(payload, lengthM);
	appendBytes(payload, widthM);
	appendBytes(payload, enable);
	return facilityDataPacket(SIMCONNECT_FACILITY_DATA_PAVEMENT, parentUniqueRequestId + 500, parentUniqueRequestId, 0, payload);
}

std::vector<char> facilityEndPacket() {
	return recvPacket(SIMCONNECT_RECV_ID_FACILITY_DATA_END, sizeof(SIMCONNECT_RECV_FACILITY_DATA_END));
}

COORDINATE pointOnRunway(const RunwaySpec& runway, double distanceM, double rightM) {
	COORDINATE center;
	center.latitude = runway.latitude;
	center.longitude = runway.longitude;
	double reverse = runway.heading - 180;
	if (reverse <= 0)
		reverse += 360;
	COORDINATE start = center.destinationWithDistanceAndBearing(runway.lengthM / 2000.0, reverse);
	COORDINATE along = start.destinationWithDistanceAndBearing(distanceM / 1000.0, runway.heading);
	if (rightM == 0)
		return along;
	double right = runway.heading + 90;
	if (right > 360)
		right -= 360;
	return along.destinationWithDistanceAndBearing(rightM / 1000.0, right);
}

FlightDriver::FlightDriver() : record(makeRecord()) {
	FakeSim::reset();
	bridge_ = std::make_unique<RecorderBridge>();
	bridge_->start();
	status().event_filter.set_clock([this] { return eventClock_; });
	send(recvPacket(SIMCONNECT_RECV_ID_OPEN, sizeof(SIMCONNECT_RECV)));
	simEvent(EVENT_SIM, 1);
}

FlightDriver::~FlightDriver() {
	bridge_.reset();
}

void FlightDriver::pump() {
	QMetaObject::invokeMethod(bridge_.get(), "pollDispatch", Qt::DirectConnection);
}

void FlightDriver::send(std::vector<char> packet) {
	FakeSim::queue(std::move(packet));
	pump();
}

void FlightDriver::simEvent(DWORD eventId, DWORD data) {
	send(eventPacket(eventId, data));
}

void FlightDriver::tick(double seconds) {
	eventClock_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		std::chrono::duration<double>(seconds));
	for (DATETIME* t : { &record.time_zulu, &record.time_local }) {
		t->time_day += seconds;
		if (t->time_day >= 86400) {
			t->time_day -= 86400;
			t->day_of_month += 1;
		}
	}
	send(samplePacket(record));
}

void FlightDriver::ticks(int count, double seconds) {
	for (int i = 0; i < count; ++i)
		tick(seconds);
}

void FlightDriver::setOnGround(bool onGround) {
	record.sim_on_ground = onGround ? 1 : 0;
}

void FlightDriver::setEngines(bool running) {
	for (double* combustion : { &record.eng_combustion_1, &record.eng_combustion_2, &record.eng_combustion_3, &record.eng_combustion_4 })
		*combustion = running ? 1 : 0;
}

void FlightDriver::moveTo(const COORDINATE& position) {
	record.plane_coordinate.latitude = position.latitude;
	record.plane_coordinate.longitude = position.longitude;
}

void FlightDriver::setHeading(double magneticDegrees) {
	record.plane_heading_degrees_magnetic = magneticDegrees;
}

int FlightDriver::startTrip() {
	setOnGround(true);
	setEngines(true);
	tick();
	return status().id_trip;
}

void FlightDriver::endTrip() {
	const int tripId = status().id_trip;
	setOnGround(true);
	setEngines(false);
	tick();
	waitFor([this, tripId] { return !bridge_->isTripFlushing(tripId); });
}

void FlightDriver::serviceLookups() {
	FakeSim::State& fake = FakeSim::state();
	bool progressed = true;
	for (int round = 0; progressed; ++round) {
		// A lookup that requests itself again on every answer would otherwise
		// never let this return.
		if (round == 100)
			QFAIL("facility lookups never settle");
		progressed = false;
		while (listRequestsServed_ < fake.facilitiesListRequests.size()) {
			const DWORD sendId = fake.facilitiesListRequests[listRequestsServed_++];
			progressed = true;
			if (rejectedRequests.count(sendId))
				continue;
			const size_t chunk = airportListChunkSize > 0 ? (size_t)airportListChunkSize : (airports.empty() ? 1 : airports.size());
			const DWORD outOf = airports.empty() ? 1 : static_cast<DWORD>((airports.size() + chunk - 1) / chunk);
			for (DWORD entry = 0; entry < outOf; ++entry) {
				std::vector<AirportSpec> part;
				for (size_t i = entry * chunk; i < airports.size() && i < (entry + 1) * chunk; ++i)
					part.push_back(airports[i]);
				FakeSim::queue(airportListPacket(part, entry, outOf));
			}
			pump();
		}
		while (dataRequestsServed_ < fake.facilityDataRequests.size()) {
			const FakeSim::FacilityDataRequest request = fake.facilityDataRequests[dataRequestsServed_++];
			progressed = true;
			for (const AirportSpec& airport : airports) {
				if (airport.ident != request.icao)
					continue;
				FakeSim::queue(facilityAirportPacket(airport));
				for (size_t i = 0; i < airport.runways.size(); ++i) {
					const RunwaySpec& runway = airport.runways[i];
					const DWORD uniqueId = static_cast<DWORD>(100 + i);
					FakeSim::queue(facilityRunwayPacket(static_cast<DWORD>(i), uniqueId, runway));
					FakeSim::queue(facilityPavementPacket(uniqueId, runway.primaryThresholdM, runway.widthM, runway.thresholdEnable));
					FakeSim::queue(facilityPavementPacket(uniqueId, runway.secondaryThresholdM, runway.widthM, runway.thresholdEnable));
				}
				break;
			}
			FakeSim::queue(facilityEndPacket());
			pump();
		}
	}
}

}

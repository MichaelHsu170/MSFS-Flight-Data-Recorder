#include "sim_link.h"
#include "logger.h"

#include <cstddef>
#include <cstring>
#include <string>

namespace {

// Indexed by EVENT_ID.
const char* const EVENT_NAMES[EVENT_ID_COUNT] = {
	"SIM",
	"PAUSE",
	"CRASHED",
#define COCKPIT_EVENT_NAME(id, name) name,
	COCKPIT_EVENTS(COCKPIT_EVENT_NAME)
#undef COCKPIT_EVENT_NAME
};

void add_flight_datum(HANDLE hSimConnect, const char* simVar, const char* unit,
	SIMCONNECT_DATATYPE type = SIMCONNECT_DATATYPE_FLOAT64) {
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, simVar, unit, type);
}

// An ENGINES field of FLIGHT_DATA_FIELDS: "simVar:1" to "simVar:MAX_ENGINES".
void add_engine_data(HANDLE hSimConnect, const char* simVar, const char* unit) {
	for (int engine = 1; engine <= MAX_ENGINES; ++engine) {
		const std::string name = std::string(simVar) + ":" + std::to_string(engine);
		add_flight_datum(hSimConnect, name.c_str(), unit);
	}
}

// A TIME field of FLIGHT_DATA_FIELDS, in DATETIME's field order.
void add_datetime(HANDLE hSimConnect, const char* prefix, const char* offsetSimVar) {
	static const char* const kParts[][2] = {
		{ " YEAR", "Number" },
		{ " MONTH OF YEAR", "Number" },
		{ " DAY OF MONTH", "Number" },
		{ " DAY OF WEEK", "Number" },
		{ " TIME", "Seconds" },
	};
	for (const auto& [suffix, unit] : kParts)
		add_flight_datum(hSimConnect, (std::string(prefix) + suffix).c_str(), unit);
	if (offsetSimVar != nullptr)
		add_flight_datum(hSimConnect, offsetSimVar, "Seconds");
}

// Logs the dwSendID SimConnect actually assigned to the request just sent, so
// a later SIMCONNECT_RECV_ID_EXCEPTION's dwSendID can be matched back to a
// specific event/name by reading the log instead of manually counting call
// order, which is error-prone.
void log_last_send_id(HANDLE hSimConnect, const char* call, const char* name) {
	DWORD sendId = 0;
	SimConnect_GetLastSentPacketID(hSimConnect, &sendId);
	Logger::logf(Logger::Trace, "Recorder", "%s(%s) -> SendID=%lu", call, name, sendId);
}

void map_client_event(HANDLE hSimConnect, EVENT_ID id, const char* name) {
	SimConnect_MapClientEventToSimEvent(hSimConnect, id, name);
	log_last_send_id(hSimConnect, "MapClientEventToSimEvent", name);
}

void add_notification_event(HANDLE hSimConnect, EVENT_ID id) {
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, id);
	log_last_send_id(hSimConnect, "AddClientEventToNotificationGroup", event_name(id));
}

}

const char* event_name(DWORD id) {
	return id < EVENT_ID_COUNT ? EVENT_NAMES[id] : nullptr;
}

const char* simconnect_exception_name(DWORD exception) {
	static const char* names[] = {
		"NONE", "ERROR", "SIZE_MISMATCH", "UNRECOGNIZED_ID", "UNOPENED",
		"VERSION_MISMATCH", "TOO_MANY_GROUPS", "NAME_UNRECOGNIZED", "TOO_MANY_EVENT_NAMES",
		"EVENT_ID_DUPLICATE", "TOO_MANY_MAPS", "TOO_MANY_OBJECTS", "TOO_MANY_REQUESTS",
		"WEATHER_INVALID_PORT", "WEATHER_INVALID_METAR", "WEATHER_UNABLE_TO_GET_OBSERVATION",
		"WEATHER_UNABLE_TO_CREATE_STATION", "WEATHER_UNABLE_TO_REMOVE_STATION",
		"INVALID_DATA_TYPE", "INVALID_DATA_SIZE", "DATA_ERROR", "INVALID_ARRAY",
		"CREATE_OBJECT_FAILED", "LOAD_FLIGHTPLAN_FAILED", "OPERATION_INVALID_FOR_OBJECT_TYPE",
		"ILLEGAL_OPERATION", "ALREADY_SUBSCRIBED", "INVALID_ENUM", "DEFINITION_ERROR",
		"DUPLICATE_ID", "DATUM_ID", "OUT_OF_BOUNDS", "ALREADY_CREATED",
		"OBJECT_OUTSIDE_REALITY_BUBBLE", "OBJECT_CONTAINER", "OBJECT_AI", "OBJECT_ATC",
		"OBJECT_SCHEDULE", "JETWAY_DATA", "ACTION_NOT_FOUND", "NOT_AN_ACTION",
		"INCORRECT_ACTION_PARAMS", "GET_INPUT_EVENT_FAILED", "SET_INPUT_EVENT_FAILED",
		"EVENT_NAME_RESERVED", "INTERNAL", "CAMERA_API",
	};
	return exception < (sizeof(names) / sizeof(names[0])) ? names[exception] : "UNKNOWN";
}

void add_flight_definition(HANDLE hSimConnect) {
#define ADD_NUM(member, simVar, unit) add_flight_datum(hSimConnect, simVar, unit);
#define ADD_ENGINES(member, simVar, unit) add_engine_data(hSimConnect, simVar, unit);
#define ADD_COORD(member, latitude, longitude) \
	add_flight_datum(hSimConnect, latitude, "Degrees"); \
	add_flight_datum(hSimConnect, longitude, "Degrees");
#define ADD_STR(member, simVar, size) add_flight_datum(hSimConnect, simVar, NULL, SIMCONNECT_DATATYPE_STRING##size);
#define ADD_TIME(member, prefix, offset) add_datetime(hSimConnect, prefix, offset);
	FLIGHT_DATA_FIELDS(ADD_NUM, ADD_ENGINES, ADD_COORD, ADD_STR, ADD_TIME)
#undef ADD_NUM
#undef ADD_ENGINES
#undef ADD_COORD
#undef ADD_STR
#undef ADD_TIME
	SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_FLIGHT, DEFINITION_FLIGHT, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME);
}

void add_client_events(HANDLE hSimConnect) {
#define MAP_COCKPIT_EVENT(id, name) map_client_event(hSimConnect, EVENT_##id, name);
	COCKPIT_EVENTS(MAP_COCKPIT_EVENT)
#undef MAP_COCKPIT_EVENT
#define NOTIFY_COCKPIT_EVENT(id, name) add_notification_event(hSimConnect, EVENT_##id);
	COCKPIT_EVENTS(NOTIFY_COCKPIT_EVENT)
#undef NOTIFY_COCKPIT_EVENT
}

DWORD flight_sample_packet_size(const SIMCONNECT_RECV_SIMOBJECT_DATA* data) {
	// The wire payload covers every field except the last one:
	// add_flight_definition() registers no ZULU counterpart of "TIME ZONE
	// OFFSET", so time_zulu.timezone_offset is never sent.
	static_assert(offsetof(DATETIME, timezone_offset) + sizeof(double) == sizeof(DATETIME)
		&& offsetof(FLIGHT_DATA_RECORD, time_zulu) + sizeof(DATETIME) == sizeof(FLIGHT_DATA_RECORD),
		"time_zulu.timezone_offset must be the last byte range of FLIGHT_DATA_RECORD");
	const size_t header = reinterpret_cast<const char*>(&data->dwData) - reinterpret_cast<const char*>(data);
	return static_cast<DWORD>(header + sizeof(struct FLIGHT_DATA_RECORD) - sizeof(double));
}

bool decode_flight_sample(const SIMCONNECT_RECV_SIMOBJECT_DATA* data, DWORD cbData, FLIGHT_DATA_RECORD& sample) {
	if (cbData < flight_sample_packet_size(data))
		return false;
	memset(&sample, 0, sizeof(struct FLIGHT_DATA_RECORD));
	memcpy(&sample, &data->dwData, sizeof(struct FLIGHT_DATA_RECORD) - sizeof(double));
	// SimConnect returns pitch and bank inverted from aviation convention:
	//   pitch: positive = nose down  → negate to positive = nose up
	//   bank:  positive = left wing down → negate to positive = right bank
	// Negate here so all downstream code -- DB, charts, data table, touchdown
	// records -- uses the standard aviation sign convention.
	sample.plane_pitch_degrees = -sample.plane_pitch_degrees;
	sample.plane_touchdown_pitch_degrees = -sample.plane_touchdown_pitch_degrees;
	sample.plane_bank_degrees = -sample.plane_bank_degrees;
	sample.plane_touchdown_bank_degrees = -sample.plane_touchdown_bank_degrees;
	return true;
}

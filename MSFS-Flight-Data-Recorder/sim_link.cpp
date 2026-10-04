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

// FLIGHT_DATA_RECORD's engine arrays, in struct order: { SimVar, unit }.
// Each is registered for engines 1..MAX_ENGINES.
const char* const ENGINE_POWER_SIMVARS[][2] = {
	{ "GENERAL ENG RPM", "rpm" },
	{ "RECIP ENG MANIFOLD PRESSURE", "inHg" },
	{ "TURB ENG N1", "Percent" },
	{ "TURB ENG N2", "Percent" },
	{ "TURB ENG MAX TORQUE PERCENT", "Percent" },
	{ "PROP RPM", "rpm" },
};

// Logs the dwSendID SimConnect actually assigned to this request, so a later
// SIMCONNECT_RECV_ID_EXCEPTION's dwSendID can be matched back to a specific
// event/name by reading the log instead of manually counting call order
// (which is error-prone -- see the FLIGHT_LEVEL_CHANGE misdiagnosis this
// replaced).
void map_client_event(HANDLE hSimConnect, EVENT_ID id, const char* name) {
	SimConnect_MapClientEventToSimEvent(hSimConnect, id, name);
	DWORD sendId = 0;
	SimConnect_GetLastSentPacketID(hSimConnect, &sendId);
	Logger::logf(Logger::Trace, "Recorder", "MapClientEventToSimEvent(%s) -> SendID=%lu", name, sendId);
}

void add_notification_event(HANDLE hSimConnect, EVENT_ID id) {
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, id);
	DWORD sendId = 0;
	SimConnect_GetLastSentPacketID(hSimConnect, &sendId);
	Logger::logf(Logger::Trace, "Recorder", "AddClientEventToNotificationGroup(%s) -> SendID=%lu", event_name(id), sendId);
}

}

const char* event_name(DWORD id) {
	return EVENT_NAMES[id];
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
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT AIRSPEED HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT AIRSPEED HOLD VAR", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT ALT RADIO MODE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT ALTITUDE LOCK", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT ALTITUDE LOCK VAR", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT APPROACH ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT APPROACH CAPTURED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT APPROACH HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT APPROACH IS LOCALIZER", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT AVIONICS MANAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT DISENGAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT FLIGHT DIRECTOR ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT FLIGHT LEVEL CHANGE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT GLIDESLOPE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT GLIDESLOPE ARM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT GLIDESLOPE HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT HEADING LOCK", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT HEADING LOCK DIR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT MACH HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT MACH HOLD VAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT MANAGED SPEED IN MACH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT MANAGED THROTTLE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT MASTER", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT TAKEOFF POWER ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT THROTTLE ARM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT THROTTLE MAX THRUST", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT VERTICAL HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOPILOT VERTICAL HOLD VAR", "Feet/minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOBRAKES ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTO BRAKE SWITCH CB", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BRAKE INDICATOR", "Position");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BRAKE PARKING INDICATOR", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "REJECTED TAKEOFF BRAKES ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR DAMAGE BY SPEED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR HANDLE POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR IS ON GROUND:0", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR IS ON GROUND:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR IS ON GROUND:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR POSITION:0", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR POSITION:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR POSITION:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR SPEED EXCEEDED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR WARNING:0", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR WARNING:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GEAR WARNING:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WHEEL RPM:0", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WHEEL RPM:1", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WHEEL RPM:2", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON LEFT DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON LEFT DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON RIGHT DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON RIGHT DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON TRIM", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AILERON TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVATOR DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVATOR DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVATOR TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVATOR TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVATOR TRIM POSITION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELEVON DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FLAP DAMAGE BY SPEED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FLAP SPEED EXCEEDED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FLAPS HANDLE INDEX", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FLAPS NUM HANDLE POSITIONS", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RUDDER DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RUDDER DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RUDDER TRIM", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RUDDER TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RUDDER TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SPOILERS ARMED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SPOILERS HANDLE POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SPOILERS LEFT POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SPOILERS RIGHT POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU BLEED PRESSURE RECEIVED BY ENGINE", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU GENERATOR ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU GENERATOR SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU ON FIRE DETECTED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU PCT RPM", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU PCT STARTER", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "APU SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BLEED AIR APU", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELECTRICAL BATTERY VOLTAGE", "Volts");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ELECTRICAL MASTER BATTERY", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "EXTERNAL POWER AVAILABLE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "EXTERNAL POWER CONNECTION ON", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "EXTERNAL POWER ON", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BLEED AIR ENGINE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BLEED AIR ENGINE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BLEED AIR SOURCE CONTROL:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BLEED AIR SOURCE CONTROL:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENGINE CONTROL SELECT", "Flags");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENGINE TYPE", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG ANTI ICE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG ANTI ICE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG COMBUSTION:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG COMBUSTION:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG EXHAUST GAS TEMPERATURE:1", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG EXHAUST GAS TEMPERATURE:2", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG FAILED:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG FAILED:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG HYDRAULIC PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG HYDRAULIC PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG OIL PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG OIL PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG OIL TEMPERATURE:1", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG OIL TEMPERATURE:2", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG ON FIRE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ENG ON FIRE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG DAMAGE PERCENT:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG DAMAGE PERCENT:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG ELAPSED TIME:1", "Hours");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG ELAPSED TIME:2", "Hours");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FIRE DETECTED:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FIRE DETECTED:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FUEL USED SINCE START:1", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FUEL USED SINCE START:2", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FUEL VALVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG FUEL VALVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG GENERATOR ACTIVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG GENERATOR ACTIVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG GENERATOR SWITCH:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG GENERATOR SWITCH:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG MASTER ALTERNATOR", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG REVERSE THRUST ENGAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG STARTER:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG STARTER:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG STARTER ACTIVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG STARTER ACTIVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG THROTTLE LEVER POSITION:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG THROTTLE LEVER POSITION:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG THROTTLE MANAGED MODE:1", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GENERAL ENG THROTTLE MANAGED MODE:2", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "MASTER IGNITION SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "NUMBER OF ENGINES", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG BLEED AIR:1", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG BLEED AIR:2", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG FUEL AVAILABLE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG FUEL AVAILABLE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG FUEL FLOW PPH:1", "Pounds per hour");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG FUEL FLOW PPH:2", "Pounds per hour");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG IGNITION SWITCH EX1:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG IGNITION SWITCH EX1:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG IS IGNITING:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG IS IGNITING:2", "Bool");
	for (const auto& [simVar, unit] : ENGINE_POWER_SIMVARS) {
		for (int engine = 1; engine <= MAX_ENGINES; ++engine) {
			const std::string name = std::string(simVar) + ":" + std::to_string(engine);
			SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, name.c_str(), unit);
		}
	}
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG VIBRATION:1", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG VIBRATION:2", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "G FORCE", "GForce");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "EMPTY WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TOTAL WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL CROSS FEED:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL CROSS FEED:3", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL SELECTED QUANTITY:2", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL SELECTED QUANTITY:3", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL SELECTED QUANTITY PERCENT:2", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL SELECTED QUANTITY PERCENT:3", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL TOTAL QUANTITY", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL TOTAL QUANTITY WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL TRANSFER PUMP ON:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL TRANSFER PUMP ON:3", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "FUEL WEIGHT PER GALLON", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ON ANY RUNWAY", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE IN PARKING STATE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SURFACE CONDITION", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SURFACE TYPE", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GROUND VELOCITY", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE ALT ABOVE GROUND", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE BANK DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE HEADING DEGREES GYRO", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE HEADING DEGREES MAGNETIC", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE HEADING DEGREES TRUE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE LATITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE LONGITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE PITCH DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN BANK DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN HEADING DEGREES MAGNETIC", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN HEADING DEGREES TRUE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN LATITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN LONGITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN NORMAL VELOCITY", "Feet per minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PLANE TOUCHDOWN PITCH DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "VERTICAL SPEED", "Feet per minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AIRSPEED INDICATED", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AIRSPEED MACH", "Mach");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AIRSPEED TRUE", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS GROUND SPEED", "Meters per second");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS GROUND TRUE HEADING", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS GROUND TRUE TRACK", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS POSITION ALT", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS POSITION LAT", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPS POSITION LON", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "RADIO HEIGHT", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AUTOTHROTTLE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AVIONICS MASTER SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "CABIN NO SMOKING ALERT SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "CABIN SEATBELTS ALERT SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPWS SYSTEM ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GPWS WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "GYRO DRIFT ERROR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "HEADING INDICATOR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "INDICATED ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "INDICATED ALTITUDE CALIBRATED", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "MAGNETIC COMPASS", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "OVERSPEED WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PITOT ICE PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PITOT HEAT", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PITOT HEAT SWITCH", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PRESSURE ALTITUDE", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "PRESSURIZATION CABIN ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "STALL WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "STRUCTURAL DEICE SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LIGHT STATES", "Mask");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "HYDRAULIC PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "HYDRAULIC PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "HYDRAULIC SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WARNING FUEL", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WARNING LOW HEIGHT", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WARNING OIL PRESSURE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WARNING VACUUM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "WARNING VOLTAGE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "SIM ON GROUND", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AMBIENT PRESSURE", "inHg");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AMBIENT TEMPERATURE", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AMBIENT VISIBILITY", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AMBIENT WIND DIRECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "AMBIENT WIND VELOCITY", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "BAROMETER PRESSURE", "Millibars");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "KOHLSMAN SETTING HG", "inHg");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "KOHLSMAN SETTING MB", "Millibars");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "KOHLSMAN SETTING STD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TITLE", NULL, SIMCONNECT_DATATYPE_STRING256);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ATC AIRLINE", NULL, SIMCONNECT_DATATYPE_STRING64);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ATC FLIGHT NUMBER", NULL, SIMCONNECT_DATATYPE_STRING8);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ATC ID", NULL, SIMCONNECT_DATATYPE_STRING32);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ATC MODEL", NULL, SIMCONNECT_DATATYPE_STRING32);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ATC TYPE", NULL, SIMCONNECT_DATATYPE_STRING64);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LOCAL YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LOCAL MONTH OF YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LOCAL DAY OF MONTH", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LOCAL DAY OF WEEK", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "LOCAL TIME", "Seconds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TIME ZONE OFFSET", "Seconds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ZULU YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ZULU MONTH OF YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ZULU DAY OF MONTH", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ZULU DAY OF WEEK", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "ZULU TIME", "Seconds");
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

void decode_flight_sample(const SIMCONNECT_RECV_SIMOBJECT_DATA* data, FLIGHT_DATA_RECORD& sample) {
	memset(&sample, 0, sizeof(struct FLIGHT_DATA_RECORD));
	// The wire payload covers every field except the last one:
	// add_flight_definition() registers no ZULU counterpart of "TIME ZONE
	// OFFSET", so time_zulu.timezone_offset is never sent.
	static_assert(offsetof(DATETIME, timezone_offset) + sizeof(double) == sizeof(DATETIME)
		&& offsetof(FLIGHT_DATA_RECORD, time_zulu) + sizeof(DATETIME) == sizeof(FLIGHT_DATA_RECORD),
		"time_zulu.timezone_offset must be the last byte range of FLIGHT_DATA_RECORD");
	memcpy(&sample, &data->dwData, sizeof(struct FLIGHT_DATA_RECORD) - sizeof(double));
	// SimConnect returns pitch and bank inverted from aviation convention:
	//   pitch: positive = nose down  → negate to positive = nose up
	//   bank:  positive = left wing down → negate to positive = right bank
	// Negate here so all downstream code — DB, charts, data table, touchdown
	// records — uses the standard aviation sign convention.
	sample.plane_pitch_degrees = -sample.plane_pitch_degrees;
	sample.plane_touchdown_pitch_degrees = -sample.plane_touchdown_pitch_degrees;
	sample.plane_bank_degrees = -sample.plane_bank_degrees;
	sample.plane_touchdown_bank_degrees = -sample.plane_touchdown_bank_degrees;
}

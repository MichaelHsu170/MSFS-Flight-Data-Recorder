#include "recorder.h"
#include "db.h"
#include "gui_notify.h"
#include "logger.h"
#include "airport_lookup.h"
#include "flight_phase.h"
#include <chrono>
#include <thread>

// Rate-limits the "Event ignored (no active trip)" TRACE line in
// commit_event() below -- independent of EventFloodFilter's tiers, and it
// never affects whether an occurrence commits: entries just age out. Exists
// for the flap events the filter lets straight through (FLAPS_INCR/
// FLAPS_DECR): without it, a flap lever held with no trip active would print
// one line per occurrence, unbounded. Same 5 s as the filter's slow-flood
// window, for consistency only.
static const std::chrono::milliseconds EVENT_NO_TRIP_LOG_COOLDOWN(5000);

// Where an event occurrence that has passed (or bypassed) EventFloodFilter's
// tiers either gets recorded or discarded, based purely on whether a trip
// exists to attach it to. Has no memory of its own governing *whether* an
// occurrence commits -- flood-shaped repetition is the filter's job; the only
// state kept here (STATUS::no_trip_log_throttle) rate-limits its own TRACE
// line, see EVENT_NO_TRIP_LOG_COOLDOWN above. trip_id was captured when the
// occurrence happened (a new trip may be live by the time a held-back one
// commits). seq is the filter's id for it, stored in the DB row (if one is
// written) so a slow flood can be retracted precisely later -- see
// db_delete_events() in db.cpp. Returns true if the occurrence was written to
// trip_events and shown in the Live Status list, false if it was dropped.
static bool commit_event(struct STATUS* status, const std::string& name, int trip_id, const std::string& time_zulu, const std::string& time_local, unsigned long long seq) {
	if (trip_id <= 0) {
		auto now = std::chrono::steady_clock::now();
		auto& last_logged = status->no_trip_log_throttle[name];
		if (now - last_logged >= EVENT_NO_TRIP_LOG_COOLDOWN) {
			gui_log_printf(status, GUI_LOG_TRACE, "Event ignored (no active trip): %s", name.c_str());
			last_logged = now;
		}
		return false;
	}
	gui_notify_event_committed(status, trip_id, seq, name.c_str());
	status->event_write_queue.push(trip_id, name, time_zulu, time_local, seq);
	return true;
}

// Wires the flood filter's output to this trip-aware commit and to the DB/UI
// retraction. A retraction's Delete goes onto the same single-threaded write
// queue as the inserts that created those rows, so it can never run before
// they land -- see EventWriteQueue in types.h.
static EventFloodFilter::Output event_output(struct STATUS* status) {
	EventFloodFilter::Output out;
	out.commit = [status](const std::string& name, int trip_id, const std::string& time_zulu,
		const std::string& time_local, unsigned long long seq) {
		commit_event(status, name, trip_id, time_zulu, time_local, seq);
	};
	out.retract = [status](const std::vector<unsigned long long>& seqs) {
		status->event_write_queue.push_delete(seqs);
		gui_notify_events_retracted(status, seqs.data(), seqs.size());
	};
	return out;
}

static const char* simconnect_exception_txt(DWORD exception) {
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

static const char* EVENT_ID_TXT[] = {
	"SIM",
	"PAUSE",
	"CRASHED",
	"AP_AIRSPEED_HOLD",
	"AP_AIRSPEED_OFF",
	"AP_AIRSPEED_ON",
	"AP_ALT_HOLD",
	"AP_ALT_HOLD_OFF",
	"AP_ALT_HOLD_ON",
	"AP_APR_HOLD",
	"AP_APR_HOLD_OFF",
	"AP_APR_HOLD_ON",
	"AP_HDG_HOLD",
	"AP_HDG_HOLD_OFF",
	"AP_HDG_HOLD_ON",
	"AP_MACH_HOLD",
	"AP_MACH_OFF",
	"AP_MACH_ON",
	"AP_MASTER",
	"AP_PANEL_ALTITUDE_HOLD",
	"AP_PANEL_ALTITUDE_OFF",
	"AP_PANEL_ALTITUDE_ON",
	"AP_PANEL_HEADING_HOLD",
	"AP_PANEL_HEADING_OFF",
	"AP_PANEL_HEADING_ON",
	"AP_PANEL_MACH_HOLD",
	"AP_PANEL_MACH_OFF",
	"AP_PANEL_MACH_ON",
	"AP_PANEL_SPEED_HOLD",
	"AP_PANEL_SPEED_OFF",
	"AP_PANEL_SPEED_ON",
	"AP_PANEL_VS_OFF",
	"AP_PANEL_VS_ON",
	"AP_PANEL_VS_HOLD",
	"AP_VS_HOLD",
	"AP_VS_OFF",
	"AP_VS_ON",
	"AP_PANEL_SPEED_HOLD_TOGGLE",
	"AP_PANEL_MACH_HOLD_TOGGLE",
	"AUTOPILOT_DISENGAGE_TOGGLE",
	"AUTOPILOT_OFF",
	"AUTOPILOT_ON",
	"AP_PANEL_SPEED_SET",
	"FLIGHT_LEVEL_CHANGE",
	"FLIGHT_LEVEL_CHANGE_OFF",
	"FLIGHT_LEVEL_CHANGE_ON",
	"AUTO_THROTTLE_ARM",
	"AUTO_THROTTLE_TO_GA",
	"AUTOBRAKE_DISARM",
	"AUTOBRAKE_HI_SET",
	"AUTOBRAKE_LO_SET",
	"AUTOBRAKE_MED_SET",
	"GPWS_SWITCH_TOGGLE",
	"TOGGLE_FLIGHT_DIRECTOR",
	"APU_BLEED_AIR_SOURCE_TOGGLE",
	"APU_GENERATOR_SWITCH_TOGGLE",
	"APU_OFF_SWITCH",
	"APU_STARTER",
	"ANTI_ICE_ON",
	"ANTI_ICE_OFF",
	"ANTI_ICE_TOGGLE",
	"ANTI_ICE_TOGGLE_ENG1",
	"ANTI_ICE_TOGGLE_ENG2",
	"THROTTLE_REVERSE_THRUST_TOGGLE",
	"FLAPS_DECR",
	"FLAPS_DOWN",
	"FLAPS_INCR",
	"FLAPS_UP",
	"SPOILERS_ARM_OFF",
	"SPOILERS_ARM_ON",
	"SPOILERS_ARM_TOGGLE",
	"SPOILERS_OFF",
	"SPOILERS_ON",
	"SPOILERS_TOGGLE",
	"CROSS_FEED_TOGGLE",
	"BRAKES",
	"GEAR_DOWN",
	"GEAR_EMERGENCY_HANDLE_TOGGLE",
	"GEAR_TOGGLE",
	"GEAR_UP",
	"PARKING_BRAKES",
	"CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE",
	"CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE",
	"WINDSHIELD_DEICE_OFF",
	"WINDSHIELD_DEICE_ON",
	"WINDSHIELD_DEICE_TOGGLE",
	"TOGGLE_AVIONICS_MASTER"
};

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
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG N1:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG N1:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG N2:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_FLIGHT, "TURB ENG N2:2", "Percent");
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

// Logs the dwSendID SimConnect actually assigned to this request, so a later
// SIMCONNECT_RECV_ID_EXCEPTION's dwSendID can be matched back to a specific
// event/name by reading the log instead of manually counting call order
// (which is error-prone -- see the FLIGHT_LEVEL_CHANGE misdiagnosis this
// replaced).
static void map_client_event(HANDLE hSimConnect, EVENT_ID id, const char* name) {
	SimConnect_MapClientEventToSimEvent(hSimConnect, id, name);
	DWORD sendId = 0;
	SimConnect_GetLastSentPacketID(hSimConnect, &sendId);
	Logger::logf(Logger::Trace, "Recorder", "MapClientEventToSimEvent(%s) -> SendID=%lu", name, sendId);
}

static void add_notification_event(HANDLE hSimConnect, EVENT_ID id) {
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, id);
	DWORD sendId = 0;
	SimConnect_GetLastSentPacketID(hSimConnect, &sendId);
	Logger::logf(Logger::Trace, "Recorder", "AddClientEventToNotificationGroup(%s) -> SendID=%lu", EVENT_ID_TXT[id], sendId);
}

void add_client_events(HANDLE hSimConnect) {
	map_client_event(hSimConnect, EVENT_AP_AIRSPEED_HOLD, "AP_AIRSPEED_HOLD");
	map_client_event(hSimConnect, EVENT_AP_AIRSPEED_OFF, "AP_AIRSPEED_OFF");
	map_client_event(hSimConnect, EVENT_AP_AIRSPEED_ON, "AP_AIRSPEED_ON");
	map_client_event(hSimConnect, EVENT_AP_ALT_HOLD, "AP_ALT_HOLD");
	map_client_event(hSimConnect, EVENT_AP_ALT_HOLD_OFF, "AP_ALT_HOLD_OFF");
	map_client_event(hSimConnect, EVENT_AP_ALT_HOLD_ON, "AP_ALT_HOLD_ON");
	map_client_event(hSimConnect, EVENT_AP_APR_HOLD, "AP_APR_HOLD");
	map_client_event(hSimConnect, EVENT_AP_APR_HOLD_OFF, "AP_APR_HOLD_OFF");
	map_client_event(hSimConnect, EVENT_AP_APR_HOLD_ON, "AP_APR_HOLD_ON");
	map_client_event(hSimConnect, EVENT_AP_HDG_HOLD, "AP_HDG_HOLD");
	map_client_event(hSimConnect, EVENT_AP_HDG_HOLD_OFF, "AP_HDG_HOLD_OFF");
	map_client_event(hSimConnect, EVENT_AP_HDG_HOLD_ON, "AP_HDG_HOLD_ON");
	map_client_event(hSimConnect, EVENT_AP_MACH_HOLD, "AP_MACH_HOLD");
	map_client_event(hSimConnect, EVENT_AP_MACH_OFF, "AP_MACH_OFF");
	map_client_event(hSimConnect, EVENT_AP_MACH_ON, "AP_MACH_ON");
	map_client_event(hSimConnect, EVENT_AP_MASTER, "AP_MASTER");
	map_client_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_HOLD, "AP_PANEL_ALTITUDE_HOLD");
	map_client_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_OFF, "AP_PANEL_ALTITUDE_OFF");
	map_client_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_ON, "AP_PANEL_ALTITUDE_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_HEADING_HOLD, "AP_PANEL_HEADING_HOLD");
	map_client_event(hSimConnect, EVENT_AP_PANEL_HEADING_OFF, "AP_PANEL_HEADING_OFF");
	map_client_event(hSimConnect, EVENT_AP_PANEL_HEADING_ON, "AP_PANEL_HEADING_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_MACH_HOLD, "AP_PANEL_MACH_HOLD");
	map_client_event(hSimConnect, EVENT_AP_PANEL_MACH_OFF, "AP_PANEL_MACH_OFF");
	map_client_event(hSimConnect, EVENT_AP_PANEL_MACH_ON, "AP_PANEL_MACH_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD, "AP_PANEL_SPEED_HOLD");
	map_client_event(hSimConnect, EVENT_AP_PANEL_SPEED_OFF, "AP_PANEL_SPEED_OFF");
	map_client_event(hSimConnect, EVENT_AP_PANEL_SPEED_ON, "AP_PANEL_SPEED_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_VS_OFF, "AP_PANEL_VS_OFF");
	map_client_event(hSimConnect, EVENT_AP_PANEL_VS_ON, "AP_PANEL_VS_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_VS_HOLD, "AP_PANEL_VS_HOLD");
	map_client_event(hSimConnect, EVENT_AP_VS_HOLD, "AP_VS_HOLD");
	map_client_event(hSimConnect, EVENT_AP_VS_OFF, "AP_VS_OFF");
	map_client_event(hSimConnect, EVENT_AP_VS_ON, "AP_VS_ON");
	map_client_event(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD_TOGGLE, "AP_PANEL_SPEED_HOLD_TOGGLE");
	map_client_event(hSimConnect, EVENT_AP_PANEL_MACH_HOLD_TOGGLE, "AP_PANEL_MACH_HOLD_TOGGLE");
	map_client_event(hSimConnect, EVENT_AUTOPILOT_DISENGAGE_TOGGLE, "AUTOPILOT_DISENGAGE_TOGGLE");
	map_client_event(hSimConnect, EVENT_AUTOPILOT_OFF, "AUTOPILOT_OFF");
	map_client_event(hSimConnect, EVENT_AUTOPILOT_ON, "AUTOPILOT_ON");
	map_client_event(hSimConnect, EVENT_AUTOPILOT_PANEL_AIRSPEED_SET, "AP_PANEL_SPEED_SET");
	map_client_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE, "FLIGHT_LEVEL_CHANGE");
	map_client_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_OFF, "FLIGHT_LEVEL_CHANGE_OFF");
	map_client_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_ON, "FLIGHT_LEVEL_CHANGE_ON");
	map_client_event(hSimConnect, EVENT_AUTO_THROTTLE_ARM, "AUTO_THROTTLE_ARM");
	map_client_event(hSimConnect, EVENT_AUTO_THROTTLE_TO_GA, "AUTO_THROTTLE_TO_GA");
	map_client_event(hSimConnect, EVENT_AUTOBRAKE_DISARM, "AUTOBRAKE_DISARM");
	map_client_event(hSimConnect, EVENT_AUTOBRAKE_HI_SET, "AUTOBRAKE_HI_SET");
	map_client_event(hSimConnect, EVENT_AUTOBRAKE_LO_SET, "AUTOBRAKE_LO_SET");
	map_client_event(hSimConnect, EVENT_AUTOBRAKE_MED_SET, "AUTOBRAKE_MED_SET");
	map_client_event(hSimConnect, EVENT_GPWS_SWITCH_TOGGLE, "GPWS_SWITCH_TOGGLE");
	map_client_event(hSimConnect, EVENT_TOGGLE_FLIGHT_DIRECTOR, "TOGGLE_FLIGHT_DIRECTOR");
	map_client_event(hSimConnect, EVENT_APU_BLEED_AIR_SOURCE_TOGGLE, "APU_BLEED_AIR_SOURCE_TOGGLE");
	map_client_event(hSimConnect, EVENT_APU_GENERATOR_SWITCH_TOGGLE, "APU_GENERATOR_SWITCH_TOGGLE");
	map_client_event(hSimConnect, EVENT_APU_OFF_SWITCH, "APU_OFF_SWITCH");
	map_client_event(hSimConnect, EVENT_APU_STARTER, "APU_STARTER");
	map_client_event(hSimConnect, EVENT_ANTI_ICE_ON, "ANTI_ICE_ON");
	map_client_event(hSimConnect, EVENT_ANTI_ICE_OFF, "ANTI_ICE_OFF");
	map_client_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE, "ANTI_ICE_TOGGLE");
	map_client_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG1, "ANTI_ICE_TOGGLE_ENG1");
	map_client_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG2, "ANTI_ICE_TOGGLE_ENG2");
	map_client_event(hSimConnect, EVENT_THROTTLE_REVERSE_THRUST_TOGGLE, "THROTTLE_REVERSE_THRUST_TOGGLE");
	map_client_event(hSimConnect, EVENT_FLAPS_DECR, "FLAPS_DECR");
	map_client_event(hSimConnect, EVENT_FLAPS_DOWN, "FLAPS_DOWN");
	map_client_event(hSimConnect, EVENT_FLAPS_INCR, "FLAPS_INCR");
	map_client_event(hSimConnect, EVENT_FLAPS_UP, "FLAPS_UP");
	map_client_event(hSimConnect, EVENT_SPOILERS_ARM_OFF, "SPOILERS_ARM_OFF");
	map_client_event(hSimConnect, EVENT_SPOILERS_ARM_ON, "SPOILERS_ARM_ON");
	map_client_event(hSimConnect, EVENT_SPOILERS_ARM_TOGGLE, "SPOILERS_ARM_TOGGLE");
	map_client_event(hSimConnect, EVENT_SPOILERS_OFF, "SPOILERS_OFF");
	map_client_event(hSimConnect, EVENT_SPOILERS_ON, "SPOILERS_ON");
	map_client_event(hSimConnect, EVENT_SPOILERS_TOGGLE, "SPOILERS_TOGGLE");
	map_client_event(hSimConnect, EVENT_CROSS_FEED_TOGGLE, "CROSS_FEED_TOGGLE");
	map_client_event(hSimConnect, EVENT_BRAKES, "BRAKES");
	map_client_event(hSimConnect, EVENT_GEAR_DOWN, "GEAR_DOWN");
	map_client_event(hSimConnect, EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE, "GEAR_EMERGENCY_HANDLE_TOGGLE");
	map_client_event(hSimConnect, EVENT_GEAR_TOGGLE, "GEAR_TOGGLE");
	map_client_event(hSimConnect, EVENT_GEAR_UP, "GEAR_UP");
	map_client_event(hSimConnect, EVENT_PARKING_BRAKES, "PARKING_BRAKES");
	map_client_event(hSimConnect, EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE, "CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE");
	map_client_event(hSimConnect, EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE, "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE");
	map_client_event(hSimConnect, EVENT_WINDSHIELD_DEICE_OFF, "WINDSHIELD_DEICE_OFF");
	map_client_event(hSimConnect, EVENT_WINDSHIELD_DEICE_ON, "WINDSHIELD_DEICE_ON");
	map_client_event(hSimConnect, EVENT_WINDSHIELD_DEICE_TOGGLE, "WINDSHIELD_DEICE_TOGGLE");
	map_client_event(hSimConnect, EVENT_TOGGLE_AVIONICS_MASTER, "TOGGLE_AVIONICS_MASTER");

	add_notification_event(hSimConnect, EVENT_AP_AIRSPEED_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_AIRSPEED_OFF);
	add_notification_event(hSimConnect, EVENT_AP_AIRSPEED_ON);
	add_notification_event(hSimConnect, EVENT_AP_ALT_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_ALT_HOLD_OFF);
	add_notification_event(hSimConnect, EVENT_AP_ALT_HOLD_ON);
	add_notification_event(hSimConnect, EVENT_AP_APR_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_APR_HOLD_OFF);
	add_notification_event(hSimConnect, EVENT_AP_APR_HOLD_ON);
	add_notification_event(hSimConnect, EVENT_AP_HDG_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_HDG_HOLD_OFF);
	add_notification_event(hSimConnect, EVENT_AP_HDG_HOLD_ON);
	add_notification_event(hSimConnect, EVENT_AP_MACH_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_MACH_OFF);
	add_notification_event(hSimConnect, EVENT_AP_MACH_ON);
	add_notification_event(hSimConnect, EVENT_AP_MASTER);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_OFF);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_ALTITUDE_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_HEADING_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_HEADING_OFF);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_HEADING_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_MACH_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_MACH_OFF);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_MACH_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_SPEED_OFF);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_SPEED_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_VS_OFF);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_VS_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_VS_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_VS_HOLD);
	add_notification_event(hSimConnect, EVENT_AP_VS_OFF);
	add_notification_event(hSimConnect, EVENT_AP_VS_ON);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD_TOGGLE);
	add_notification_event(hSimConnect, EVENT_AP_PANEL_MACH_HOLD_TOGGLE);
	add_notification_event(hSimConnect, EVENT_AUTOPILOT_DISENGAGE_TOGGLE);
	add_notification_event(hSimConnect, EVENT_AUTOPILOT_OFF);
	add_notification_event(hSimConnect, EVENT_AUTOPILOT_ON);
	add_notification_event(hSimConnect, EVENT_AUTOPILOT_PANEL_AIRSPEED_SET);
	add_notification_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE);
	add_notification_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_OFF);
	add_notification_event(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_ON);
	add_notification_event(hSimConnect, EVENT_AUTO_THROTTLE_ARM);
	add_notification_event(hSimConnect, EVENT_AUTO_THROTTLE_TO_GA);
	add_notification_event(hSimConnect, EVENT_AUTOBRAKE_DISARM);
	add_notification_event(hSimConnect, EVENT_AUTOBRAKE_HI_SET);
	add_notification_event(hSimConnect, EVENT_AUTOBRAKE_LO_SET);
	add_notification_event(hSimConnect, EVENT_AUTOBRAKE_MED_SET);
	add_notification_event(hSimConnect, EVENT_GPWS_SWITCH_TOGGLE);
	add_notification_event(hSimConnect, EVENT_TOGGLE_FLIGHT_DIRECTOR);
	add_notification_event(hSimConnect, EVENT_APU_BLEED_AIR_SOURCE_TOGGLE);
	add_notification_event(hSimConnect, EVENT_APU_GENERATOR_SWITCH_TOGGLE);
	add_notification_event(hSimConnect, EVENT_APU_OFF_SWITCH);
	add_notification_event(hSimConnect, EVENT_APU_STARTER);
	add_notification_event(hSimConnect, EVENT_ANTI_ICE_ON);
	add_notification_event(hSimConnect, EVENT_ANTI_ICE_OFF);
	add_notification_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE);
	add_notification_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG1);
	add_notification_event(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG2);
	add_notification_event(hSimConnect, EVENT_THROTTLE_REVERSE_THRUST_TOGGLE);
	add_notification_event(hSimConnect, EVENT_FLAPS_DECR);
	add_notification_event(hSimConnect, EVENT_FLAPS_DOWN);
	add_notification_event(hSimConnect, EVENT_FLAPS_INCR);
	add_notification_event(hSimConnect, EVENT_FLAPS_UP);
	add_notification_event(hSimConnect, EVENT_SPOILERS_ARM_OFF);
	add_notification_event(hSimConnect, EVENT_SPOILERS_ARM_ON);
	add_notification_event(hSimConnect, EVENT_SPOILERS_ARM_TOGGLE);
	add_notification_event(hSimConnect, EVENT_SPOILERS_OFF);
	add_notification_event(hSimConnect, EVENT_SPOILERS_ON);
	add_notification_event(hSimConnect, EVENT_SPOILERS_TOGGLE);
	add_notification_event(hSimConnect, EVENT_CROSS_FEED_TOGGLE);
	add_notification_event(hSimConnect, EVENT_BRAKES);
	add_notification_event(hSimConnect, EVENT_GEAR_DOWN);
	add_notification_event(hSimConnect, EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE);
	add_notification_event(hSimConnect, EVENT_GEAR_TOGGLE);
	add_notification_event(hSimConnect, EVENT_GEAR_UP);
	add_notification_event(hSimConnect, EVENT_PARKING_BRAKES);
	add_notification_event(hSimConnect, EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE);
	add_notification_event(hSimConnect, EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE);
	add_notification_event(hSimConnect, EVENT_WINDSHIELD_DEICE_OFF);
	add_notification_event(hSimConnect, EVENT_WINDSHIELD_DEICE_ON);
	add_notification_event(hSimConnect, EVENT_WINDSHIELD_DEICE_TOGGLE);
	add_notification_event(hSimConnect, EVENT_TOGGLE_AVIONICS_MASTER);
}

// Signals the DB-write worker to drain and exit, then joins it. Callers must
// call this before closing/nulling status->sql.
void wait_for_db_writers(struct STATUS* status) {
	// Must run before event_write_queue.stop() below: this is the last chance
	// for any occurrence still held back by the flood filter to reach the
	// queue at all -- once the worker thread is stopped and joined, and
	// status->sql is closed by the caller right after this function returns,
	// nothing will ever flush them again for this session. Each still carries
	// the trip_id it was captured with, so it lands in the trip it happened in.
	status->event_filter.flush_all(event_output(status));
	status->sample_write_queue.stop();
	if (status->db_writer_thread.joinable())
		status->db_writer_thread.join();
	status->event_write_queue.stop();
	if (status->event_writer_thread.joinable())
		status->event_writer_thread.join();
}

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
	struct STATUS* status = (struct STATUS*)pContext;
	try {
	switch (pData->dwID) {
	case SIMCONNECT_RECV_ID_OPEN:
		gui_log_printf(status, GUI_LOG_INFO, "Connected to Microsoft Flight Simulator");
		gui_notify_connection_changed(status, true);
		break;
	case SIMCONNECT_RECV_ID_QUIT:
		gui_log_printf(status, GUI_LOG_INFO, "Disconnected from Microsoft Flight Simulator");
		gui_notify_connection_changed(status, false);
		status->quit = TRUE;
		break;
	case SIMCONNECT_RECV_ID_EVENT_EX1:
	case SIMCONNECT_RECV_ID_EVENT:
	{
		SIMCONNECT_RECV_EVENT* evt = (SIMCONNECT_RECV_EVENT*)pData;
		switch (evt->uEventID) {
		case EVENT_SIM:
			status->sim_running = (bool)evt->dwData;
			if (!status->sim_running && status->in_sim) {
				status->in_sim = FALSE;
				if (status->recording)
					stop_recording(status);
			}
			break;
		case EVENT_PAUSE:
			status->paused = (bool)evt->dwData;
			break;
		case EVENT_CRASHED:
			gui_log_printf(status, GUI_LOG_WARNING, "Plane crashed!");
			break;
		case EVENT_BRAKES:
		case EVENT_AP_AIRSPEED_HOLD:
		case EVENT_AP_AIRSPEED_OFF:
		case EVENT_AP_AIRSPEED_ON:
		case EVENT_AP_ALT_HOLD:
		case EVENT_AP_ALT_HOLD_OFF:
		case EVENT_AP_ALT_HOLD_ON:
		case EVENT_AP_APR_HOLD:
		case EVENT_AP_APR_HOLD_OFF:
		case EVENT_AP_APR_HOLD_ON:
		case EVENT_AP_HDG_HOLD:
		case EVENT_AP_HDG_HOLD_OFF:
		case EVENT_AP_HDG_HOLD_ON:
		case EVENT_AP_MACH_HOLD:
		case EVENT_AP_MACH_OFF:
		case EVENT_AP_MACH_ON:
		case EVENT_AP_MASTER:
		case EVENT_AP_PANEL_ALTITUDE_HOLD:
		case EVENT_AP_PANEL_ALTITUDE_OFF:
		case EVENT_AP_PANEL_ALTITUDE_ON:
		case EVENT_AP_PANEL_HEADING_HOLD:
		case EVENT_AP_PANEL_HEADING_OFF:
		case EVENT_AP_PANEL_HEADING_ON:
		case EVENT_AP_PANEL_MACH_HOLD:
		case EVENT_AP_PANEL_MACH_OFF:
		case EVENT_AP_PANEL_MACH_ON:
		case EVENT_AP_PANEL_SPEED_HOLD:
		case EVENT_AP_PANEL_SPEED_OFF:
		case EVENT_AP_PANEL_SPEED_ON:
		case EVENT_AP_PANEL_VS_OFF:
		case EVENT_AP_PANEL_VS_ON:
		case EVENT_AP_PANEL_VS_HOLD:
		case EVENT_AP_VS_HOLD:
		case EVENT_AP_VS_OFF:
		case EVENT_AP_VS_ON:
		case EVENT_AP_PANEL_SPEED_HOLD_TOGGLE:
		case EVENT_AP_PANEL_MACH_HOLD_TOGGLE:
		case EVENT_AUTOPILOT_DISENGAGE_TOGGLE:
		case EVENT_AUTOPILOT_OFF:
		case EVENT_AUTOPILOT_ON:
		case EVENT_AUTOPILOT_PANEL_AIRSPEED_SET:
		case EVENT_FLIGHT_LEVEL_CHANGE:
		case EVENT_FLIGHT_LEVEL_CHANGE_OFF:
		case EVENT_FLIGHT_LEVEL_CHANGE_ON:
		case EVENT_AUTO_THROTTLE_ARM:
		case EVENT_AUTO_THROTTLE_TO_GA:
		case EVENT_AUTOBRAKE_DISARM:
		case EVENT_AUTOBRAKE_HI_SET:
		case EVENT_AUTOBRAKE_LO_SET:
		case EVENT_AUTOBRAKE_MED_SET:
		case EVENT_GPWS_SWITCH_TOGGLE:
		case EVENT_TOGGLE_FLIGHT_DIRECTOR:
		case EVENT_APU_BLEED_AIR_SOURCE_TOGGLE:
		case EVENT_APU_GENERATOR_SWITCH_TOGGLE:
		case EVENT_APU_OFF_SWITCH:
		case EVENT_APU_STARTER:
		case EVENT_ANTI_ICE_ON:
		case EVENT_ANTI_ICE_OFF:
		case EVENT_ANTI_ICE_TOGGLE:
		case EVENT_ANTI_ICE_TOGGLE_ENG1:
		case EVENT_ANTI_ICE_TOGGLE_ENG2:
		case EVENT_THROTTLE_REVERSE_THRUST_TOGGLE:
		case EVENT_FLAPS_DECR:
		case EVENT_FLAPS_DOWN:
		case EVENT_FLAPS_INCR:
		case EVENT_FLAPS_UP:
		case EVENT_SPOILERS_ARM_OFF:
		case EVENT_SPOILERS_ARM_ON:
		case EVENT_SPOILERS_ARM_TOGGLE:
		case EVENT_SPOILERS_OFF:
		case EVENT_SPOILERS_ON:
		case EVENT_SPOILERS_TOGGLE:
		case EVENT_CROSS_FEED_TOGGLE:
		case EVENT_GEAR_DOWN:
		case EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE:
		case EVENT_GEAR_TOGGLE:
		case EVENT_GEAR_UP:
		case EVENT_PARKING_BRAKES:
		case EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE:
		case EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE:
		case EVENT_WINDSHIELD_DEICE_OFF:
		case EVENT_WINDSHIELD_DEICE_ON:
		case EVENT_WINDSHIELD_DEICE_TOGGLE:
		case EVENT_TOGGLE_AVIONICS_MASTER:
			// Flood detection runs regardless of trip state; whether a trip
			// exists to attach this occurrence to is decided per-occurrence
			// by commit_event(), not here.
			status->event_filter.record(EVENT_ID_TXT[evt->uEventID], status->id_trip,
				status->data.time_zulu.format_date_time(), status->data.time_local.format_date_time(),
				event_output(status));
			break;
		default:
			gui_log_printf(status, GUI_LOG_WARNING, "Unknown event ID: %ld", evt->uEventID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
	{
		SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
		switch (pObjData->dwRequestID) {
		case REQUEST_FLIGHT:
		{
			// Piggybacks on this periodic tick to resolve flood-filter entries
			// whose quiet period has elapsed, so no dedicated timer is needed.
			status->event_filter.flush_stale(event_output(status));
			struct FLIGHT_DATA_RECORD tmp;
			memset(&tmp, 0, sizeof(struct FLIGHT_DATA_RECORD));
			// The wire payload covers every field except the last one:
			// add_flight_definition() registers no ZULU counterpart of "TIME ZONE
			// OFFSET", so time_zulu.timezone_offset is never sent.
			static_assert(offsetof(DATETIME, timezone_offset) + sizeof(double) == sizeof(DATETIME)
				&& offsetof(FLIGHT_DATA_RECORD, time_zulu) + sizeof(DATETIME) == sizeof(FLIGHT_DATA_RECORD),
				"time_zulu.timezone_offset must be the last byte range of FLIGHT_DATA_RECORD");
			memcpy(&tmp, &pObjData->dwData, sizeof(struct FLIGHT_DATA_RECORD) - sizeof(double));
			// SimConnect returns pitch and bank inverted from aviation convention:
			//   pitch: positive = nose down  → negate to positive = nose up
			//   bank:  positive = left wing down → negate to positive = right bank
			// Negate here so all downstream code — DB, charts, data table, touchdown
			// records — uses the standard aviation sign convention.
			tmp.plane_pitch_degrees = -tmp.plane_pitch_degrees;
			tmp.plane_touchdown_pitch_degrees = -tmp.plane_touchdown_pitch_degrees;
			tmp.plane_bank_degrees = -tmp.plane_bank_degrees;
			tmp.plane_touchdown_bank_degrees = -tmp.plane_touchdown_bank_degrees;
			flight_on_sample(status, tmp);
		}
		break;
		default:
			gui_log_printf(status, GUI_LOG_WARNING, "SIMCONNECT_RECV_SIMOBJECT_DATA: %d", pObjData->dwRequestID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_AIRPORT_LIST:
		lookup_on_airport_list(status, (SIMCONNECT_RECV_AIRPORT_LIST*)pData);
		break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA:
		lookup_on_facility_data(status, (SIMCONNECT_RECV_FACILITY_DATA*)pData);
		break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA_END:
		lookup_on_facility_data_end(status);
		break;
	case SIMCONNECT_RECV_ID_EXCEPTION: {
		// SimConnect reports failed AddToDataDefinition/MapClientEventToSimEvent/
		// RequestDataOnSimObject calls asynchronously here rather than through
		// their own (synchronous, "queued OK") return values, so this is the
		// only place a bad simvar/event name from an SDK or aircraft-SDK change
		// would ever surface -- silently dropping it would desync the data
		// definition's field ordering with zero trace in the log.
		SIMCONNECT_RECV_EXCEPTION* except = (SIMCONNECT_RECV_EXCEPTION*)pData;
		gui_log_printf(status, GUI_LOG_WARNING,
			"SimConnect exception SIMCONNECT_EXCEPTION_%s (%lu) (SendID=%lu, Index=%lu)",
			simconnect_exception_txt(except->dwException), except->dwException,
			except->dwSendID, except->dwIndex);
		// If this exception corresponds to the currently outstanding facility-lookup
		// request (matched by SendID -- see lookup.send_id in types.h), the
		// lookup will never receive its normal terminal response (the AIRPORT_LIST
		// no-match branch or FACILITY_DATA_END), so without this lookup.pending
		// would stay stuck true forever, silently disabling all future departure/
		// destination airport-runway resolution for the rest of the app session.
		lookup_on_exception(status, except->dwSendID);
		break;
	}
	default:
		gui_log_printf(status, GUI_LOG_WARNING, "SIMCONNECT_RECV: %d", pData->dwID);
		break;
	}
	} catch (const db_exception& e) {
		gui_log_printf(status, GUI_LOG_WARNING, "Database error in dispatch: %s", e.message.c_str());
	} catch (...) {
		// Catch-all, not just db_exception -- this callback also does raw SimConnect
		// data handling, malloc/free, and geodesic math, any of which could throw
		// something else. Left uncaught, it would escape a Qt timer slot and likely
		// terminate the app instead of just logging and continuing.
		gui_log_printf(status, GUI_LOG_WARNING, "Unknown error in dispatch");
	}
}

#include "recorder.h"
#include "db.h"
#include "gui_notify.h"
#include "logger.h"
#include <chrono>
#include <thread>

static bool is_skipped_event(struct STATUS* status, const char* eventName) {
	return status->skip_events.count(eventName) > 0;
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

void stop_recording(struct STATUS* status) {
	gui_log_printf(status, GUI_LOG_TRACE, "stop_recording: trip=%d, last_sample=%s",
		status->id_trip, status->last_sample != NULL ? "present" : "none");
	status->recording = FALSE;
	status->no_trip_events_logged.clear();
	// Destination lat/lon was written at each touchdown; only the arrival time
	// (engine shutdown) is set here — consistent with departure time being engine start.
	// last_sample is NULL if the trip ended before a single sample was ever
	// recorded (e.g. engine start immediately followed by engine cutoff, within
	// one sample interval) -- there's no flight data to source a destination
	// time from, so leave those columns unset instead of dereferencing NULL.
	if (status->last_sample != NULL) {
		// Caught, not left to propagate: everything below this point (freeing
		// touchdown_data, resetting id_trip, pushing the end-of-trip barrier)
		// must still run even if this UPDATE fails, or the next trip to start
		// inherits this one's dangling touchdown list/id_trip. Two of this
		// function's three callers (RecorderBridge destructor/shutdown()) have
		// no try/catch of their own, so an uncaught db_exception here would
		// otherwise crash the app on quit instead of just losing one UPDATE.
		try {
			db_insert_update_table(
				status->sql,
				"UPDATE trips SET destination_zulu_time=?,destination_local_time=? WHERE id=?;",
				status->last_sample,
				status,
				NULL,
				[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
					struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
					db_bind(stmt, stmt_txt, 1, pS->time_zulu.format_date_time().c_str());
					db_bind(stmt, stmt_txt, 2, pS->time_local.format_date_time().c_str());
					db_bind(stmt, stmt_txt, 3, status->id_trip);
				}
			);
		} catch (const db_exception& e) {
			gui_log_printf(status, GUI_LOG_WARNING, "stop_recording: failed to write destination time (trip %d): %s",
				status->id_trip, e.message.c_str());
		}
	}
	// trip_touchdowns rows were already inserted at touchdown time; just free the list.
	while (status->touchdown_data != NULL) {
		struct TOUCHDOWN_DATA* cur = status->touchdown_data;
		status->touchdown_data = status->touchdown_data->next;
		cur->airport.clear();
		free(cur);
	}
	status->touchdown_data_end = NULL;
	// trip_takeoffs rows for touch-and-go markers were already inserted at
	// takeoff time; just free the list. The trip's single departure record
	// (status->departure/departure_db_id) is untouched -- it isn't part of
	// this list.
	while (status->takeoff_data != NULL) {
		struct TAKEOFF_DATA* cur = status->takeoff_data;
		status->takeoff_data = status->takeoff_data->next;
		cur->airport.clear();
		free(cur);
	}
	status->takeoff_data_end = NULL;
	// A lookup this trip skipped (because another one was still in flight) and
	// meant to retry later is now moot -- the trip that needed it is gone.
	// Note this deliberately leaves facility_lookup_pending/facility_lookup_trip_id
	// untouched: if this trip's own lookup is still in flight, it must stay
	// pending so a new trip's takeoff doesn't race it, and the staleness checks
	// in MyDispatchProc recognize and drop that response once it does arrive.
	status->facility_lookup_departure_needed = FALSE;
	int ended_trip_id = status->id_trip;
	// Marks this trip as still-draining until db_write_worker processes the
	// barrier pushed below, so the UI can keep treating it as undeletable even
	// though id_trip (reset next) will already say no trip is live -- see
	// flushing_trip_ids in types.h.
	{
		std::lock_guard<std::mutex> lock(status->flushing_trip_ids_mutex);
		status->flushing_trip_ids.insert(ended_trip_id);
	}
	// Reset id_trip synchronously (not from the worker thread) so a new trip
	// starting right after this one can never have its dispatch-callback event
	// logging (see the id_trip > 0 gate above) mistaken for the ended trip's.
	status->id_trip = -1;
	// Push an end-of-trip barrier instead of flushing here: the worker thread
	// still has this trip's earlier samples queued ahead of this entry, and
	// processes everything strictly in order, so "Recording stopped" and the
	// GUI notification only fire once every sample has actually been written.
	// A new trip's samples pushed after this point queue up safely behind it
	// -- there's nothing to race, since it's the same one worker thread and
	// the same queue for every trip.
	status->sample_write_queue.push(NULL, ended_trip_id);
}

// Signals the DB-write worker to drain and exit, then joins it. Callers must
// call this before closing/nulling status->sql.
void wait_for_db_writers(struct STATUS* status) {
	status->sample_write_queue.stop();
	if (status->db_writer_thread.joinable())
		status->db_writer_thread.join();
}

// Called whenever the shared facility-lookup slot becomes free (from the tail
// end of any terminal SIMCONNECT_RECV_ID_AIRPORT_LIST/FACILITY_DATA_END
// outcome, with facility_lookup_pending already cleared). If a takeoff or
// touchdown happened while a previous lookup was still in flight, its own
// SimConnect_RequestFacilitiesList_EX1 call was skipped to avoid racing the
// in-flight one (see facility_lookup_pending in types.h) -- this picks it
// back up immediately. Departure takes priority since it always happens
// first within a trip; touchdowns are then matched in the same FIFO order
// FACILITY_DATA_END uses to attach a resolved lookup to a touchdown row,
// which requires strict in-order resolution -- skipping straight to a later
// touchdown here would attribute its resolved airport/runway to an earlier,
// still-unresolved one instead.
static void request_next_touchdown_facility_lookup(struct STATUS* status) {
	if (status->facility_lookup_pending)
		return;
	if (status->facility_lookup_departure_needed) {
		gui_log_printf(status, GUI_LOG_TRACE, "Facility lookup slot free: picking up deferred departure lookup (trip %d)", status->id_trip);
		status->facility_lookup_departure_needed = FALSE;
		status->facility_lookup_is_takeoff = FALSE;
		status->facility_lookup_is_departure = TRUE;
		status->facility_lookup_pending = TRUE;
		status->facility_lookup_trip_id = status->id_trip;
		status->facility_lookup_coordinate = status->facility_lookup_departure_coordinate;
		status->facility_lookup_heading = status->facility_lookup_departure_heading;
		SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
		SimConnect_GetLastSentPacketID(status->hSimConnect, &status->facility_lookup_send_id);
		return;
	}
	struct TOUCHDOWN_DATA* next_td = status->touchdown_data;
	while (next_td != NULL && next_td->airport.runway_act.distances[0] != -1)
		next_td = next_td->next;
	struct TAKEOFF_DATA* next_to = status->takeoff_data;
	while (next_to != NULL && next_to->airport.runway_act.distances[0] != -1)
		next_to = next_to->next;
	if (next_td == NULL && next_to == NULL)
		return;
	// Both lists are each individually in chronological order (appended at
	// their own tail), but interleaved with each other -- e.g. a touch-and-go
	// produces takeoff, touchdown, takeoff in that order. seq (shared across
	// both lists, see types.h) picks whichever of the two earliest-unresolved
	// candidates actually happened first, so FACILITY_DATA_END's strict
	// in-order-resolution assumption still holds across the combined stream.
	bool pick_takeoff = next_td == NULL || (next_to != NULL && next_to->seq < next_td->seq);
	status->facility_lookup_is_takeoff = pick_takeoff;
	status->facility_lookup_is_departure = FALSE;
	gui_log_printf(status, GUI_LOG_TRACE, "Facility lookup slot free: picking up queued %s lookup (trip %d)",
		pick_takeoff ? "takeoff" : "touchdown", status->id_trip);
	status->facility_lookup_pending = TRUE;
	status->facility_lookup_trip_id = status->id_trip;
	status->facility_lookup_coordinate = pick_takeoff ? next_to->flight_data.coordinate : next_td->flight_data.coordinate;
	status->facility_lookup_heading = pick_takeoff ? next_to->flight_data.heading : next_td->flight_data.heading;
	SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
	SimConnect_GetLastSentPacketID(status->hSimConnect, &status->facility_lookup_send_id);
}

// Resolves which AIRPORT slot the in-flight facility lookup (AIRPORT_LIST /
// FACILITY_DATA / FACILITY_DATA_END / EXCEPTION) targets. Deliberately reads
// only facility_lookup_is_departure/facility_lookup_is_takeoff -- both
// captured once, at the same three call sites that start a lookup (the
// deferred-departure and immediate-departure branches above, and the
// queued-pickup branch in request_next_touchdown_facility_lookup()) -- rather
// than any live/mutable state such as status->departure.runway_act.index.
// That field used to be used for this instead, but it can be reset to -1 by
// a *later* trip's status->departure.clear() while an older trip's takeoff-
// marker or destination lookup is still in flight, which would misattribute
// the stale response to &status->departure. See facility_lookup_is_departure
// in types.h for the full history.
static AIRPORT* facility_lookup_target(struct STATUS* status) {
	if (status->facility_lookup_is_departure)
		return &status->departure;
	// Takeoff-marker and touchdown/destination lookups use separate scratch
	// objects (status->takeoff_scratch vs. status->destination) even though
	// they're never in flight at the same time -- see destination's/
	// takeoff_scratch's declarations in types.h for why they're kept apart
	// instead of sharing one field.
	return status->facility_lookup_is_takeoff ? &status->takeoff_scratch : &status->destination;
}

// Human-readable label for gui_log_printf tracing, matching whichever slot
// facility_lookup_target() returned for this same in-flight lookup.
static const char* facility_lookup_target_label(struct STATUS* status, AIRPORT* apt) {
	return (apt == &status->departure) ? "departure" : status->facility_lookup_is_takeoff ? "takeoff" : "destination";
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
			if (status->id_trip > 0 && !is_skipped_event(status, EVENT_ID_TXT[evt->uEventID])) {
				gui_log_printf(status, GUI_LOG_INFO, "Event: %s", EVENT_ID_TXT[evt->uEventID]);
				std::string tz = status->data.time_zulu.format_date_time();
				std::string tl = status->data.time_local.format_date_time();
				db_insert_event(status, EVENT_ID_TXT[evt->uEventID], tz.c_str(), tl.c_str());
			} else if (status->id_trip > 0) {
				if (status->skip_events_logged.insert(EVENT_ID_TXT[evt->uEventID]).second) {
					gui_log_printf(status, GUI_LOG_TRACE, "Event skipped (in skip_events list): %s (further occurrences won't be logged)", EVENT_ID_TXT[evt->uEventID]);
				}
			} else {
				if (status->no_trip_events_logged.insert(EVENT_ID_TXT[evt->uEventID]).second) {
					gui_log_printf(status, GUI_LOG_TRACE, "Event ignored (no active trip): %s (further occurrences won't be logged)", EVENT_ID_TXT[evt->uEventID]);
				}
			}
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
			struct FLIGHT_DATA_RECORD tmp;
			memset(&tmp, 0, sizeof(struct FLIGHT_DATA_RECORD));
			memcpy(&tmp, &pObjData->dwData, sizeof(struct FLIGHT_DATA_RECORD) - sizeof(double) - sizeof(struct FLIGHT_DATA_RECORD*));
			// SimConnect returns pitch and bank inverted from aviation convention:
			//   pitch: positive = nose down  → negate to positive = nose up
			//   bank:  positive = left wing down → negate to positive = right bank
			// Negate here so all downstream code — DB, charts, data table, touchdown
			// records — uses the standard aviation sign convention.
			tmp.plane_pitch_degrees = -tmp.plane_pitch_degrees;
			tmp.plane_touchdown_pitch_degrees = -tmp.plane_touchdown_pitch_degrees;
			tmp.plane_bank_degrees = -tmp.plane_bank_degrees;
			tmp.plane_touchdown_bank_degrees = -tmp.plane_touchdown_bank_degrees;
			status->data.altitude = (int)tmp.plane_altitude;
			status->data.heading = (int)tmp.plane_heading_degrees_magnetic;
			status->data.speed = (int)tmp.airspeed_indicated;
			status->data.vertical_speed = (int)tmp.vertical_speed;
			status->data.bank = tmp.plane_bank_degrees;
			status->data.pitch = tmp.plane_pitch_degrees;
			status->data.g_force = tmp.g_force;
			status->data.coordinate = tmp.plane_coordinate;
			status->data.time_zulu = tmp.time_zulu;
			status->data.time_local = tmp.time_local;
			// Only touchdown/destination runway-end matching uses this -- see
			// the bearing_tra computation in FACILITY_DATA_END below for why
			// departure/takeoff never can (their only candidate crossing of
			// this band happens during climb-out, after liftoff, not before).
			if (tmp.radio_height > 50 && tmp.radio_height < 100) {
				status->loc_dh.latitude = tmp.plane_coordinate.latitude;
				status->loc_dh.longitude = tmp.plane_coordinate.longitude;
			} else if (tmp.radio_height >= 100 && status->loc_dh.latitude != 360) {
				// Invalidate once the aircraft climbs clear of the capture band above,
				// so a touchdown can never reuse a position frozen on a different,
				// earlier low pass (e.g. an earlier circuit/go-around in the same
				// trip). This runs every sim frame (SIMCONNECT_PERIOD_SIM_FRAME,
				// registered once at connect time via SimConnect_RequestDataOnSimObject),
				// so the latitude != 360 guard clears it exactly
				// once per climb-out rather than every frame for the rest of the time
				// spent above 100ft -- most of the flight. If this trip's next descent
				// happens to skip resampling inside the 50-100ft band (a sim-frame
				// hitch/stall), FACILITY_DATA_END's loc_dh_source->latitude != 360
				// check below then falls back to heading-based bearing instead of
				// silently reusing this now-cleared, stale position.
				status->loc_dh.clear();
			}
			if (status->sim_running && !status->paused && tmp.surface_type != 255) {
				status->in_sim = TRUE;
				if ((bool)tmp.sim_on_ground) {
					if ((bool)tmp.eng_combustion_1 || (bool)tmp.eng_combustion_2) {
						if (!status->recording) {
							status->recording = TRUE;
							gui_log_printf(status, GUI_LOG_INFO, "Recording started");

							// No queue state to reset here -- sample_write_queue is shared
							// across trips by design (see stop_recording()'s end-of-trip
							// barrier), so a previous trip's still-draining samples are
							// simply ahead of this trip's in line, not something this trip
							// needs to wait for or clear out.
							if (status->last_sample != NULL) {
								free(status->last_sample);
								status->last_sample = NULL;
							}

							status->departure.clear();
							status->destination.clear();
							// Also cleared here (unlike departure/destination above, this
							// wasn't previously): if a takeoff-marker lookup targeting this
							// object is still in flight when this trip ends, the flag reset
							// below makes facility_lookup_target() resolve that lookup's late
							// response to &status->destination instead once it arrives,
							// leaking this object's runways buffer since nothing then frees
							// it. Unconditionally free+null-ing it here up front, the same as
							// departure/destination, closes that regardless of which slot the
							// stale response ends up misattributed to.
							status->takeoff_scratch.clear();
							status->departure_lookup_initiated = FALSE;
							// Not a fix for an observed bug on its own -- every call site that
							// starts a lookup already sets both of these fresh before use (see
							// facility_lookup_is_departure in types.h) -- but a lookup still in
							// flight when this trip boundary is crossed reads them again when
							// its (possibly stale) response arrives later; see the
							// takeoff_scratch.clear() above for why that case needs its target
							// AIRPORT cleared here too, not just these flags.
							status->facility_lookup_is_takeoff = FALSE;
							status->facility_lookup_is_departure = FALSE;
							status->departure_db_id = -1;
							// A go-around or bounced landing from a previous trip can leave
							// this set -- between trips (engines off, on the ground) radio_height
							// stays well under 100ft, so neither of the two automatic resets above
							// (a fresh 50-100ft crossing, or climbing back above 100ft -- see the
							// SIMOBJECT_DATA handler above) reliably fires during that window.
							// Without clearing it here, a fast climbout on this new trip that skips
							// back through the 50-100ft band between samples would attribute this
							// trip's departure runway bearing to the previous trip's stale position.
							status->loc_dh.clear();
							status->next_facility_lookup_seq = 0;
							status->airborne = !(bool)tmp.sim_on_ground;

							// If this throws, status->id_trip is never assigned -- recording
							// must not stay TRUE in that case, or every sample from here on
							// gets tagged with whatever stale id_trip was left over (-1, or
							// an already-ended previous trip) instead of a real one. Reverting
							// to FALSE makes this same block retry on the next sample tick
							// (engine/on-ground state is unchanged) rather than silently
							// recording under the wrong trip for the rest of the flight.
							try {
								db_insert_update_table(
									status->sql,
									"INSERT INTO trips ("
									"title,"
									"atc_airline,"
									"atc_flight_number,"
									"atc_id,"
									"atc_model,"
									"atc_type,"
									"departure_latitude,"
									"departure_longitude,"
									"departure_zulu_time,"
									"departure_local_time"
									") VALUES (?,?,?,?,?,?,?,?,?,?);",
									&tmp,
									status,
									NULL,
									[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
										struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
										db_bind(stmt, stmt_txt, 1, pS->title);
										db_bind(stmt, stmt_txt, 2, pS->atc_airline);
										db_bind(stmt, stmt_txt, 3, pS->atc_flight_number);
										db_bind(stmt, stmt_txt, 4, pS->atc_id);
										db_bind(stmt, stmt_txt, 5, pS->atc_model);
										db_bind(stmt, stmt_txt, 6, pS->atc_type);
										db_bind(stmt, stmt_txt, 7, pS->plane_coordinate.latitude);
										db_bind(stmt, stmt_txt, 8, pS->plane_coordinate.longitude);
										db_bind(stmt, stmt_txt, 9, pS->time_zulu.format_date_time().c_str());
										db_bind(stmt, stmt_txt, 10, pS->time_local.format_date_time().c_str());
									},
									&status->id_trip
								);
								gui_notify_recording_changed(status, true, status->id_trip);
							} catch (const db_exception& e) {
								status->recording = FALSE;
								gui_log_printf(status, GUI_LOG_WARNING, "Recording start failed (trip insert): %s", e.message.c_str());
							}
						}
					} else {
						if (status->recording)
							stop_recording(status);
					}
				}
			}
			if (status->recording && !status->paused) {
				// Takeoff
				// Guarded by departure_lookup_initiated (set right below), not by
				// departure.runway_act.index == -1 -- that field only flips once the
				// departure lookup actually *resolves*, so a touch-and-go (or several
				// full-stop taxi-back-and-takeoffs) occurring before a slow lookup
				// resolves would otherwise still see -1 and be mistaken for a fresh
				// departure. See departure_lookup_initiated in types.h.
				if (!(bool)tmp.sim_on_ground && !status->airborne && !status->departure_lookup_initiated) {
					status->departure_lookup_initiated = TRUE;
					status->next_facility_lookup_seq++;
					// Captured now (the actual takeoff moment) regardless of whether
					// the lookup fires immediately below or is deferred -- see
					// facility_lookup_departure_coordinate in types.h.
					status->facility_lookup_departure_coordinate = status->data.coordinate;
					status->facility_lookup_departure_heading = status->data.heading;
					gui_log_printf(status, GUI_LOG_TRACE, "Takeoff detected (trip %d): lat=%.6f, lon=%.6f, heading=%.1f",
						status->id_trip, status->facility_lookup_departure_coordinate.latitude,
						status->facility_lookup_departure_coordinate.longitude, status->facility_lookup_departure_heading);
					// Insert immediately so the row survives a crash before stop_recording.
					// Airport/runway fields are NULL until the facility callback resolves --
					// same immediate-INSERT-then-async-UPDATE pattern as touchdowns below.
					db_insert_update_table(status->sql,
						"INSERT INTO trip_takeoffs ("
						"trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,"
						"plane_bank_degrees,heading_indicator,plane_latitude,plane_longitude,"
						"wind_direction,wind_velocity,time_zulu,time_local"
						") VALUES (?,?,?,?,?,?,?,?,?,?,?,?);",
						&tmp, status, NULL,
						[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
							struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
							db_bind(stmt, stmt_txt, 1, status->id_trip);
							db_bind(stmt, stmt_txt, 2, (int)pS->airspeed_indicated);
							db_bind(stmt, stmt_txt, 3, (int)pS->vertical_speed);
							db_bind(stmt, stmt_txt, 4, pS->plane_pitch_degrees);
							db_bind(stmt, stmt_txt, 5, pS->plane_bank_degrees);
							db_bind(stmt, stmt_txt, 6, (int)pS->plane_heading_degrees_magnetic);
							db_bind(stmt, stmt_txt, 7, pS->plane_coordinate.latitude);
							db_bind(stmt, stmt_txt, 8, pS->plane_coordinate.longitude);
							db_bind(stmt, stmt_txt, 9, (int)pS->ambient_wind_direction);
							db_bind(stmt, stmt_txt, 10, (int)pS->ambient_wind_velocity);
							db_bind(stmt, stmt_txt, 11, pS->time_zulu.format_date_time().c_str());
							db_bind(stmt, stmt_txt, 12, pS->time_local.format_date_time().c_str());
						},
						&status->departure_db_id
					);
					gui_log_printf(status, GUI_LOG_TRACE, "Takeoff trip_takeoffs row inserted: db_id=%d", status->departure_db_id);
					if (!status->facility_lookup_pending) {
						gui_log_printf(status, GUI_LOG_TRACE, "Requesting departure facility lookup (trip %d)", status->id_trip);
						status->facility_lookup_is_takeoff = FALSE;
						status->facility_lookup_is_departure = TRUE;
						status->facility_lookup_pending = TRUE;
						status->facility_lookup_trip_id = status->id_trip;
						status->facility_lookup_coordinate = status->facility_lookup_departure_coordinate;
						status->facility_lookup_heading = status->facility_lookup_departure_heading;
						SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
						SimConnect_GetLastSentPacketID(status->hSimConnect, &status->facility_lookup_send_id);
					} else {
						// A previous trip's lookup is still draining (see
						// facility_lookup_departure_needed in types.h) -- this
						// takeoff only fires once per trip, so if the request is
						// skipped now it must be retried later rather than lost.
						gui_log_printf(status, GUI_LOG_TRACE, "Deferring departure facility lookup (trip %d): another lookup in flight", status->id_trip);
						status->facility_lookup_departure_needed = TRUE;
					}
				} else if (!(bool)tmp.sim_on_ground && !status->airborne) {
					// Touch-and-go / subsequent takeoff: this trip's departure is
					// already locked in (departure_lookup_initiated above), so this
					// is recorded purely as a takeoff *marker* occurrence -- same
					// immediate-INSERT-then-async-UPDATE pattern as a touchdown, but
					// with no trips.* update (a trip has exactly one departure).
					gui_log_printf(status, GUI_LOG_TRACE, "Takeoff detected (trip %d, touch-and-go): lat=%.6f, lon=%.6f, heading=%.1f",
						status->id_trip, tmp.plane_coordinate.latitude, tmp.plane_coordinate.longitude,
						tmp.plane_heading_degrees_magnetic);
					struct TAKEOFF_DATA* tmp_takeoff = (struct TAKEOFF_DATA*)malloc(sizeof(struct TAKEOFF_DATA));
					if (tmp_takeoff == NULL) {
						gui_log_printf(status, GUI_LOG_WARNING, "Takeoff: malloc failed for takeoff record; this takeoff marker will not be recorded");
					} else {
						memset(tmp_takeoff, 0, sizeof(struct TAKEOFF_DATA));
						tmp_takeoff->airport.clear();
						// Same reasoning as TOUCHDOWN_DATA::db_id above: memset zeroed
						// this to 0, not the declared default of -1.
						tmp_takeoff->db_id = -1;
						tmp_takeoff->seq = status->next_facility_lookup_seq++;
						if (status->takeoff_data == NULL) {
							status->takeoff_data = tmp_takeoff;
							status->takeoff_data_end = tmp_takeoff;
						} else {
							status->takeoff_data_end->next = tmp_takeoff;
							status->takeoff_data_end = tmp_takeoff;
						}
						status->takeoff_data_end->flight_data.heading = (int)tmp.plane_heading_degrees_magnetic;
						status->takeoff_data_end->flight_data.pitch = tmp.plane_pitch_degrees;
						status->takeoff_data_end->flight_data.bank = tmp.plane_bank_degrees;
						status->takeoff_data_end->flight_data.speed = (int)tmp.airspeed_indicated;
						status->takeoff_data_end->flight_data.vertical_speed = (int)tmp.vertical_speed;
						status->takeoff_data_end->flight_data.wind_direction = (int)tmp.ambient_wind_direction;
						status->takeoff_data_end->flight_data.wind_velocity = (int)tmp.ambient_wind_velocity;
						status->takeoff_data_end->flight_data.coordinate.latitude = tmp.plane_coordinate.latitude;
						status->takeoff_data_end->flight_data.coordinate.longitude = tmp.plane_coordinate.longitude;
						status->takeoff_data_end->flight_data.time_zulu = tmp.time_zulu;
						status->takeoff_data_end->flight_data.time_local = tmp.time_local;
						status->takeoff_data_end->airport.runway_act.distances[0] = -1;
						// Insert immediately so the row survives a crash before stop_recording.
						// Airport/runway fields are NULL until the facility callback resolves.
						db_insert_update_table(status->sql,
							"INSERT INTO trip_takeoffs ("
							"trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,"
							"plane_bank_degrees,heading_indicator,plane_latitude,plane_longitude,"
							"wind_direction,wind_velocity,time_zulu,time_local"
							") VALUES (?,?,?,?,?,?,?,?,?,?,?,?);",
							status->takeoff_data_end, status, NULL,
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TAKEOFF_DATA* pS = (struct TAKEOFF_DATA*)data;
								db_bind(stmt, stmt_txt, 1, status->id_trip);
								db_bind(stmt, stmt_txt, 2, pS->flight_data.speed);
								db_bind(stmt, stmt_txt, 3, pS->flight_data.vertical_speed);
								db_bind(stmt, stmt_txt, 4, pS->flight_data.pitch);
								db_bind(stmt, stmt_txt, 5, pS->flight_data.bank);
								db_bind(stmt, stmt_txt, 6, pS->flight_data.heading);
								db_bind(stmt, stmt_txt, 7, pS->flight_data.coordinate.latitude);
								db_bind(stmt, stmt_txt, 8, pS->flight_data.coordinate.longitude);
								db_bind(stmt, stmt_txt, 9, pS->flight_data.wind_direction);
								db_bind(stmt, stmt_txt, 10, pS->flight_data.wind_velocity);
								db_bind(stmt, stmt_txt, 11, pS->flight_data.time_zulu.format_date_time().c_str());
								db_bind(stmt, stmt_txt, 12, pS->flight_data.time_local.format_date_time().c_str());
							},
							&status->takeoff_data_end->db_id
						);
						gui_log_printf(status, GUI_LOG_TRACE, "Takeoff marker trip_takeoffs row inserted: db_id=%d", status->takeoff_data_end->db_id);
						gui_notify_trip_updated(status);
						// No-op if a previous lookup (departure, an earlier touchdown, or
						// an earlier takeoff marker) is still resolving -- this takeoff's
						// lookup will be picked up automatically once that one completes,
						// via request_next_touchdown_facility_lookup().
						request_next_touchdown_facility_lookup(status);
					}
				}
				// Landing
				if ((bool)tmp.sim_on_ground && status->airborne) {
					gui_log_printf(status, GUI_LOG_TRACE, "Touchdown detected (trip %d): lat=%.6f, lon=%.6f, heading=%d",
						status->id_trip, tmp.plane_coordinate.latitude, tmp.plane_coordinate.longitude,
						(int)tmp.plane_heading_degrees_magnetic);
					struct TOUCHDOWN_DATA* tmp_touchdown = (struct TOUCHDOWN_DATA*)malloc(sizeof(struct TOUCHDOWN_DATA));
					if (tmp_touchdown == NULL) {
						// Same handling as the takeoff-marker malloc failure above: log
						// and move on rather than aborting the rest of this tick (this
						// case used to duplicate status->airborne's update and break
						// out here instead, silently dropping the regular flight-data
						// sample below too -- a separate, unrelated allocation -- for
						// no stated reason). status->airborne is still updated
						// unconditionally right after this "Landing" block either way.
						gui_log_printf(status, GUI_LOG_WARNING, "Landing: malloc failed for touchdown record; this touchdown will not be recorded");
					} else {
						memset(tmp_touchdown, 0, sizeof(struct TOUCHDOWN_DATA));
						tmp_touchdown->airport.clear();
						// memset zeroed this to 0, not TOUCHDOWN_DATA::db_id's declared
						// default of -1 (malloc+memset never runs the member initializer).
						// db_id is only overwritten with the real rowid if the immediate
						// INSERT below succeeds; if it throws, db_id must stay -1 (an
						// invalid rowid) rather than 0, which would make a later
						// "UPDATE trip_touchdowns ... WHERE id = 0" resolve to no rows
						// and silently drop the touchdown's icao/runway match instead of
						// visibly failing.
						tmp_touchdown->db_id = -1;
						tmp_touchdown->seq = status->next_facility_lookup_seq++;
						if (status->touchdown_data == NULL) {
							status->touchdown_data = tmp_touchdown;
							status->touchdown_data_end = tmp_touchdown;
						} else {
							status->touchdown_data_end->next = tmp_touchdown;
							status->touchdown_data_end = tmp_touchdown;
						}
						status->touchdown_data_end->flight_data.heading = (int)tmp.plane_heading_degrees_magnetic;
						status->touchdown_data_end->flight_data.pitch = tmp.plane_pitch_degrees;
						status->touchdown_data_end->flight_data.bank = tmp.plane_bank_degrees;
						status->touchdown_data_end->flight_data.speed = (int)tmp.airspeed_indicated;
						status->touchdown_data_end->flight_data.vertical_speed = (int)tmp.vertical_speed;
						status->touchdown_data_end->flight_data.g_force = tmp.g_force;
						status->touchdown_data_end->flight_data.wind_direction = (int)tmp.ambient_wind_direction;
						status->touchdown_data_end->flight_data.wind_velocity = (int)tmp.ambient_wind_velocity;
						status->touchdown_data_end->flight_data.coordinate.latitude = tmp.plane_coordinate.latitude;
						status->touchdown_data_end->flight_data.coordinate.longitude = tmp.plane_coordinate.longitude;
						status->touchdown_data_end->flight_data.time_zulu = tmp.time_zulu;
						status->touchdown_data_end->flight_data.time_local = tmp.time_local;
						// Freeze this touchdown's own low-altitude position now -- see
						// TOUCHDOWN_DATA::loc_dh in types.h for why status->loc_dh itself
						// can't be trusted once this touchdown's facility lookup is queued.
						status->touchdown_data_end->loc_dh = status->loc_dh;
						status->touchdown_data_end->airport.runway_act.distances[0] = -1;
						// Insert immediately so the row survives a crash before stop_recording.
						// Airport/runway fields are NULL until the facility callback resolves.
						db_insert_update_table(status->sql,
							"INSERT INTO trip_touchdowns ("
							"trip,airspeed_indicated,vertical_speed,g_force,plane_pitch_degrees,"
							"plane_bank_degrees,heading_indicator,plane_latitude,plane_longitude,"
							"wind_direction,wind_velocity,time_zulu,time_local"
							") VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);",
							status->touchdown_data_end, status, NULL,
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TOUCHDOWN_DATA* pS = (struct TOUCHDOWN_DATA*)data;
								db_bind(stmt, stmt_txt, 1, status->id_trip);
								db_bind(stmt, stmt_txt, 2, pS->flight_data.speed);
								db_bind(stmt, stmt_txt, 3, pS->flight_data.vertical_speed);
								db_bind(stmt, stmt_txt, 4, pS->flight_data.g_force);
								db_bind(stmt, stmt_txt, 5, pS->flight_data.pitch);
								db_bind(stmt, stmt_txt, 6, pS->flight_data.bank);
								db_bind(stmt, stmt_txt, 7, pS->flight_data.heading);
								db_bind(stmt, stmt_txt, 8, pS->flight_data.coordinate.latitude);
								db_bind(stmt, stmt_txt, 9, pS->flight_data.coordinate.longitude);
								db_bind(stmt, stmt_txt, 10, pS->flight_data.wind_direction);
								db_bind(stmt, stmt_txt, 11, pS->flight_data.wind_velocity);
								db_bind(stmt, stmt_txt, 12, pS->flight_data.time_zulu.format_date_time().c_str());
								db_bind(stmt, stmt_txt, 13, pS->flight_data.time_local.format_date_time().c_str());
							},
							&status->touchdown_data_end->db_id
						);
						gui_log_printf(status, GUI_LOG_TRACE, "Touchdown trip_touchdowns row inserted: db_id=%d", status->touchdown_data_end->db_id);
						// Update the destination position in trips to reflect this landing.
						db_insert_update_table(status->sql,
							"UPDATE trips SET destination_latitude=?,destination_longitude=? WHERE id=?;",
							status->touchdown_data_end, status, NULL,
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TOUCHDOWN_DATA* pS = (struct TOUCHDOWN_DATA*)data;
								db_bind(stmt, stmt_txt, 1, pS->flight_data.coordinate.latitude);
								db_bind(stmt, stmt_txt, 2, pS->flight_data.coordinate.longitude);
								db_bind(stmt, stmt_txt, 3, status->id_trip);
							}
						);
						gui_notify_trip_updated(status);
						// No-op if a previous lookup (this trip's departure, or an earlier
						// touchdown from a bounce/go-around) is still resolving -- this
						// touchdown's lookup will be picked up automatically once that one
						// completes, via request_next_touchdown_facility_lookup().
						request_next_touchdown_facility_lookup(status);
					}
				}
				status->airborne = !(bool)tmp.sim_on_ground;

				double delta_s = status->sample_interval_ms / 1000.0;
				if (status->last_sample != NULL)
					delta_s = tmp.time_zulu.time_day - status->last_sample->time_zulu.time_day;
				if (delta_s < 0)
					delta_s += 86400;
				if (delta_s >= status->sample_interval_ms / 1000.0) {
					struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)malloc(sizeof(struct FLIGHT_DATA_RECORD));
					if (pS == NULL) {
						gui_log_printf(status, GUI_LOG_WARNING, "malloc failed for sample record; dropping this sample");
						break;
					}
					memset(pS, 0, sizeof(struct FLIGHT_DATA_RECORD));
					memcpy(pS, &tmp, sizeof(struct FLIGHT_DATA_RECORD));
					gui_notify_sample(status, pS);
					// pS is handed off to the DB-write worker below, which owns it
					// from here and frees it once flushed. Keep our own copy so the
					// next delta_s calculation (and stop_recording()'s destination-
					// time update) don't touch memory the worker thread may be
					// using or have already freed.
					if (status->last_sample != NULL)
						free(status->last_sample);
					status->last_sample = (struct FLIGHT_DATA_RECORD*)malloc(sizeof(struct FLIGHT_DATA_RECORD));
					if (status->last_sample != NULL)
						memcpy(status->last_sample, pS, sizeof(struct FLIGHT_DATA_RECORD));
					else
						gui_log_printf(status, GUI_LOG_WARNING, "malloc failed for last_sample cache; next delta_s will use the default interval");
					status->sample_write_queue.push(pS, status->id_trip);
				}
			}
		}
		break;
		default:
			gui_log_printf(status, GUI_LOG_WARNING, "SIMCONNECT_RECV_SIMOBJECT_DATA: %d", pObjData->dwRequestID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_AIRPORT_LIST:
	{
		// Drop responses for a lookup issued by a trip that has since ended --
		// id_trip only ever changes on this same dispatch thread (stop_recording()/
		// new-trip-start), so this comparison is race-free. Applying it now would
		// write stale airport data into whatever trip is active today. A large
		// facility list is split across multiple AIRPORT_LIST callbacks sharing
		// one request (see dwEntryNumber/dwOutOf below) -- only run the
		// pending-clear + next-lookup cleanup once, on the last chunk, so an
		// earlier chunk's cleanup can't race a lookup it just started back into
		// "not pending" while that new lookup is genuinely still in flight.
		if (status->facility_lookup_trip_id != status->id_trip) {
			SIMCONNECT_RECV_AIRPORT_LIST* pStaleData = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
			if (pStaleData->dwEntryNumber + 1 == pStaleData->dwOutOf) {
				status->facility_lookup_pending = FALSE;
				// Same reason as every other terminal path below: a touchdown/departure
				// lookup may have been queued behind this (now-stale) one and would
				// otherwise sit stranded until some unrelated lookup happens to drain it.
				request_next_touchdown_facility_lookup(status);
			}
			break;
		}
		SIMCONNECT_RECV_AIRPORT_LIST* pWxData = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
		// SimConnect splits a large facility list (e.g. every airport in loaded
		// scenery, 1000+ entries) across multiple AIRPORT_LIST callbacks that
		// share one request (dwEntryNumber counts 0..dwOutOf-1). Deciding
		// "nearest airport" independently per chunk is wrong -- a later chunk
		// that happens to contain only distant airports would conclude "not
		// found" and terminate/overwrite the lookup a second time while the
		// real match from an earlier chunk was still being resolved. Instead,
		// accumulate the running top-N nearest across all chunks in status,
		// and only act once the last chunk has been folded in.
		if (pWxData->dwEntryNumber == 0) {
			for (int k = 0; k < STATUS::FACILITY_LIST_TOP_N; k++) {
				status->facility_lookup_top[k].distance = 1e9;
				status->facility_lookup_top[k].ident[0] = '\0';
				status->facility_lookup_top[k].region[0] = '\0';
			}
		}
		for (int i = 0; i < (int)pWxData->dwArraySize; i++) {
			struct SIMCONNECT_DATA_FACILITY_AIRPORT airport = pWxData->rgData[i];
			COORDINATE airport_loc;
			airport_loc.latitude = airport.Latitude;
			airport_loc.longitude = airport.Longitude;
			double distance = abs(status->facility_lookup_coordinate.distanceInKm2Coordinate(airport_loc));
			if (distance < status->facility_lookup_top[STATUS::FACILITY_LIST_TOP_N - 1].distance) {
				int pos = STATUS::FACILITY_LIST_TOP_N - 1;
				while (pos > 0 && status->facility_lookup_top[pos - 1].distance > distance) {
					status->facility_lookup_top[pos] = status->facility_lookup_top[pos - 1];
					pos--;
				}
				status->facility_lookup_top[pos].distance = distance;
				strncpy(status->facility_lookup_top[pos].ident, airport.Ident, sizeof(status->facility_lookup_top[pos].ident) - 1);
				status->facility_lookup_top[pos].ident[sizeof(status->facility_lookup_top[pos].ident) - 1] = '\0';
				strncpy(status->facility_lookup_top[pos].region, airport.Region, sizeof(status->facility_lookup_top[pos].region) - 1);
				status->facility_lookup_top[pos].region[sizeof(status->facility_lookup_top[pos].region) - 1] = '\0';
			}
		}
		// Wait for the rest of the (possibly multi-chunk) list before deciding.
		if (pWxData->dwEntryNumber + 1 < pWxData->dwOutOf)
			break;
		for (int k = 0; k < STATUS::FACILITY_LIST_TOP_N && status->facility_lookup_top[k].ident[0] != '\0'; k++) {
			gui_log_printf(status, GUI_LOG_TRACE, "AIRPORT_LIST nearest #%d: %s (%s) at %.2f km",
				k + 1, status->facility_lookup_top[k].ident, status->facility_lookup_top[k].region, status->facility_lookup_top[k].distance);
		}
		double min_distance = status->facility_lookup_top[0].distance;
		if (min_distance < 5) {
			char* ident = status->facility_lookup_top[0].ident;
			char* region = status->facility_lookup_top[0].region;
			AIRPORT* apt = facility_lookup_target(status);
			strncpy(apt->icao, ident, sizeof(apt->icao) - 1);
			apt->icao[sizeof(apt->icao) - 1] = '\0';
			strncpy(apt->region, region, sizeof(apt->region) - 1);
			apt->region[sizeof(apt->region) - 1] = '\0';
			gui_log_printf(status, GUI_LOG_TRACE, "Requesting facility data for %s (%s) into %s slot",
				apt->icao, apt->region, facility_lookup_target_label(status, apt));
			// The definition's fields are server-side, per-connection state -- only
			// need to be registered once per connection, not once per lookup (see
			// facility_definition_runways_added in types.h).
			if (!status->facility_definition_runways_added) {
				status->facility_definition_runways_added = TRUE;
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN AIRPORT");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "NAME64");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "MAGVAR");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "N_RUNWAYS");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN RUNWAY");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LENGTH");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "WIDTH");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "HEADING");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_NUMBER");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_NUMBER");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_DESIGNATOR");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_DESIGNATOR");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LATITUDE");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LONGITUDE");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE RUNWAY");
				SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE AIRPORT");
			}
			SimConnect_RequestFacilityData_EX1(status->hSimConnect, DEFINITION_RUNWAYS, REQUEST_RUNWAYS, ident, region);
			SimConnect_GetLastSentPacketID(status->hSimConnect, &status->facility_lookup_send_id);
		} else {
			gui_log_printf(status, GUI_LOG_TRACE, "AIRPORT_LIST: nearest airport %.2f km away exceeds 5km threshold; using coordinate-only fallback", min_distance);
			if (status->facility_lookup_is_departure) {
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s, %s at %s",
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					status->data.time_local.format_date_time().c_str());
				status->departure.runway_act.index = -2;
			} else if (status->facility_lookup_is_takeoff) {
				// Found up front, same reasoning as the touchdown branch below: this
				// callback fires asynchronously and can lag well behind the actual
				// takeoff moment. No trips.* update -- a trip has exactly one
				// departure, and this is a touch-and-go marker, not it.
				struct TAKEOFF_DATA* tmp = status->takeoff_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff (touch-and-go) at %s, %s at %s",
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				if (tmp != NULL) {
					tmp->airport.runway_act.distances[0] = -2;
					// trip_takeoffs row already has NULL airport fields from the immediate
					// INSERT at takeoff; no further DB update needed for this path.
					gui_notify_trip_updated(status);
				}
			} else {
				// Found up front (rather than after the log line, as it used to be)
				// so its own recorded touchdown time -- not "now" -- can be logged:
				// this callback fires asynchronously once the facility lookup
				// resolves, which can lag well behind the actual touchdown moment.
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s, %s at %s",
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				db_insert_update_table(status->sql,
					"UPDATE trips SET destination_icao=NULL,destination_rwy=NULL,destination_region=NULL WHERE id=?;",
					NULL,
					status,
					NULL,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->id_trip);
					}
				);
				if (tmp != NULL) {
					tmp->airport.runway_act.distances[0] = -2;
						// trip_touchdowns row already has NULL airport fields from the immediate
					// INSERT at touchdown; no further DB update needed for this path.
					gui_notify_trip_updated(status);
				}
			}
			// Terminal outcome for this lookup -- no facility data request was made,
			// so FACILITY_DATA_END will never fire to clear this.
			status->facility_lookup_pending = FALSE;
			request_next_touchdown_facility_lookup(status);
		}
	}
	break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA:
	{
		// Same staleness guard as AIRPORT_LIST above -- but no need to clear
		// facility_lookup_pending here: FACILITY_DATA_END always follows this
		// (possibly stale) response and clears it there.
		if (status->facility_lookup_trip_id != status->id_trip)
			break;
		SIMCONNECT_RECV_FACILITY_DATA* pWxData = (SIMCONNECT_RECV_FACILITY_DATA*)pData;
		switch (pWxData->Type) {
		case SIMCONNECT_FACILITY_DATA_AIRPORT:
		{
			AIRPORT* tmp = facility_lookup_target(status);
			memcpy(tmp, &pWxData->Data, sizeof(tmp->name) + sizeof(tmp->magvar) + sizeof(tmp->n_runways));
			if (tmp->n_runways < 0) {
				gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_AIRPORT: negative n_runways=%d from sim; treating as 0 runways", tmp->n_runways);
				tmp->n_runways = 0;
			}
			// calloc, not malloc: any slot whose SIMCONNECT_FACILITY_DATA_RUNWAY
			// response never arrives (e.g. n_runways overstates what MSFS actually
			// sends) must read back as zero, not uninitialized heap garbage, since
			// the FACILITY_DATA_END matching loop below iterates all n_runways
			// slots unconditionally.
			tmp->runways = (RUNWAY*)calloc((size_t)tmp->n_runways, sizeof(RUNWAY));
			if (tmp->runways == NULL) {
				gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_AIRPORT: malloc failed for %d runways; treating as 0 runways", tmp->n_runways);
				tmp->n_runways = 0;
			}
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_AIRPORT: %s slot, name=%s, n_runways=%d",
				facility_lookup_target_label(status, tmp), tmp->name, tmp->n_runways);
		}
		break;
		case SIMCONNECT_FACILITY_DATA_RUNWAY:
		{
			AIRPORT* apt = facility_lookup_target(status);
			RUNWAY* rep = apt->runways;
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_RUNWAY: %s slot, ItemIndex=%lu, n_runways=%d",
				facility_lookup_target_label(status, apt), pWxData->ItemIndex, apt->n_runways);
			// apt->n_runways is guaranteed >= 0 (clamped in FACILITY_DATA_AIRPORT
			// above); ItemIndex is unsigned, so comparing it directly against a
			// non-negative n_runways (rather than casting ItemIndex down to a
			// possibly-negative int) can't be bypassed by an out-of-range ItemIndex.
			if (rep == NULL || pWxData->ItemIndex >= (unsigned int)apt->n_runways) {
				gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_RUNWAY: no runways buffer for ItemIndex=%lu; dropping", pWxData->ItemIndex);
				break;
			}
			memset(&rep[pWxData->ItemIndex], 0, sizeof(RUNWAY));
			memcpy((char*)&rep[pWxData->ItemIndex] + sizeof(rep->placeholder), &pWxData->Data, sizeof(RUNWAY) - sizeof(rep->placeholder) - sizeof(rep->start_points));
		}
		break;
		default:
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA_END:
	{
		AIRPORT* rep = facility_lookup_target(status);
		// RAII guard: frees rep->runways and clears facility_lookup_pending no
		// matter how this case block exits -- including a db_exception thrown by
		// one of the db_insert_update_table calls below, which previously unwound
		// straight past the manual cleanup at the bottom of this case (skipping it
		// entirely) to the outer catch in this function, permanently leaking the
		// runways malloc and leaving facility_lookup_pending stuck true.
		struct FacilityLookupCleanup {
			AIRPORT* rep;
			struct STATUS* status;
			~FacilityLookupCleanup() {
				if (rep->runways != NULL) {
					free(rep->runways);
					rep->runways = NULL;
				}
				status->facility_lookup_pending = FALSE;
				// Pick up a touchdown that landed while this lookup was still in
				// flight and had its own request skipped -- see
				// request_next_touchdown_facility_lookup() above. A no-op if the
				// trip has ended (touchdown_data is freed) or there's nothing queued.
				request_next_touchdown_facility_lookup(status);
			}
		} cleanup_guard{ rep, status };
		// Drop a response for a lookup issued by a trip that has since ended --
		// see the identical check in SIMCONNECT_RECV_ID_AIRPORT_LIST above. rep
		// may already belong to a newly-started trip's (freshly cleared) departure/
		// destination slot at this point, so nothing below may touch it.
		if (status->facility_lookup_trip_id != status->id_trip) {
			gui_log_printf(status, GUI_LOG_TRACE, "Dropping stale facility lookup response for trip %d (current trip %d)",
				status->facility_lookup_trip_id, status->id_trip);
			break;
		}
		gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_END: %s slot, icao=%s, n_runways=%d",
			facility_lookup_target_label(status, rep), rep->icao, rep->n_runways);
		double bearing_tra = (double)status->facility_lookup_heading - rep->magvar;
		if (bearing_tra <= 0)
			bearing_tra += 360;
		// Ground-track refinement only applies to a touchdown/destination
		// lookup: the aircraft can still be crabbed into wind right up to the
		// moment of touchdown, so the bearing from that touchdown's own frozen
		// final-approach loc_dh snapshot (see TOUCHDOWN_DATA::loc_dh in
		// types.h) to the touchdown point is a better estimate of the
		// direction of travel than instantaneous heading. Departure/takeoff
		// have no such crossing available beforehand -- the aircraft is still
		// on the ground before liftoff, so the only 50-100ft AGL crossing it
		// could ever have is during climb-out, *after* the event -- and using
		// that would describe the reverse of the actual departure direction.
		// The aircraft is also mechanically tracking the runway during the
		// ground roll (no crab yet), so its own heading is already correct
		// for departure/takeoff; it's used unrefined for both.
		bool is_touchdown = (rep != &status->departure) && !status->facility_lookup_is_takeoff;
		COORDINATE* loc_dh_source = &status->loc_dh;
		if (is_touchdown) {
			struct TOUCHDOWN_DATA* pending = status->touchdown_data;
			while (pending != NULL && pending->airport.runway_act.distances[0] != -1)
				pending = pending->next;
			if (pending != NULL)
				loc_dh_source = &pending->loc_dh;
			if (loc_dh_source->latitude != 360)
				bearing_tra = loc_dh_source->bearing2Coordinate(status->facility_lookup_coordinate);
		}
		gui_log_printf(status, GUI_LOG_TRACE, "Runway match: bearing_tra=%.1f (%s), evaluating %d runway(s) for %s slot",
			bearing_tra, (is_touchdown && loc_dh_source->latitude != 360) ? "loc_dh-based" : "heading-based",
			rep->n_runways, facility_lookup_target_label(status, rep));
		std::vector<struct RUNWAY_OPERATION> candidates;
		for (int i = 0; i < rep->n_runways; i++) {
			RUNWAY* rwy = &rep->runways[i];
			// Human-readable "06L/24R"-style id for trace output -- numbers[]/
			// designators[] alone (e.g. 6/24) don't carry the leading zero or
			// the L/R/C side letter that a pilot would recognize.
			std::string rwy_id = rwy->runway_code_generator(true) + "/" + rwy->runway_code_generator(false);
			double heading = rwy->heading;
			rwy->start_points[1] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);
			heading -= 180;
			if (heading <= 0)
				heading += 360;
			rwy->start_points[0] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);

			double angle = atan(rwy->width / 2 / rwy->length) / V_PI * 180;
			double bearing = rwy->start_points[0].bearing2Coordinate(status->facility_lookup_coordinate);
			double distance = rwy->start_points[0].distanceInKm2Coordinate(status->facility_lookup_coordinate) * 1000;
			double diff_bearing = abs(bearing - rwy->heading);
			if (diff_bearing > 180)
				diff_bearing = 360 - diff_bearing;
			double distance2 = 0;
			if (diff_bearing >= 0 && diff_bearing <= angle)
				distance2 = rwy->length / cos(diff_bearing / 180 * V_PI);
			else if (diff_bearing > angle && diff_bearing <= 90)
				distance2 = rwy->width / 2 / sin(diff_bearing / 180 * V_PI);

			gui_log_printf(status, GUI_LOG_TRACE, "Runway candidate %d/%d: %s (heading=%.1f, len=%.0f, width=%.0f): distance=%.1fm, distance2=%.1fm -> %s",
				i + 1, rep->n_runways, rwy_id.c_str(), rwy->heading, rwy->length, rwy->width,
				distance, distance2, (distance <= distance2) ? "pass" : "fail (outside runway footprint)");

			if (distance <= distance2) {
				RUNWAY_OPERATION candidate;
				candidate.index = i;
				diff_bearing = abs(bearing_tra - rwy->heading);
				if (diff_bearing > 180)
					diff_bearing = 360 - diff_bearing;
				candidate.is_primary = diff_bearing < 90;

				heading = rwy->heading;
				int index = 0;
				if (!candidate.is_primary) {
					heading -= 180;
					if (heading <= 0)
						heading += 360;
					index = 1;
				}

				candidate.diff_bearing_tra = abs(bearing_tra - heading);
				if (candidate.diff_bearing_tra > 180)
					candidate.diff_bearing_tra = 360 - candidate.diff_bearing_tra;

				int dir = -1;
				double tmp_heading = heading + 90;
				if (tmp_heading > 360)
					tmp_heading -= 360;
				COORDINATE loc = rwy->start_points[index].intersectionCoordinate(heading, status->facility_lookup_coordinate, tmp_heading);
				if (loc.latitude == 360) {
					dir = 1;
					tmp_heading = heading - 90;
					if (tmp_heading <= 0)
						tmp_heading += 360;
					loc = rwy->start_points[index].intersectionCoordinate(heading, status->facility_lookup_coordinate, tmp_heading);
				}
				// Both attempts failed (parallel/coincident great circles) -- loc is
				// still the (360,360) invalid sentinel. Skip this runway rather than
				// computing a distance against it, which would silently write a
				// nonsensical distance_length/distance_width for this touchdown.
				if (loc.latitude == 360) {
					gui_log_printf(status, GUI_LOG_TRACE, "Runway candidate %d/%d: %s passed footprint check but intersection calc failed (parallel/coincident bearings); skipping",
						i + 1, rep->n_runways, rwy_id.c_str());
					continue;
				}
				candidate.distances[0] = loc.distanceInKm2Coordinate(rwy->start_points[index]) * 1000 * M_2_FT;
				candidate.distances[1] = loc.distanceInKm2Coordinate(status->facility_lookup_coordinate) * dir * 1000 * M_2_FT;
				candidate.distances_percent[0] = candidate.distances[0] / rwy->length / M_2_FT;
				candidate.distances_percent[1] = candidate.distances[1] / rwy->width * 2 / M_2_FT;

				gui_log_printf(status, GUI_LOG_TRACE, "Runway candidate %d/%d: %s accepted, is_primary=%d, diff_bearing_tra=%.1f",
					i + 1, rep->n_runways, rwy_id.c_str(), candidate.is_primary ? 1 : 0, candidate.diff_bearing_tra);
				candidates.push_back(candidate);
			}
		}
		gui_log_printf(status, GUI_LOG_TRACE, "Runway match: %zu candidate(s) for %s slot",
			candidates.size(), facility_lookup_target_label(status, rep));
		if (candidates.size() > 0) {
			auto it = std::min_element(
				candidates.begin(),
				candidates.end(),
				[](struct RUNWAY_OPERATION& rwy1, struct RUNWAY_OPERATION& rwy2) {
					return rwy1.diff_bearing_tra < rwy2.diff_bearing_tra;
				}
			);
			rep->runway_act = *it;
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: selected runway index=%d (diff_bearing_tra=%.1f) for %s slot",
				rep->runway_act.index, rep->runway_act.diff_bearing_tra, facility_lookup_target_label(status, rep));
		}
		if (rep->runway_act.index != -1) {
			std::string strRunway = rep->runway_code_generator();
			if (rep == &status->departure) {
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s (%s) runway %s at %s", rep->name, rep->icao, strRunway.c_str(), status->data.time_local.format_date_time().c_str());
				db_insert_update_table(status->sql,
					"UPDATE trips SET departure_icao=?,departure_rwy=?,departure_region=?,departure_name=? WHERE id=?;",
					NULL,
					status,
					(char*)strRunway.c_str(),
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->departure.icao);
						db_bind(stmt, stmt_txt, 2, (char*)aux);
						db_bind(stmt, stmt_txt, 3, status->departure.region);
						db_bind(stmt, stmt_txt, 4, status->departure.name);
						db_bind(stmt, stmt_txt, 5, status->id_trip);
					}
				);
				gui_notify_trip_updated(status);
				if (status->departure_db_id < 0) {
					// The immediate INSERT at takeoff time never got a valid rowid
					// (e.g. it hit SQLITE_BUSY and threw) -- "WHERE id=?" with an
					// invalid id would just match zero rows and silently drop this
					// resolution, so skip it and say why instead.
					gui_log_printf(status, GUI_LOG_WARNING, "Takeoff from %s (%s) runway %s: trip_takeoffs row was never inserted; dropping this resolution", rep->name, rep->icao, strRunway.c_str());
				} else {
					db_insert_update_table(status->sql,
						"UPDATE trip_takeoffs SET icao=?,airport_name=?,runway=?,"
						"distance_length=?,distance_width=?,distance_length_percent=?,distance_width_percent=?"
						" WHERE id=?;",
						rep, status,
						(char*)strRunway.c_str(),
						[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
							struct AIRPORT* pS = (struct AIRPORT*)data;
							db_bind(stmt, stmt_txt, 1, pS->icao);
							db_bind(stmt, stmt_txt, 2, pS->name);
							db_bind(stmt, stmt_txt, 3, (char*)aux);
							db_bind(stmt, stmt_txt, 4, pS->runway_act.distances[0] < 0 ? -1.0 : pS->runway_act.distances[0]);
							db_bind(stmt, stmt_txt, 5, pS->runway_act.distances[1]);
							db_bind(stmt, stmt_txt, 6, pS->runway_act.distances_percent[0]);
							db_bind(stmt, stmt_txt, 7, pS->runway_act.distances_percent[1]);
							db_bind(stmt, stmt_txt, 8, status->departure_db_id);
						}
					);
				}
			} else if (status->facility_lookup_is_takeoff) {
				// Mirrors the touchdown branch below, but against takeoff_data and
				// trip_takeoffs, with NO trips.* update -- a trip has exactly one
				// departure, and this is a touch-and-go marker, not it.
				struct TAKEOFF_DATA* tmp = status->takeoff_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff (touch-and-go) from %s (%s) runway %s at %s", rep->name, rep->icao, strRunway.c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				if (tmp != NULL) {
					tmp->airport.copy(rep);
					if (tmp->db_id < 0) {
						// See the identical guard in the departure branch above: an
						// invalid db_id means the immediate INSERT never completed, so
						// there is no row for "WHERE id=?" to match -- skip it rather
						// than silently no-op.
						gui_log_printf(status, GUI_LOG_WARNING, "Takeoff (touch-and-go) from %s (%s) runway %s: trip_takeoffs row was never inserted; dropping this resolution", rep->name, rep->icao, strRunway.c_str());
					} else {
						db_insert_update_table(status->sql,
							"UPDATE trip_takeoffs SET icao=?,airport_name=?,runway=?,"
							"distance_length=?,distance_width=?,distance_length_percent=?,distance_width_percent=?"
							" WHERE id=?;",
							tmp, status,
							(char*)strRunway.c_str(),
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TAKEOFF_DATA* pS = (struct TAKEOFF_DATA*)data;
								db_bind(stmt, stmt_txt, 1, pS->airport.icao);
								db_bind(stmt, stmt_txt, 2, pS->airport.name);
								db_bind(stmt, stmt_txt, 3, (char*)aux);
								db_bind(stmt, stmt_txt, 4, pS->airport.runway_act.distances[0] < 0 ? -1.0 : pS->airport.runway_act.distances[0]);
								db_bind(stmt, stmt_txt, 5, pS->airport.runway_act.distances[1]);
								db_bind(stmt, stmt_txt, 6, pS->airport.runway_act.distances_percent[0]);
								db_bind(stmt, stmt_txt, 7, pS->airport.runway_act.distances_percent[1]);
								db_bind(stmt, stmt_txt, 8, pS->db_id);
							}
						);
					}
					gui_notify_trip_updated(status);
				}
				// FACILITY_DATA_AIRPORT does not carry icao -- only AIRPORT_LIST does.
				// Same preservation-across-clear() reasoning as the touchdown branch
				// below, for the takeoff_scratch scratch object instead of destination.
				{
					TAKEOFF_DATA* next = status->takeoff_data;
					while (next && next->airport.runway_act.distances[0] != -1)
						next = next->next;
					char saved_icao[sizeof(rep->icao)];
					char saved_region[sizeof(rep->region)];
					bool has_more = (next != nullptr);
					if (has_more) {
						memcpy(saved_icao, rep->icao, sizeof(saved_icao));
						memcpy(saved_region, rep->region, sizeof(saved_region));
					}
					rep->clear();
					if (has_more) {
						memcpy(rep->icao, saved_icao, sizeof(rep->icao));
						memcpy(rep->region, saved_region, sizeof(rep->region));
					}
				}
			} else {
				// Found up front (rather than after the log line, as it used to be)
				// so its own recorded touchdown time -- not "now" -- can be logged:
				// this callback fires asynchronously once the facility lookup
				// resolves, which can lag well behind the actual touchdown moment.
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s (%s) runway %s at %s", rep->name, rep->icao, strRunway.c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				db_insert_update_table(status->sql,
					"UPDATE trips SET destination_icao=?,destination_rwy=?,destination_region=?,destination_name=? WHERE id=?;",
					NULL,
					status,
					(char*)strRunway.c_str(),
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->destination.icao);
						db_bind(stmt, stmt_txt, 2, (char*)aux);
						db_bind(stmt, stmt_txt, 3, status->destination.region);
						db_bind(stmt, stmt_txt, 4, status->destination.name);
						db_bind(stmt, stmt_txt, 5, status->id_trip);
					}
				);
				if (tmp != NULL) {
					tmp->airport.copy(rep);
					if (tmp->db_id < 0) {
						// The immediate INSERT at touchdown time never got a valid
						// rowid (e.g. it hit SQLITE_BUSY and threw) -- "WHERE id=?"
						// with an invalid id would just match zero rows and silently
						// drop this resolution, so skip it and say why instead.
						gui_log_printf(status, GUI_LOG_WARNING, "Touchdown at %s (%s) runway %s: trip_touchdowns row was never inserted; dropping this resolution", rep->name, rep->icao, strRunway.c_str());
					} else {
						db_insert_update_table(status->sql,
							"UPDATE trip_touchdowns SET icao=?,airport_name=?,runway=?,"
							"distance_length=?,distance_width=?,distance_length_percent=?,distance_width_percent=?"
							" WHERE id=?;",
							tmp, status,
							(char*)strRunway.c_str(),
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TOUCHDOWN_DATA* pS = (struct TOUCHDOWN_DATA*)data;
								db_bind(stmt, stmt_txt, 1, pS->airport.icao);
								db_bind(stmt, stmt_txt, 2, pS->airport.name);
								db_bind(stmt, stmt_txt, 3, (char*)aux);
								db_bind(stmt, stmt_txt, 4, pS->airport.runway_act.distances[0] < 0 ? -1.0 : pS->airport.runway_act.distances[0]);
								db_bind(stmt, stmt_txt, 5, pS->airport.runway_act.distances[1]);
								db_bind(stmt, stmt_txt, 6, pS->airport.runway_act.distances_percent[0]);
								db_bind(stmt, stmt_txt, 7, pS->airport.runway_act.distances_percent[1]);
								db_bind(stmt, stmt_txt, 8, pS->db_id);
							}
						);
					}
					gui_notify_trip_updated(status);
				}
				// FACILITY_DATA_AIRPORT does not carry icao — only AIRPORT_LIST does.
				// If another touchdown's AIRPORT_LIST already fired but its FACILITY_DATA
				// callbacks haven't completed yet, rep->clear() would zero icao before
				// that touchdown's FACILITY_DATA_END runs copy(rep). Preserve it.
				{
					TOUCHDOWN_DATA* next = status->touchdown_data;
					while (next && next->airport.runway_act.distances[0] != -1)
						next = next->next;
					char saved_icao[sizeof(rep->icao)];
					char saved_region[sizeof(rep->region)];
					bool has_more = (next != nullptr);
					if (has_more) {
						memcpy(saved_icao, rep->icao, sizeof(saved_icao));
						memcpy(saved_region, rep->region, sizeof(saved_region));
					}
					rep->clear();
					if (has_more) {
						memcpy(rep->icao, saved_icao, sizeof(rep->icao));
						memcpy(rep->region, saved_region, sizeof(rep->region));
					}
				}
			}
		} else {
			if (rep == &status->departure) {
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s (%s) [%s, %s] at %s",
					rep->name,
					rep->icao,
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					status->data.time_local.format_date_time().c_str());
				status->departure.runway_act.index = -2;
				db_insert_update_table(status->sql,
					"UPDATE trips SET departure_icao=?,departure_region=?,departure_name=? WHERE id=?;",
					NULL,
					status,
					NULL,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->departure.icao);
						db_bind(stmt, stmt_txt, 2, status->departure.region);
						db_bind(stmt, stmt_txt, 3, status->departure.name);
						db_bind(stmt, stmt_txt, 4, status->id_trip);
					}
				);
				gui_notify_trip_updated(status);
			} else if (status->facility_lookup_is_takeoff) {
				// Mirrors the touchdown branch below, but against takeoff_data and
				// trip_takeoffs, with NO trips.* update (see the runway-found branch
				// above for why).
				struct TAKEOFF_DATA* tmp = status->takeoff_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff (touch-and-go) from %s (%s) [%s, %s] at %s",
					rep->name,
					rep->icao,
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				if (tmp != NULL) {
					memcpy(tmp->airport.icao, rep->icao, sizeof(rep->icao));
					memcpy(tmp->airport.name, rep->name, sizeof(rep->name));
					tmp->airport.runway_act.distances[0] = -2;
					// Airport found but no matching runway; update trip_takeoffs with
					// the ICAO/name only (runway and distances stay NULL).
					if (tmp->db_id < 0) {
						// See the identical guard above: an invalid db_id means the
						// immediate INSERT never completed, so there is no row for
						// "WHERE id=?" to match -- skip it rather than silently no-op.
						gui_log_printf(status, GUI_LOG_WARNING, "Takeoff (touch-and-go) from %s (%s): trip_takeoffs row was never inserted; dropping this resolution", rep->name, rep->icao);
					} else {
						db_insert_update_table(status->sql,
							"UPDATE trip_takeoffs SET icao=?,airport_name=? WHERE id=?;",
							tmp, status, NULL,
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TAKEOFF_DATA* pS = (struct TAKEOFF_DATA*)data;
								db_bind(stmt, stmt_txt, 1, pS->airport.icao);
								db_bind(stmt, stmt_txt, 2, pS->airport.name);
								db_bind(stmt, stmt_txt, 3, pS->db_id);
							}
						);
					}
					gui_notify_trip_updated(status);
				}
			} else {
				// Found up front (rather than after the log line, as it used to be)
				// so its own recorded touchdown time -- not "now" -- can be logged:
				// this callback fires asynchronously once the facility lookup
				// resolves, which can lag well behind the actual touchdown moment.
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s (%s) [%s, %s] at %s",
					rep->name,
					rep->icao,
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->facility_lookup_coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					tmp != NULL ? tmp->flight_data.time_local.format_date_time().c_str() : "unknown time");
				db_insert_update_table(status->sql,
					"UPDATE trips SET destination_icao=?,destination_region=?,destination_name=?,destination_rwy=NULL WHERE id=?;",
					NULL,
					status,
					NULL,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->destination.icao);
						db_bind(stmt, stmt_txt, 2, status->destination.region);
						db_bind(stmt, stmt_txt, 3, status->destination.name);
						db_bind(stmt, stmt_txt, 4, status->id_trip);
					}
				);
				if (tmp != NULL) {
					memcpy(tmp->airport.icao, rep->icao, sizeof(rep->icao));
					memcpy(tmp->airport.name, rep->name, sizeof(rep->name));
					tmp->airport.runway_act.distances[0] = -2;
					// Airport found but no matching runway; update trip_touchdowns with
					// the ICAO/name only (runway and distances stay NULL).
					if (tmp->db_id < 0) {
						// See the identical guard above: an invalid db_id means the
						// immediate INSERT never completed, so there is no row for
						// "WHERE id=?" to match -- skip it rather than silently no-op.
						gui_log_printf(status, GUI_LOG_WARNING, "Touchdown at %s (%s): trip_touchdowns row was never inserted; dropping this resolution", rep->name, rep->icao);
					} else {
						db_insert_update_table(status->sql,
							"UPDATE trip_touchdowns SET icao=?,airport_name=? WHERE id=?;",
							tmp, status, NULL,
							[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
								struct TOUCHDOWN_DATA* pS = (struct TOUCHDOWN_DATA*)data;
								db_bind(stmt, stmt_txt, 1, pS->airport.icao);
								db_bind(stmt, stmt_txt, 2, pS->airport.name);
								db_bind(stmt, stmt_txt, 3, pS->db_id);
							}
						);
					}
					gui_notify_trip_updated(status);
				}
			}
		}
		// runways free + facility_lookup_pending reset happen in cleanup_guard's
		// destructor above, regardless of which branch was taken. status->loc_dh
		// is deliberately left untouched here -- it's reset by three things only:
		// a fresh 50-100ft AGL crossing (overwrites with new data), climbing back
		// above 100ft (clears to sentinel -- see the SIMOBJECT_DATA handler above),
		// or trip start. This lets repeated touchdowns from the same low bounce/
		// touch-and-go sequence (which never climbs above 100ft) keep reusing the
		// one real approach ground track instead of falling back to heading-based.
		// Safe to leave unreset here: a genuinely distinct later landing climbing
		// above 100ft is a real-world flying assumption, not something this code
		// enforces -- status->airborne (set purely from sim_on_ground, with no
		// altitude term) can't tell a low bounce apart from a full circuit. A
		// genuine later landing either gets fresh data on the way back down, or
		// -- if that descent doesn't happen to resample the 50-100ft band --
		// finds loc_dh already cleared to the sentinel and falls back to
		// heading-based bearing, so a stale cross-approach position can never
		// reach a later touchdown either way.
	}
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
		// request (matched by SendID -- see facility_lookup_send_id in types.h), the
		// lookup will never receive its normal terminal response (the AIRPORT_LIST
		// no-match branch or FACILITY_DATA_END), so without this facility_lookup_pending
		// would stay stuck true forever, silently disabling all future departure/
		// destination airport-runway resolution for the rest of the app session.
		if (status->facility_lookup_pending && except->dwSendID == status->facility_lookup_send_id) {
			if (status->facility_lookup_trip_id == status->id_trip) {
				AIRPORT* rep = facility_lookup_target(status);
				if (rep == &status->departure) {
					rep->runway_act.index = -2;
				} else if (status->facility_lookup_is_takeoff) {
					struct TAKEOFF_DATA* tmp = status->takeoff_data;
					while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
						tmp = tmp->next;
					if (tmp != NULL) {
						tmp->airport.runway_act.distances[0] = -2;
						gui_notify_trip_updated(status);
					}
				} else {
					struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
					while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
						tmp = tmp->next;
					if (tmp != NULL) {
						tmp->airport.runway_act.distances[0] = -2;
						gui_notify_trip_updated(status);
					}
				}
				if (rep->runways != NULL) {
					free(rep->runways);
					rep->runways = NULL;
				}
			}
			status->facility_lookup_pending = FALSE;
			request_next_touchdown_facility_lookup(status);
		}
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

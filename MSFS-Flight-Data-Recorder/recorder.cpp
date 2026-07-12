#include "recorder.h"
#include "db.h"
#include "gui_notify.h"
#include <thread>

static bool is_skipped_event(struct STATUS* status, const char* eventName) {
	return status->skip_events.count(eventName) > 0;
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
	"AUTOPILOT_PANEL_AIRSPEED_SET",
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

void add_client_events(HANDLE hSimConnect) {
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_AIRSPEED_HOLD, "AP_AIRSPEED_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_AIRSPEED_OFF, "AP_AIRSPEED_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_AIRSPEED_ON, "AP_AIRSPEED_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_ALT_HOLD, "AP_ALT_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_ALT_HOLD_OFF, "AP_ALT_HOLD_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_ALT_HOLD_ON, "AP_ALT_HOLD_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_APR_HOLD, "AP_APR_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_APR_HOLD_OFF, "AP_APR_HOLD_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_APR_HOLD_ON, "AP_APR_HOLD_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_HDG_HOLD, "AP_HDG_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_HDG_HOLD_OFF, "AP_HDG_HOLD_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_HDG_HOLD_ON, "AP_HDG_HOLD_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_MACH_HOLD, "AP_MACH_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_MACH_OFF, "AP_MACH_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_MACH_ON, "AP_MACH_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_MASTER, "AP_MASTER");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_ALTITUDE_HOLD, "AP_PANEL_ALTITUDE_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_ALTITUDE_OFF, "AP_PANEL_ALTITUDE_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_ALTITUDE_ON, "AP_PANEL_ALTITUDE_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_HEADING_HOLD, "AP_PANEL_HEADING_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_HEADING_OFF, "AP_PANEL_HEADING_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_HEADING_ON, "AP_PANEL_HEADING_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_MACH_HOLD, "AP_PANEL_MACH_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_MACH_OFF, "AP_PANEL_MACH_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_MACH_ON, "AP_PANEL_MACH_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD, "AP_PANEL_SPEED_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_SPEED_OFF, "AP_PANEL_SPEED_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_SPEED_ON, "AP_PANEL_SPEED_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_VS_OFF, "AP_PANEL_VS_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_VS_ON, "AP_PANEL_VS_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_VS_HOLD, "AP_PANEL_VS_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_VS_HOLD, "AP_VS_HOLD");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_VS_OFF, "AP_VS_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_VS_ON, "AP_VS_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_SPEED_HOLD_TOGGLE, "AP_PANEL_SPEED_HOLD_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AP_PANEL_MACH_HOLD_TOGGLE, "AP_PANEL_MACH_HOLD_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOPILOT_DISENGAGE_TOGGLE, "AUTOPILOT_DISENGAGE_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOPILOT_OFF, "AUTOPILOT_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOPILOT_ON, "AUTOPILOT_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOPILOT_PANEL_AIRSPEED_SET, "AUTOPILOT_PANEL_AIRSPEED_SET");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE, "FLIGHT_LEVEL_CHANGE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_OFF, "FLIGHT_LEVEL_CHANGE_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLIGHT_LEVEL_CHANGE_ON, "FLIGHT_LEVEL_CHANGE_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTO_THROTTLE_ARM, "AUTO_THROTTLE_ARM");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTO_THROTTLE_TO_GA, "AUTO_THROTTLE_TO_GA");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOBRAKE_DISARM, "AUTOBRAKE_DISARM");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOBRAKE_HI_SET, "AUTOBRAKE_HI_SET");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOBRAKE_LO_SET, "AUTOBRAKE_LO_SET");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_AUTOBRAKE_MED_SET, "AUTOBRAKE_MED_SET");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GPWS_SWITCH_TOGGLE, "GPWS_SWITCH_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_TOGGLE_FLIGHT_DIRECTOR, "TOGGLE_FLIGHT_DIRECTOR");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_APU_BLEED_AIR_SOURCE_TOGGLE, "APU_BLEED_AIR_SOURCE_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_APU_GENERATOR_SWITCH_TOGGLE, "APU_GENERATOR_SWITCH_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_APU_OFF_SWITCH, "APU_OFF_SWITCH");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_APU_STARTER, "APU_STARTER");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ANTI_ICE_ON, "ANTI_ICE_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ANTI_ICE_OFF, "ANTI_ICE_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ANTI_ICE_TOGGLE, "ANTI_ICE_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG1, "ANTI_ICE_TOGGLE_ENG1");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_ANTI_ICE_TOGGLE_ENG2, "ANTI_ICE_TOGGLE_ENG2");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_THROTTLE_REVERSE_THRUST_TOGGLE, "THROTTLE_REVERSE_THRUST_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLAPS_DECR, "FLAPS_DECR");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLAPS_DOWN, "FLAPS_DOWN");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLAPS_INCR, "FLAPS_INCR");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_FLAPS_UP, "FLAPS_UP");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_ARM_OFF, "SPOILERS_ARM_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_ARM_ON, "SPOILERS_ARM_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_ARM_TOGGLE, "SPOILERS_ARM_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_OFF, "SPOILERS_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_ON, "SPOILERS_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_SPOILERS_TOGGLE, "SPOILERS_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_CROSS_FEED_TOGGLE, "CROSS_FEED_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_BRAKES, "BRAKES");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_DOWN, "GEAR_DOWN");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE, "GEAR_EMERGENCY_HANDLE_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_TOGGLE, "GEAR_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_GEAR_UP, "GEAR_UP");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_PARKING_BRAKES, "PARKING_BRAKES");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE, "CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE, "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_WINDSHIELD_DEICE_OFF, "WINDSHIELD_DEICE_OFF");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_WINDSHIELD_DEICE_ON, "WINDSHIELD_DEICE_ON");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_WINDSHIELD_DEICE_TOGGLE, "WINDSHIELD_DEICE_TOGGLE");
	SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_TOGGLE_AVIONICS_MASTER, "TOGGLE_AVIONICS_MASTER");

	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_AIRSPEED_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_AIRSPEED_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_AIRSPEED_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_ALT_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_ALT_HOLD_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_ALT_HOLD_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_APR_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_APR_HOLD_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_APR_HOLD_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_HDG_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_HDG_HOLD_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_HDG_HOLD_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_MACH_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_MACH_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_MACH_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_MASTER);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_ALTITUDE_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_ALTITUDE_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_ALTITUDE_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_HEADING_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_HEADING_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_HEADING_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_MACH_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_MACH_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_MACH_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_SPEED_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_SPEED_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_SPEED_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_VS_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_VS_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_VS_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_VS_HOLD);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_VS_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_VS_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_SPEED_HOLD_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AP_PANEL_MACH_HOLD_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOPILOT_DISENGAGE_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOPILOT_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOPILOT_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOPILOT_PANEL_AIRSPEED_SET);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLIGHT_LEVEL_CHANGE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLIGHT_LEVEL_CHANGE_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLIGHT_LEVEL_CHANGE_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTO_THROTTLE_ARM);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTO_THROTTLE_TO_GA);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOBRAKE_DISARM);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOBRAKE_HI_SET);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOBRAKE_LO_SET);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_AUTOBRAKE_MED_SET);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_GPWS_SWITCH_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_TOGGLE_FLIGHT_DIRECTOR);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_APU_BLEED_AIR_SOURCE_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_APU_GENERATOR_SWITCH_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_APU_OFF_SWITCH);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_APU_STARTER);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_ANTI_ICE_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_ANTI_ICE_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_ANTI_ICE_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_ANTI_ICE_TOGGLE_ENG1);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_ANTI_ICE_TOGGLE_ENG2);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_THROTTLE_REVERSE_THRUST_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLAPS_DECR);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLAPS_DOWN);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLAPS_INCR);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_FLAPS_UP);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_ARM_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_ARM_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_ARM_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_SPOILERS_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_CROSS_FEED_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_BRAKES);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_GEAR_DOWN);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_GEAR_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_GEAR_UP);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_PARKING_BRAKES);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_WINDSHIELD_DEICE_OFF);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_WINDSHIELD_DEICE_ON);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_WINDSHIELD_DEICE_TOGGLE);
	SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_1, EVENT_TOGGLE_AVIONICS_MASTER);
}

void stop_recording(struct STATUS* status) {
	status->recording = FALSE;
	// Destination lat/lon was written at each touchdown; only the arrival time
	// (engine shutdown) is set here — consistent with departure time being engine start.
	db_insert_update_table(
		status->sql,
		"UPDATE trips SET destination_zulu_time=?,destination_local_time=? WHERE id=?;",
		status->q_data_end == NULL ? status->q_data_last : status->q_data_end,
		status,
		NULL,
		[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
			struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
			db_bind(stmt, stmt_txt, 1, pS->time_zulu.format_date_time().c_str());
			db_bind(stmt, stmt_txt, 2, pS->time_local.format_date_time().c_str());
			db_bind(stmt, stmt_txt, 3, status->id_trip);
		}
	);
	// trip_touchdowns rows were already inserted at touchdown time; just free the list.
	while (status->touchdown_data != NULL) {
		struct TOUCHDOWN_DATA* cur = status->touchdown_data;
		status->touchdown_data = status->touchdown_data->next;
		cur->airport.clear();
		free(cur);
	}
	status->touchdown_data_end = NULL;
	int ended_trip_id = status->id_trip;
	std::thread([status, ended_trip_id]() {
		db_consume(status, ended_trip_id, true);
		if (status->q_data_last != NULL) {
			free(status->q_data_last);
			status->q_data_last = NULL;
		}
		status->id_trip = -1;
		gui_log_printf(status, GUI_LOG_INFO, "Recording stopped\n");
		gui_notify_recording_changed(status, false, ended_trip_id);
	}).detach();
}

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
	struct STATUS* status = (struct STATUS*)pContext;
	try {
	switch (pData->dwID) {
	case SIMCONNECT_RECV_ID_OPEN:
		gui_log_printf(status, GUI_LOG_INFO, "Connected to Microsoft Flight Simulator\n");
		gui_notify_connection_changed(status, true);
		break;
	case SIMCONNECT_RECV_ID_QUIT:
		gui_log_printf(status, GUI_LOG_INFO, "Disconnected from Microsoft Flight Simulator\n");
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
			gui_log_printf(status, GUI_LOG_WARNING, "Plane crashed!\n");
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
				gui_log_printf(status, GUI_LOG_INFO, "Event: %s\n", EVENT_ID_TXT[evt->uEventID]);
				std::string tz = status->data.time_zulu.format_date_time();
				std::string tl = status->data.time_local.format_date_time();
				db_insert_event(status, EVENT_ID_TXT[evt->uEventID], tz.c_str(), tl.c_str());
			}
			break;
		default:
			gui_log_printf(status, GUI_LOG_WARNING, "Unknown event ID: %ld\n", evt->uEventID);
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
			if (tmp.radio_height > 50 && tmp.radio_height < 100) {
				status->loc_dh.latitude = tmp.plane_coordinate.latitude;
				status->loc_dh.longitude = tmp.plane_coordinate.longitude;
			}
			if (status->sim_running && !status->paused && tmp.surface_type != 255) {
				status->in_sim = TRUE;
				if ((bool)tmp.sim_on_ground) {
					if ((bool)tmp.eng_combustion_1 || (bool)tmp.eng_combustion_2) {
						if (!status->recording) {
							status->recording = TRUE;
							gui_log_printf(status, GUI_LOG_INFO, "Recording started\n");

							status->q_data_db_start  = NULL;
							status->q_data_db_end    = NULL;
							status->q_data_end       = NULL;
							status->q_data_db_length = 0;
							if (status->q_data_last != NULL) {
								free(status->q_data_last);
								status->q_data_last = NULL;
							}

							status->departure.clear();
							status->destination.clear();
							status->airborne = !(bool)tmp.sim_on_ground;

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
								}
							);
							db_query_table(
								status->sql,
								"SELECT id FROM trips ORDER BY id DESC LIMIT 1;",
								&tmp,
								status,
								NULL,
								NULL,
								NULL,
								[](sqlite3_stmt* stmt, const char* stmt_txt, struct STATUS* status, void* aux) {
									status->id_trip = sqlite3_column_int(stmt, 0);
								}
							);
							gui_notify_recording_changed(status, true, status->id_trip);
						}
					} else {
						if (status->recording)
							stop_recording(status);
					}
				}
			}
			if (status->recording && !status->paused) {
				// Takeoff
				if (!(bool)tmp.sim_on_ground && !status->airborne && status->departure.runway_act.index == -1)
					SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
				// Landing
				if ((bool)tmp.sim_on_ground && status->airborne) {
					struct TOUCHDOWN_DATA* tmp_touchdown = (struct TOUCHDOWN_DATA*)malloc(sizeof(struct TOUCHDOWN_DATA));
					memset(tmp_touchdown, 0, sizeof(struct TOUCHDOWN_DATA));
					tmp_touchdown->airport.clear();
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
					SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
				}
				status->airborne = !(bool)tmp.sim_on_ground;

				double delta_s = status->sample_interval_ms / 1000.0;
				if (status->q_data_end != NULL)
					delta_s = tmp.time_zulu.time_day - status->q_data_end->time_zulu.time_day;
				else if (status->q_data_last != NULL)
					delta_s = tmp.time_zulu.time_day - status->q_data_last->time_zulu.time_day;
				if (delta_s < 0)
					delta_s += 86400;
				if (delta_s >= status->sample_interval_ms / 1000.0) {
					struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)malloc(sizeof(struct FLIGHT_DATA_RECORD));
					memset(pS, 0, sizeof(struct FLIGHT_DATA_RECORD));
					memcpy(pS, &tmp, sizeof(struct FLIGHT_DATA_RECORD));
					gui_notify_sample(status, pS);
					if (status->q_data_end == NULL)
						status->q_data_end = pS;
					else {
						status->q_data_end->next = pS;
						status->q_data_end = pS;
					}
					status->q_data_db_length++;
					if (status->q_data_db_start == NULL)
						status->q_data_db_start = pS;
					if (status->q_data_db_length == Q_DB_LENGTH) {
						status->q_data_db_end = pS;
						status->q_data_db_length = 0;
						int batch_trip_id = status->id_trip;
						std::thread thd([status, batch_trip_id]() { db_consume(status, batch_trip_id); });
						thd.detach();
					}
				}
			}
		}
		break;
		default:
			gui_log_printf(status, GUI_LOG_PROFILE, "SIMCONNECT_RECV_SIMOBJECT_DATA: %d\n", pObjData->dwRequestID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_AIRPORT_LIST:
	{
		SIMCONNECT_RECV_AIRPORT_LIST* pWxData = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
		int min_index = -1;
		double min_distance = 100000;
		for (int i = 0; i < (int)pWxData->dwArraySize; i++) {
			struct SIMCONNECT_DATA_FACILITY_AIRPORT airport = pWxData->rgData[i];
			COORDINATE airport_loc;
			airport_loc.latitude = airport.Latitude;
			airport_loc.longitude = airport.Longitude;
			double distance = abs(status->data.coordinate.distanceInKm2Coordinate(airport_loc));
			if (distance <= min_distance) {
				min_distance = distance;
				min_index = i;
			}
		}
		if (min_distance < 5) {
			char* ident = pWxData->rgData[min_index].Ident;
			char* region = pWxData->rgData[min_index].Region;
			AIRPORT* apt = (status->departure.runway_act.index == -1) ? &status->departure : &status->destination;
			strncpy(apt->icao, ident, sizeof(apt->icao) - 1);
			apt->icao[sizeof(apt->icao) - 1] = '\0';
			strncpy(apt->region, region, sizeof(apt->region) - 1);
			apt->region[sizeof(apt->region) - 1] = '\0';
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
			SimConnect_RequestFacilityData_EX1(status->hSimConnect, DEFINITION_RUNWAYS, REQUEST_RUNWAYS, ident, region);
		} else {
			if (status->departure.runway_act.index == -1) {
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s, %s at %s\n",
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
					status->data.time_local.format_date_time().c_str());
				status->departure.runway_act.index = -2;
			} else {
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s, %s\n",
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str());
				db_insert_update_table(status->sql,
					"UPDATE trips SET destination_icao=NULL,destination_rwy=NULL,destination_region=NULL WHERE id=?;",
					NULL,
					status,
					NULL,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->id_trip);
					}
				);
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				if (tmp != NULL) {
					tmp->airport.runway_act.distances[0] = -2;
						// trip_touchdowns row already has NULL airport fields from the immediate
					// INSERT at touchdown; no further DB update needed for this path.
					gui_notify_trip_updated(status);
				}
			}
		}
	}
	break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA:
	{
		SIMCONNECT_RECV_FACILITY_DATA* pWxData = (SIMCONNECT_RECV_FACILITY_DATA*)pData;
		switch (pWxData->Type) {
		case SIMCONNECT_FACILITY_DATA_AIRPORT:
		{
			AIRPORT* tmp = (status->departure.runway_act.index == -1) ? &status->departure : &status->destination;
			memcpy(tmp, &pWxData->Data, sizeof(tmp->name) + sizeof(tmp->magvar) + sizeof(tmp->n_runways));
			tmp->runways = (RUNWAY*)malloc(sizeof(RUNWAY) * tmp->n_runways);
		}
		break;
		case SIMCONNECT_FACILITY_DATA_RUNWAY:
		{
			RUNWAY* rep = (status->departure.runway_act.index == -1) ? status->departure.runways : status->destination.runways;
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
		AIRPORT* rep = (status->departure.runway_act.index == -1) ? &status->departure : &status->destination;
		double bearing_tra = (double)status->data.heading - rep->magvar;
		if (bearing_tra <= 0)
			bearing_tra += 360;
		if (status->loc_dh.latitude != 360)
			bearing_tra = status->loc_dh.bearing2Coordinate(status->data.coordinate);
		std::vector<struct RUNWAY_OPERATION> candidates;
		for (int i = 0; i < rep->n_runways; i++) {
			RUNWAY* rwy = &rep->runways[i];
			double heading = rwy->heading;
			rwy->start_points[1] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);
			heading -= 180;
			if (heading <= 0)
				heading += 360;
			rwy->start_points[0] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);

			double angle = atan(rwy->width / 2 / rwy->length) / V_PI * 180;
			double bearing = rwy->start_points[0].bearing2Coordinate(status->data.coordinate);
			double distance = rwy->start_points[0].distanceInKm2Coordinate(status->data.coordinate) * 1000;
			double diff_bearing = abs(bearing - rwy->heading);
			if (diff_bearing > 180)
				diff_bearing = 360 - diff_bearing;
			double distance2 = 0;
			if (diff_bearing >= 0 && diff_bearing <= angle)
				distance2 = rwy->length / cos(diff_bearing / 180 * V_PI);
			else if (diff_bearing > angle && diff_bearing <= 90)
				distance2 = rwy->width / 2 / sin(diff_bearing / 180 * V_PI);

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
				COORDINATE loc = rwy->start_points[index].intersectionCoordinate(heading, status->data.coordinate, tmp_heading);
				if (loc.latitude == 360) {
					dir = 1;
					tmp_heading = heading - 90;
					if (tmp_heading <= 0)
						tmp_heading += 360;
					loc = rwy->start_points[index].intersectionCoordinate(heading, status->data.coordinate, tmp_heading);
				}
				candidate.distances[0] = loc.distanceInKm2Coordinate(rwy->start_points[index]) * 1000 * M_2_FT;
				candidate.distances[1] = loc.distanceInKm2Coordinate(status->data.coordinate) * dir * 1000 * M_2_FT;
				candidate.distances_percent[0] = candidate.distances[0] / rwy->length / M_2_FT;
				candidate.distances_percent[1] = candidate.distances[1] / rwy->width * 2 / M_2_FT;

				candidates.push_back(candidate);
			}
		}
		if (candidates.size() > 0) {
			auto it = std::min_element(
				candidates.begin(),
				candidates.end(),
				[](struct RUNWAY_OPERATION& rwy1, struct RUNWAY_OPERATION& rwy2) {
					return rwy1.diff_bearing_tra < rwy2.diff_bearing_tra;
				}
			);
			rep->runway_act = *it;
		}
		if (rep->runway_act.index != -1) {
			std::string strRunway = rep->runway_code_generator();
			if (rep == &status->departure) {
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s (%s) runway %s at %s\n", rep->name, rep->icao, strRunway.c_str(), status->data.time_local.format_date_time().c_str());
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
			} else {
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s (%s) runway %s\n", rep->name, rep->icao, strRunway.c_str());
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
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				if (tmp != NULL) {
					tmp->airport.copy(rep);
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
				gui_log_printf(status, GUI_LOG_INFO, "Takeoff from %s (%s) [%s, %s] at %s\n",
					rep->name,
					rep->icao,
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str(),
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
			} else {
				gui_log_printf(status, GUI_LOG_INFO, "Touchdown at %s (%s) [%s, %s]\n",
					rep->name,
					rep->icao,
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE).c_str(),
					status->data.coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE).c_str());
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
				struct TOUCHDOWN_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && tmp->airport.runway_act.distances[0] != -1)
					tmp = tmp->next;
				if (tmp != NULL) {
					memcpy(tmp->airport.icao, rep->icao, sizeof(rep->icao));
					memcpy(tmp->airport.name, rep->name, sizeof(rep->name));
					tmp->airport.runway_act.distances[0] = -2;
					// Airport found but no matching runway; update trip_touchdowns with
					// the ICAO/name only (runway and distances stay NULL).
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
					gui_notify_trip_updated(status);
				}
			}
		}
		status->loc_dh.clear();
	}
	break;
	case SIMCONNECT_RECV_ID_EXCEPTION:
		// SIMCONNECT_RECV_EXCEPTION* except = (SIMCONNECT_RECV_EXCEPTION*)pData;
		break;
	default:
		gui_log_printf(status, GUI_LOG_PROFILE, "SIMCONNECT_RECV: %d\n", pData->dwID);
		break;
	}
	} catch (const db_exception& e) {
		gui_log_printf(status, GUI_LOG_WARNING, "Database error in dispatch: %s\n", e.message.c_str());
	}
}

#pragma once

#include "types.h"
#include "engine_power.h"
#include "SimConnect.h"

enum GROUP_ID {
	GROUP_1,
};

// Every cockpit event recorded to trip_events, in registration order:
// X(EVENT_ID name without the EVENT_ prefix, SimConnect client event name).
// The client event name is also what trip_events.event stores -- it differs
// from the enum name only for AUTOPILOT_PANEL_AIRSPEED_SET.
#define COCKPIT_EVENTS(X) \
	X(AP_AIRSPEED_HOLD, "AP_AIRSPEED_HOLD") \
	X(AP_AIRSPEED_OFF, "AP_AIRSPEED_OFF") \
	X(AP_AIRSPEED_ON, "AP_AIRSPEED_ON") \
	X(AP_ALT_HOLD, "AP_ALT_HOLD") \
	X(AP_ALT_HOLD_OFF, "AP_ALT_HOLD_OFF") \
	X(AP_ALT_HOLD_ON, "AP_ALT_HOLD_ON") \
	X(AP_APR_HOLD, "AP_APR_HOLD") \
	X(AP_APR_HOLD_OFF, "AP_APR_HOLD_OFF") \
	X(AP_APR_HOLD_ON, "AP_APR_HOLD_ON") \
	X(AP_HDG_HOLD, "AP_HDG_HOLD") \
	X(AP_HDG_HOLD_OFF, "AP_HDG_HOLD_OFF") \
	X(AP_HDG_HOLD_ON, "AP_HDG_HOLD_ON") \
	X(AP_MACH_HOLD, "AP_MACH_HOLD") \
	X(AP_MACH_OFF, "AP_MACH_OFF") \
	X(AP_MACH_ON, "AP_MACH_ON") \
	X(AP_MASTER, "AP_MASTER") \
	X(AP_PANEL_ALTITUDE_HOLD, "AP_PANEL_ALTITUDE_HOLD") \
	X(AP_PANEL_ALTITUDE_OFF, "AP_PANEL_ALTITUDE_OFF") \
	X(AP_PANEL_ALTITUDE_ON, "AP_PANEL_ALTITUDE_ON") \
	X(AP_PANEL_HEADING_HOLD, "AP_PANEL_HEADING_HOLD") \
	X(AP_PANEL_HEADING_OFF, "AP_PANEL_HEADING_OFF") \
	X(AP_PANEL_HEADING_ON, "AP_PANEL_HEADING_ON") \
	X(AP_PANEL_MACH_HOLD, "AP_PANEL_MACH_HOLD") \
	X(AP_PANEL_MACH_OFF, "AP_PANEL_MACH_OFF") \
	X(AP_PANEL_MACH_ON, "AP_PANEL_MACH_ON") \
	X(AP_PANEL_SPEED_HOLD, "AP_PANEL_SPEED_HOLD") \
	X(AP_PANEL_SPEED_OFF, "AP_PANEL_SPEED_OFF") \
	X(AP_PANEL_SPEED_ON, "AP_PANEL_SPEED_ON") \
	X(AP_PANEL_VS_OFF, "AP_PANEL_VS_OFF") \
	X(AP_PANEL_VS_ON, "AP_PANEL_VS_ON") \
	X(AP_PANEL_VS_HOLD, "AP_PANEL_VS_HOLD") \
	X(AP_VS_HOLD, "AP_VS_HOLD") \
	X(AP_VS_OFF, "AP_VS_OFF") \
	X(AP_VS_ON, "AP_VS_ON") \
	X(AP_PANEL_SPEED_HOLD_TOGGLE, "AP_PANEL_SPEED_HOLD_TOGGLE") \
	X(AP_PANEL_MACH_HOLD_TOGGLE, "AP_PANEL_MACH_HOLD_TOGGLE") \
	X(AUTOPILOT_DISENGAGE_TOGGLE, "AUTOPILOT_DISENGAGE_TOGGLE") \
	X(AUTOPILOT_OFF, "AUTOPILOT_OFF") \
	X(AUTOPILOT_ON, "AUTOPILOT_ON") \
	X(AUTOPILOT_PANEL_AIRSPEED_SET, "AP_PANEL_SPEED_SET") \
	X(FLIGHT_LEVEL_CHANGE, "FLIGHT_LEVEL_CHANGE") \
	X(FLIGHT_LEVEL_CHANGE_OFF, "FLIGHT_LEVEL_CHANGE_OFF") \
	X(FLIGHT_LEVEL_CHANGE_ON, "FLIGHT_LEVEL_CHANGE_ON") \
	X(AUTO_THROTTLE_ARM, "AUTO_THROTTLE_ARM") \
	X(AUTO_THROTTLE_TO_GA, "AUTO_THROTTLE_TO_GA") \
	X(AUTOBRAKE_DISARM, "AUTOBRAKE_DISARM") \
	X(AUTOBRAKE_HI_SET, "AUTOBRAKE_HI_SET") \
	X(AUTOBRAKE_LO_SET, "AUTOBRAKE_LO_SET") \
	X(AUTOBRAKE_MED_SET, "AUTOBRAKE_MED_SET") \
	X(GPWS_SWITCH_TOGGLE, "GPWS_SWITCH_TOGGLE") \
	X(TOGGLE_FLIGHT_DIRECTOR, "TOGGLE_FLIGHT_DIRECTOR") \
	X(APU_BLEED_AIR_SOURCE_TOGGLE, "APU_BLEED_AIR_SOURCE_TOGGLE") \
	X(APU_GENERATOR_SWITCH_TOGGLE, "APU_GENERATOR_SWITCH_TOGGLE") \
	X(APU_OFF_SWITCH, "APU_OFF_SWITCH") \
	X(APU_STARTER, "APU_STARTER") \
	X(ANTI_ICE_ON, "ANTI_ICE_ON") \
	X(ANTI_ICE_OFF, "ANTI_ICE_OFF") \
	X(ANTI_ICE_TOGGLE, "ANTI_ICE_TOGGLE") \
	X(ANTI_ICE_TOGGLE_ENG1, "ANTI_ICE_TOGGLE_ENG1") \
	X(ANTI_ICE_TOGGLE_ENG2, "ANTI_ICE_TOGGLE_ENG2") \
	X(THROTTLE_REVERSE_THRUST_TOGGLE, "THROTTLE_REVERSE_THRUST_TOGGLE") \
	X(FLAPS_DECR, "FLAPS_DECR") \
	X(FLAPS_DOWN, "FLAPS_DOWN") \
	X(FLAPS_INCR, "FLAPS_INCR") \
	X(FLAPS_UP, "FLAPS_UP") \
	X(SPOILERS_ARM_OFF, "SPOILERS_ARM_OFF") \
	X(SPOILERS_ARM_ON, "SPOILERS_ARM_ON") \
	X(SPOILERS_ARM_TOGGLE, "SPOILERS_ARM_TOGGLE") \
	X(SPOILERS_OFF, "SPOILERS_OFF") \
	X(SPOILERS_ON, "SPOILERS_ON") \
	X(SPOILERS_TOGGLE, "SPOILERS_TOGGLE") \
	X(CROSS_FEED_TOGGLE, "CROSS_FEED_TOGGLE") \
	X(BRAKES, "BRAKES") \
	X(GEAR_DOWN, "GEAR_DOWN") \
	X(GEAR_EMERGENCY_HANDLE_TOGGLE, "GEAR_EMERGENCY_HANDLE_TOGGLE") \
	X(GEAR_TOGGLE, "GEAR_TOGGLE") \
	X(GEAR_UP, "GEAR_UP") \
	X(PARKING_BRAKES, "PARKING_BRAKES") \
	X(CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE, "CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE") \
	X(CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE, "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE") \
	X(WINDSHIELD_DEICE_OFF, "WINDSHIELD_DEICE_OFF") \
	X(WINDSHIELD_DEICE_ON, "WINDSHIELD_DEICE_ON") \
	X(WINDSHIELD_DEICE_TOGGLE, "WINDSHIELD_DEICE_TOGGLE") \
	X(TOGGLE_AVIONICS_MASTER, "TOGGLE_AVIONICS_MASTER")

enum EVENT_ID {
	// System events (SimConnect_SubscribeToSystemEvent in recorder_bridge.cpp).
	EVENT_SIM,
	EVENT_PAUSE,
	EVENT_CRASHED,
#define COCKPIT_EVENT_ID(id, name) EVENT_##id,
	COCKPIT_EVENTS(COCKPIT_EVENT_ID)
#undef COCKPIT_EVENT_ID
	EVENT_ID_COUNT
};

enum DATA_DEFINE_ID {
	DEFINITION_FLIGHT,
	DEFINITION_RUNWAYS,
};

enum DATA_REQUEST_ID {
	REQUEST_FLIGHT,
	REQUEST_AIRPORTS,
	REQUEST_RUNWAYS,
};

// Every SimVar of the flight data definition, in the order SimConnect sends
// them and decode_flight_sample() copies them into FLIGHT_DATA_RECORD. This
// one list declares the struct's fields and makes add_flight_definition()'s
// registrations, so the two can't drift apart. Field kinds:
//   NUM(member, SimVar, unit): a double.
//   ENGINES(member, SimVar, unit): double[MAX_ENGINES], from "SimVar:1" to
//     "SimVar:MAX_ENGINES".
//   COORD(member, latitude SimVar, longitude SimVar): a COORDINATE, in
//     degrees.
//   STR(member, SimVar, size): char[size], as SIMCONNECT_DATATYPE_STRING<size>.
//   TIME(member, prefix, UTC offset SimVar or nullptr): a DATETIME, from
//     "<prefix> YEAR", "<prefix> MONTH OF YEAR", "<prefix> DAY OF MONTH",
//     "<prefix> DAY OF WEEK", "<prefix> TIME" (seconds) and the offset SimVar
//     (seconds). Without an offset SimVar, timezone_offset isn't sent.
#define FLIGHT_DATA_FIELDS(NUM, ENGINES, COORD, STR, TIME) \
	NUM(autopilot_airspeed_hold, "AUTOPILOT AIRSPEED HOLD", "Bool") \
	NUM(autopilot_airspeed_hold_var, "AUTOPILOT AIRSPEED HOLD VAR", "Knots") \
	NUM(autopilot_alt_radio_mode, "AUTOPILOT ALT RADIO MODE", "Bool") \
	NUM(autopilot_altitude_lock, "AUTOPILOT ALTITUDE LOCK", "Bool") \
	NUM(autopilot_altitude_lock_var, "AUTOPILOT ALTITUDE LOCK VAR", "Feet") \
	NUM(autopilot_approach_active, "AUTOPILOT APPROACH ACTIVE", "Bool") \
	NUM(autopilot_approach_captured, "AUTOPILOT APPROACH CAPTURED", "Bool") \
	NUM(autopilot_approach_hold, "AUTOPILOT APPROACH HOLD", "Bool") \
	NUM(autopilot_approach_is_localizer, "AUTOPILOT APPROACH IS LOCALIZER", "Bool") \
	NUM(autopilot_avionics_managed, "AUTOPILOT AVIONICS MANAGED", "Bool") \
	NUM(autopilot_disengaged, "AUTOPILOT DISENGAGED", "Bool") \
	NUM(autopilot_flight_director_active, "AUTOPILOT FLIGHT DIRECTOR ACTIVE", "Bool") \
	NUM(autopilot_flight_level_change, "AUTOPILOT FLIGHT LEVEL CHANGE", "Bool") \
	NUM(autopilot_glideslope_active, "AUTOPILOT GLIDESLOPE ACTIVE", "Bool") \
	NUM(autopilot_glideslope_arm, "AUTOPILOT GLIDESLOPE ARM", "Bool") \
	NUM(autopilot_glideslope_hold, "AUTOPILOT GLIDESLOPE HOLD", "Bool") \
	NUM(autopilot_heading_lock, "AUTOPILOT HEADING LOCK", "Bool") \
	NUM(autopilot_heading_lock_dir, "AUTOPILOT HEADING LOCK DIR", "Degrees") \
	NUM(autopilot_mach_hold, "AUTOPILOT MACH HOLD", "Bool") \
	NUM(autopilot_mach_hold_var, "AUTOPILOT MACH HOLD VAR", "Number") \
	NUM(autopilot_managed_speed_in_mach, "AUTOPILOT MANAGED SPEED IN MACH", "Bool") \
	NUM(autopilot_managed_throttle_active, "AUTOPILOT MANAGED THROTTLE ACTIVE", "Bool") \
	NUM(autopilot_master, "AUTOPILOT MASTER", "Bool") \
	NUM(autopilot_takeoff_power_active, "AUTOPILOT TAKEOFF POWER ACTIVE", "Bool") \
	NUM(autopilot_throttle_arm, "AUTOPILOT THROTTLE ARM", "Bool") \
	NUM(autopilot_throttle_max_thrust, "AUTOPILOT THROTTLE MAX THRUST", "Percent") \
	NUM(autopilot_vertical_hold, "AUTOPILOT VERTICAL HOLD", "Bool") \
	NUM(autopilot_vertical_hold_var, "AUTOPILOT VERTICAL HOLD VAR", "Feet/minute") \
	NUM(autobrakes_active, "AUTOBRAKES ACTIVE", "Bool") \
	NUM(auto_brake_switch_cb, "AUTO BRAKE SWITCH CB", "Number") \
	NUM(brake_indicator, "BRAKE INDICATOR", "Position") \
	NUM(brake_parking_indicator, "BRAKE PARKING INDICATOR", "Bool") \
	NUM(rejected_takeoff_brakes_active, "REJECTED TAKEOFF BRAKES ACTIVE", "Bool") \
	NUM(gear_damage_by_speed, "GEAR DAMAGE BY SPEED", "Bool") \
	NUM(gear_handle_position, "GEAR HANDLE POSITION", "Percent Over 100") \
	NUM(gear_is_on_ground_0, "GEAR IS ON GROUND:0", "Bool") \
	NUM(gear_is_on_ground_1, "GEAR IS ON GROUND:1", "Bool") \
	NUM(gear_is_on_ground_2, "GEAR IS ON GROUND:2", "Bool") \
	NUM(gear_position_0, "GEAR POSITION:0", "Enum") \
	NUM(gear_position_1, "GEAR POSITION:1", "Enum") \
	NUM(gear_position_2, "GEAR POSITION:2", "Enum") \
	NUM(gear_speed_exceeded, "GEAR SPEED EXCEEDED", "Bool") \
	NUM(gear_warning_0, "GEAR WARNING:0", "Enum") \
	NUM(gear_warning_1, "GEAR WARNING:1", "Enum") \
	NUM(gear_warning_2, "GEAR WARNING:2", "Enum") \
	NUM(wheel_rpm_0, "WHEEL RPM:0", "RPM") \
	NUM(wheel_rpm_1, "WHEEL RPM:1", "RPM") \
	NUM(wheel_rpm_2, "WHEEL RPM:2", "RPM") \
	NUM(aileron_left_deflection, "AILERON LEFT DEFLECTION", "Degrees") \
	NUM(aileron_left_deflection_pct, "AILERON LEFT DEFLECTION PCT", "Percent Over 100") \
	NUM(aileron_right_deflection, "AILERON RIGHT DEFLECTION", "Degrees") \
	NUM(aileron_right_deflection_pct, "AILERON RIGHT DEFLECTION PCT", "Percent Over 100") \
	NUM(aileron_trim, "AILERON TRIM", "Degrees") \
	NUM(aileron_trim_disabled, "AILERON TRIM DISABLED", "Bool") \
	NUM(aileron_trim_pct, "AILERON TRIM PCT", "Percent Over 100") \
	NUM(elevator_deflection, "ELEVATOR DEFLECTION", "Degrees") \
	NUM(elevator_deflection_pct, "ELEVATOR DEFLECTION PCT", "Percent Over 100") \
	NUM(elevator_trim_disabled, "ELEVATOR TRIM DISABLED", "Bool") \
	NUM(elevator_trim_pct, "ELEVATOR TRIM PCT", "Percent Over 100") \
	NUM(elevator_trim_position, "ELEVATOR TRIM POSITION", "Degrees") \
	NUM(elevon_deflection, "ELEVON DEFLECTION", "Degrees") \
	NUM(flap_damage_by_speed, "FLAP DAMAGE BY SPEED", "Bool") \
	NUM(flap_speed_exceeded, "FLAP SPEED EXCEEDED", "Bool") \
	NUM(flaps_handle_index, "FLAPS HANDLE INDEX", "Number") \
	NUM(flaps_num_handle_positions, "FLAPS NUM HANDLE POSITIONS", "Number") \
	NUM(rudder_deflection, "RUDDER DEFLECTION", "Degrees") \
	NUM(rudder_deflection_pct, "RUDDER DEFLECTION PCT", "Percent Over 100") \
	NUM(rudder_trim, "RUDDER TRIM", "Degrees") \
	NUM(rudder_trim_disabled, "RUDDER TRIM DISABLED", "Bool") \
	NUM(rudder_trim_pct, "RUDDER TRIM PCT", "Percent Over 100") \
	NUM(spoilers_armed, "SPOILERS ARMED", "Bool") \
	NUM(spoilers_handle_position, "SPOILERS HANDLE POSITION", "Percent Over 100") \
	NUM(spoilers_left_position, "SPOILERS LEFT POSITION", "Percent Over 100") \
	NUM(spoilers_right_position, "SPOILERS RIGHT POSITION", "Percent Over 100") \
	NUM(apu_bleed_pressure_received_by_engine, "APU BLEED PRESSURE RECEIVED BY ENGINE", "psi") \
	NUM(apu_generator_active, "APU GENERATOR ACTIVE", "Bool") \
	NUM(apu_generator_switch, "APU GENERATOR SWITCH", "Bool") \
	NUM(apu_on_fire_detected, "APU ON FIRE DETECTED", "Bool") \
	NUM(apu_pct_rpm, "APU PCT RPM", "Percent Over 100") \
	NUM(apu_pct_starter, "APU PCT STARTER", "Percent Over 100") \
	NUM(apu_switch, "APU SWITCH", "Bool") \
	NUM(bleed_air_apu, "BLEED AIR APU", "Bool") \
	NUM(electrical_battery_estimated_capacity_pct, "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT", "Percent") \
	NUM(electrical_battery_voltage, "ELECTRICAL BATTERY VOLTAGE", "Volts") \
	NUM(electrical_master_battery, "ELECTRICAL MASTER BATTERY", "Bool") \
	NUM(external_power_available, "EXTERNAL POWER AVAILABLE", "Bool") \
	NUM(external_power_connection_on, "EXTERNAL POWER CONNECTION ON", "Bool") \
	NUM(external_power_on, "EXTERNAL POWER ON", "Bool") \
	NUM(bleed_air_engine_1, "BLEED AIR ENGINE:1", "Bool") \
	NUM(bleed_air_engine_2, "BLEED AIR ENGINE:2", "Bool") \
	NUM(bleed_air_source_control_1, "BLEED AIR SOURCE CONTROL:1", "Enum") \
	NUM(bleed_air_source_control_2, "BLEED AIR SOURCE CONTROL:2", "Enum") \
	NUM(engine_control_select, "ENGINE CONTROL SELECT", "Flags") \
	NUM(engine_type, "ENGINE TYPE", "Enum") \
	NUM(eng_anti_ice_1, "ENG ANTI ICE:1", "Bool") \
	NUM(eng_anti_ice_2, "ENG ANTI ICE:2", "Bool") \
	NUM(eng_combustion_1, "ENG COMBUSTION:1", "Bool") \
	NUM(eng_combustion_2, "ENG COMBUSTION:2", "Bool") \
	/* Read only to start/stop the trip (flight_phase.cpp); trip_data's \
	   bool groups have no free bit to store them in. */ \
	NUM(eng_combustion_3, "ENG COMBUSTION:3", "Bool") \
	NUM(eng_combustion_4, "ENG COMBUSTION:4", "Bool") \
	NUM(eng_exhaust_gas_temperature_1, "ENG EXHAUST GAS TEMPERATURE:1", "Celsius") \
	NUM(eng_exhaust_gas_temperature_2, "ENG EXHAUST GAS TEMPERATURE:2", "Celsius") \
	NUM(eng_failed_1, "ENG FAILED:1", "Bool") \
	NUM(eng_failed_2, "ENG FAILED:2", "Bool") \
	NUM(eng_hydraulic_pressure_1, "ENG HYDRAULIC PRESSURE:1", "psf") \
	NUM(eng_hydraulic_pressure_2, "ENG HYDRAULIC PRESSURE:2", "psf") \
	NUM(eng_oil_pressure_1, "ENG OIL PRESSURE:1", "psf") \
	NUM(eng_oil_pressure_2, "ENG OIL PRESSURE:2", "psf") \
	NUM(eng_oil_temperature_1, "ENG OIL TEMPERATURE:1", "Celsius") \
	NUM(eng_oil_temperature_2, "ENG OIL TEMPERATURE:2", "Celsius") \
	NUM(eng_on_fire_1, "ENG ON FIRE:1", "Bool") \
	NUM(eng_on_fire_2, "ENG ON FIRE:2", "Bool") \
	NUM(general_eng_damage_percent_1, "GENERAL ENG DAMAGE PERCENT:1", "Percent") \
	NUM(general_eng_damage_percent_2, "GENERAL ENG DAMAGE PERCENT:2", "Percent") \
	NUM(general_eng_elapsed_time_1, "GENERAL ENG ELAPSED TIME:1", "Hours") \
	NUM(general_eng_elapsed_time_2, "GENERAL ENG ELAPSED TIME:2", "Hours") \
	NUM(general_eng_fire_detected_1, "GENERAL ENG FIRE DETECTED:1", "Bool") \
	NUM(general_eng_fire_detected_2, "GENERAL ENG FIRE DETECTED:2", "Bool") \
	NUM(general_eng_fuel_used_since_start_1, "GENERAL ENG FUEL USED SINCE START:1", "Pounds") \
	NUM(general_eng_fuel_used_since_start_2, "GENERAL ENG FUEL USED SINCE START:2", "Pounds") \
	NUM(general_eng_fuel_valve_1, "GENERAL ENG FUEL VALVE:1", "Bool") \
	NUM(general_eng_fuel_valve_2, "GENERAL ENG FUEL VALVE:2", "Bool") \
	NUM(general_eng_generator_active_1, "GENERAL ENG GENERATOR ACTIVE:1", "Bool") \
	NUM(general_eng_generator_active_2, "GENERAL ENG GENERATOR ACTIVE:2", "Bool") \
	NUM(general_eng_generator_switch_1, "GENERAL ENG GENERATOR SWITCH:1", "Bool") \
	NUM(general_eng_generator_switch_2, "GENERAL ENG GENERATOR SWITCH:2", "Bool") \
	NUM(general_eng_master_alternator, "GENERAL ENG MASTER ALTERNATOR", "Bool") \
	NUM(general_eng_reverse_thrust_engaged, "GENERAL ENG REVERSE THRUST ENGAGED", "Bool") \
	NUM(general_eng_starter_1, "GENERAL ENG STARTER:1", "Bool") \
	NUM(general_eng_starter_2, "GENERAL ENG STARTER:2", "Bool") \
	NUM(general_eng_starter_active_1, "GENERAL ENG STARTER ACTIVE:1", "Bool") \
	NUM(general_eng_starter_active_2, "GENERAL ENG STARTER ACTIVE:2", "Bool") \
	NUM(general_eng_throttle_lever_position_1, "GENERAL ENG THROTTLE LEVER POSITION:1", "Percent") \
	NUM(general_eng_throttle_lever_position_2, "GENERAL ENG THROTTLE LEVER POSITION:2", "Percent") \
	NUM(general_eng_throttle_managed_mode_1, "GENERAL ENG THROTTLE MANAGED MODE:1", "Number") \
	NUM(general_eng_throttle_managed_mode_2, "GENERAL ENG THROTTLE MANAGED MODE:2", "Number") \
	NUM(master_ignition_switch, "MASTER IGNITION SWITCH", "Bool") \
	NUM(number_of_engines, "NUMBER OF ENGINES", "Number") \
	NUM(turb_eng_bleed_air_1, "TURB ENG BLEED AIR:1", "psi") \
	NUM(turb_eng_bleed_air_2, "TURB ENG BLEED AIR:2", "psi") \
	NUM(turb_eng_fuel_available_1, "TURB ENG FUEL AVAILABLE:1", "Bool") \
	NUM(turb_eng_fuel_available_2, "TURB ENG FUEL AVAILABLE:2", "Bool") \
	NUM(turb_eng_fuel_flow_pph_1, "TURB ENG FUEL FLOW PPH:1", "Pounds per hour") \
	NUM(turb_eng_fuel_flow_pph_2, "TURB ENG FUEL FLOW PPH:2", "Pounds per hour") \
	NUM(turb_eng_ignition_switch_ex1_1, "TURB ENG IGNITION SWITCH EX1:1", "Enum") \
	NUM(turb_eng_ignition_switch_ex1_2, "TURB ENG IGNITION SWITCH EX1:2", "Enum") \
	NUM(turb_eng_is_igniting_1, "TURB ENG IS IGNITING:1", "Bool") \
	NUM(turb_eng_is_igniting_2, "TURB ENG IS IGNITING:2", "Bool") \
	/* enginePowerFromRecord() picks which engines a sample records. */ \
	ENGINES(general_eng_rpm, "GENERAL ENG RPM", "rpm") \
	ENGINES(recip_eng_manifold_pressure, "RECIP ENG MANIFOLD PRESSURE", "inHg") \
	ENGINES(turb_eng_n1, "TURB ENG N1", "Percent") \
	ENGINES(turb_eng_n2, "TURB ENG N2", "Percent") \
	ENGINES(turb_eng_max_torque_percent, "TURB ENG MAX TORQUE PERCENT", "Percent") \
	ENGINES(prop_rpm, "PROP RPM", "rpm") \
	NUM(turb_eng_vibration_1, "TURB ENG VIBRATION:1", "Number") \
	NUM(turb_eng_vibration_2, "TURB ENG VIBRATION:2", "Number") \
	NUM(g_force, "G FORCE", "GForce") \
	NUM(empty_weight, "EMPTY WEIGHT", "Pounds") \
	NUM(total_weight, "TOTAL WEIGHT", "Pounds") \
	NUM(fuel_cross_feed_l, "FUEL CROSS FEED:2", "Enum") \
	NUM(fuel_cross_feed_r, "FUEL CROSS FEED:3", "Enum") \
	NUM(fuel_selected_quantity_l, "FUEL SELECTED QUANTITY:2", "Gallons") \
	NUM(fuel_selected_quantity_r, "FUEL SELECTED QUANTITY:3", "Gallons") \
	NUM(fuel_selected_quantity_percent_l, "FUEL SELECTED QUANTITY PERCENT:2", "Percent Over 100") \
	NUM(fuel_selected_quantity_percent_r, "FUEL SELECTED QUANTITY PERCENT:3", "Percent Over 100") \
	NUM(fuel_total_quantity, "FUEL TOTAL QUANTITY", "Gallons") \
	NUM(fuel_total_quantity_weight, "FUEL TOTAL QUANTITY WEIGHT", "Pounds") \
	NUM(fuel_transfer_pump_on_l, "FUEL TRANSFER PUMP ON:2", "Bool") \
	NUM(fuel_transfer_pump_on_r, "FUEL TRANSFER PUMP ON:3", "Bool") \
	NUM(fuel_weight_per_gallon, "FUEL WEIGHT PER GALLON", "Pounds") \
	NUM(on_any_runway, "ON ANY RUNWAY", "Bool") \
	NUM(plane_in_parking_state, "PLANE IN PARKING STATE", "Bool") \
	NUM(surface_condition, "SURFACE CONDITION", "Enum") \
	NUM(surface_type, "SURFACE TYPE", "Enum") \
	NUM(ground_velocity, "GROUND VELOCITY", "Knots") \
	NUM(plane_altitude, "PLANE ALTITUDE", "Feet") \
	NUM(plane_alt_above_ground, "PLANE ALT ABOVE GROUND", "Feet") \
	NUM(plane_bank_degrees, "PLANE BANK DEGREES", "Degrees") \
	NUM(plane_heading_degrees_gyro, "PLANE HEADING DEGREES GYRO", "Degrees") \
	NUM(plane_heading_degrees_magnetic, "PLANE HEADING DEGREES MAGNETIC", "Degrees") \
	NUM(plane_heading_degrees_true, "PLANE HEADING DEGREES TRUE", "Degrees") \
	COORD(plane_coordinate, "PLANE LATITUDE", "PLANE LONGITUDE") \
	NUM(plane_pitch_degrees, "PLANE PITCH DEGREES", "Degrees") \
	NUM(plane_touchdown_bank_degrees, "PLANE TOUCHDOWN BANK DEGREES", "Degrees") \
	NUM(plane_touchdown_heading_degrees_magnetic, "PLANE TOUCHDOWN HEADING DEGREES MAGNETIC", "Degrees") \
	NUM(plane_touchdown_heading_degrees_true, "PLANE TOUCHDOWN HEADING DEGREES TRUE", "Degrees") \
	COORD(plane_touchdown_coordinate, "PLANE TOUCHDOWN LATITUDE", "PLANE TOUCHDOWN LONGITUDE") \
	NUM(plane_touchdown_normal_velocity, "PLANE TOUCHDOWN NORMAL VELOCITY", "Feet per minute") \
	NUM(plane_touchdown_pitch_degrees, "PLANE TOUCHDOWN PITCH DEGREES", "Degrees") \
	NUM(vertical_speed, "VERTICAL SPEED", "Feet per minute") \
	NUM(airspeed_indicated, "AIRSPEED INDICATED", "Knots") \
	NUM(airspeed_mach, "AIRSPEED MACH", "Mach") \
	NUM(airspeed_true, "AIRSPEED TRUE", "Knots") \
	NUM(gps_ground_speed, "GPS GROUND SPEED", "Meters per second") \
	NUM(gps_ground_true_heading, "GPS GROUND TRUE HEADING", "Degrees") \
	NUM(gps_ground_true_track, "GPS GROUND TRUE TRACK", "Degrees") \
	NUM(gps_position_alt, "GPS POSITION ALT", "Meters") \
	COORD(gps_position_coordinate, "GPS POSITION LAT", "GPS POSITION LON") \
	NUM(radio_height, "RADIO HEIGHT", "Feet") \
	NUM(autothrottle_active, "AUTOTHROTTLE ACTIVE", "Bool") \
	NUM(avionics_master_switch, "AVIONICS MASTER SWITCH", "Bool") \
	NUM(cabin_no_smoking_alert_switch, "CABIN NO SMOKING ALERT SWITCH", "Bool") \
	NUM(cabin_seatbelts_alert_switch, "CABIN SEATBELTS ALERT SWITCH", "Bool") \
	NUM(gpws_system_active, "GPWS SYSTEM ACTIVE", "Bool") \
	NUM(gpws_warning, "GPWS WARNING", "Bool") \
	NUM(gyro_drift_error, "GYRO DRIFT ERROR", "Degrees") \
	NUM(heading_indicator, "HEADING INDICATOR", "Degrees") \
	NUM(indicated_altitude, "INDICATED ALTITUDE", "Feet") \
	NUM(indicated_altitude_calibrated, "INDICATED ALTITUDE CALIBRATED", "Feet") \
	NUM(magnetic_compass, "MAGNETIC COMPASS", "Degrees") \
	NUM(overspeed_warning, "OVERSPEED WARNING", "Bool") \
	NUM(pitot_ice_pct, "PITOT ICE PCT", "Percent Over 100") \
	NUM(pitot_heat, "PITOT HEAT", "Bool") \
	NUM(pitot_heat_switch, "PITOT HEAT SWITCH", "Enum") \
	NUM(pressure_altitude, "PRESSURE ALTITUDE", "Meters") \
	NUM(pressurization_cabin_altitude, "PRESSURIZATION CABIN ALTITUDE", "Feet") \
	NUM(stall_warning, "STALL WARNING", "Bool") \
	NUM(structural_deice_switch, "STRUCTURAL DEICE SWITCH", "Bool") \
	NUM(light_states, "LIGHT STATES", "Mask") \
	NUM(hydraulic_pressure_1, "HYDRAULIC PRESSURE:1", "psf") \
	NUM(hydraulic_pressure_2, "HYDRAULIC PRESSURE:2", "psf") \
	NUM(hydraulic_switch, "HYDRAULIC SWITCH", "Bool") \
	NUM(warning_fuel, "WARNING FUEL", "Bool") \
	NUM(warning_low_height, "WARNING LOW HEIGHT", "Bool") \
	NUM(warning_oil_pressure, "WARNING OIL PRESSURE", "Bool") \
	NUM(warning_vacuum, "WARNING VACUUM", "Bool") \
	NUM(warning_voltage, "WARNING VOLTAGE", "Bool") \
	NUM(sim_on_ground, "SIM ON GROUND", "Bool") \
	NUM(ambient_pressure, "AMBIENT PRESSURE", "inHg") \
	NUM(ambient_temperature, "AMBIENT TEMPERATURE", "Celsius") \
	NUM(ambient_visibility, "AMBIENT VISIBILITY", "Meters") \
	NUM(ambient_wind_direction, "AMBIENT WIND DIRECTION", "Degrees") \
	NUM(ambient_wind_velocity, "AMBIENT WIND VELOCITY", "Knots") \
	NUM(barometer_pressure, "BAROMETER PRESSURE", "Millibars") \
	NUM(kohlsman_setting_hg, "KOHLSMAN SETTING HG", "inHg") \
	NUM(kohlsman_setting_mb, "KOHLSMAN SETTING MB", "Millibars") \
	NUM(kohlsman_setting_std, "KOHLSMAN SETTING STD", "Bool") \
	STR(title, "TITLE", 256) \
	STR(atc_airline, "ATC AIRLINE", 64) \
	STR(atc_flight_number, "ATC FLIGHT NUMBER", 8) \
	STR(atc_id, "ATC ID", 32) \
	STR(atc_model, "ATC MODEL", 32) \
	STR(atc_type, "ATC TYPE", 64) \
	TIME(time_local, "LOCAL", "TIME ZONE OFFSET") \
	TIME(time_zulu, "ZULU", nullptr)

struct FLIGHT_DATA_RECORD {
#define FLIGHT_FIELD_NUM(member, simVar, unit) double member;
#define FLIGHT_FIELD_ENGINES(member, simVar, unit) double member[MAX_ENGINES];
#define FLIGHT_FIELD_COORD(member, latitude, longitude) COORDINATE member;
#define FLIGHT_FIELD_STR(member, simVar, size) char member[size];
#define FLIGHT_FIELD_TIME(member, prefix, offset) DATETIME member;
	FLIGHT_DATA_FIELDS(FLIGHT_FIELD_NUM, FLIGHT_FIELD_ENGINES, FLIGHT_FIELD_COORD, FLIGHT_FIELD_STR, FLIGHT_FIELD_TIME)
#undef FLIGHT_FIELD_NUM
#undef FLIGHT_FIELD_ENGINES
#undef FLIGHT_FIELD_COORD
#undef FLIGHT_FIELD_STR
#undef FLIGHT_FIELD_TIME
};

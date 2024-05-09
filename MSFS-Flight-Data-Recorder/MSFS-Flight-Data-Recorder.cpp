// MSFS-Flight-Recorder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <mutex>
#include <thread>
#include <Windows.h>
#include "SimConnect.h"
#include "sqlite3.h"

// Queue length before committing to database
#define Q_DB_LENGTH 100
// Interval for data sampling to store in database, in second.
#define DB_SAMPLING_INTERVAL 0.3
#define DATABASE_NAME "flight_data"
#define V_PI 3.14159265358979323846
#define EARTHRADIUSKM 6371.0

// SimConnect
// Events
static enum GROUP_ID {
	GROUP_1,
};
static enum EVENT_ID {
	EVENT_SIM,
	EVENT_PAUSE,
	EVENT_CRASHED,
	EVENT_AP_AIRSPEED_HOLD,
	EVENT_AP_AIRSPEED_OFF,
	EVENT_AP_AIRSPEED_ON,
	EVENT_AP_ALT_HOLD,
	EVENT_AP_ALT_HOLD_OFF,
	EVENT_AP_ALT_HOLD_ON,
	EVENT_AP_APR_HOLD,
	EVENT_AP_APR_HOLD_OFF,
	EVENT_AP_APR_HOLD_ON,
	EVENT_AP_HDG_HOLD,
	EVENT_AP_HDG_HOLD_OFF,
	EVENT_AP_HDG_HOLD_ON,
	EVENT_AP_MACH_HOLD,
	EVENT_AP_MACH_OFF,
	EVENT_AP_MACH_ON,
	EVENT_AP_MASTER,
	EVENT_AP_PANEL_ALTITUDE_HOLD,
	EVENT_AP_PANEL_ALTITUDE_OFF,
	EVENT_AP_PANEL_ALTITUDE_ON,
	EVENT_AP_PANEL_HEADING_HOLD,
	EVENT_AP_PANEL_HEADING_OFF,
	EVENT_AP_PANEL_HEADING_ON,
	EVENT_AP_PANEL_MACH_HOLD,
	EVENT_AP_PANEL_MACH_OFF,
	EVENT_AP_PANEL_MACH_ON,
	EVENT_AP_PANEL_SPEED_HOLD,
	EVENT_AP_PANEL_SPEED_OFF,
	EVENT_AP_PANEL_SPEED_ON,
	EVENT_AP_PANEL_VS_OFF,
	EVENT_AP_PANEL_VS_ON,
	EVENT_AP_PANEL_VS_HOLD,
	EVENT_AP_VS_HOLD,
	EVENT_AP_VS_OFF,
	EVENT_AP_VS_ON,
	EVENT_AP_PANEL_SPEED_HOLD_TOGGLE,
	EVENT_AP_PANEL_MACH_HOLD_TOGGLE,
	EVENT_AUTOPILOT_DISENGAGE_TOGGLE,
	EVENT_AUTOPILOT_OFF,
	EVENT_AUTOPILOT_ON,
	EVENT_AUTOPILOT_PANEL_AIRSPEED_SET,
	EVENT_FLIGHT_LEVEL_CHANGE,
	EVENT_FLIGHT_LEVEL_CHANGE_OFF,
	EVENT_FLIGHT_LEVEL_CHANGE_ON,
	EVENT_AUTO_THROTTLE_ARM,
	EVENT_AUTO_THROTTLE_TO_GA,
	EVENT_AUTOBRAKE_DISARM,
	EVENT_AUTOBRAKE_HI_SET,
	EVENT_AUTOBRAKE_LO_SET,
	EVENT_AUTOBRAKE_MED_SET,
	EVENT_GPWS_SWITCH_TOGGLE,
	EVENT_TOGGLE_FLIGHT_DIRECTOR,
	EVENT_APU_BLEED_AIR_SOURCE_TOGGLE,
	EVENT_APU_GENERATOR_SWITCH_TOGGLE,
	EVENT_APU_OFF_SWITCH,
	EVENT_APU_STARTER,
	EVENT_ANTI_ICE_ON,
	EVENT_ANTI_ICE_OFF,
	EVENT_ANTI_ICE_TOGGLE,
	EVENT_ANTI_ICE_TOGGLE_ENG1,
	EVENT_ANTI_ICE_TOGGLE_ENG2,
	EVENT_THROTTLE_REVERSE_THRUST_TOGGLE,
	EVENT_FLAPS_DECR,
	EVENT_FLAPS_DOWN,
	EVENT_FLAPS_INCR,
	EVENT_FLAPS_UP,
	EVENT_SPOILERS_ARM_OFF,
	EVENT_SPOILERS_ARM_ON,
	EVENT_SPOILERS_ARM_TOGGLE,
	EVENT_SPOILERS_OFF,
	EVENT_SPOILERS_ON,
	EVENT_SPOILERS_TOGGLE,
	EVENT_CROSS_FEED_TOGGLE,
	EVENT_BRAKES,
	EVENT_GEAR_DOWN,
	EVENT_GEAR_EMERGENCY_HANDLE_TOGGLE,
	EVENT_GEAR_TOGGLE,
	EVENT_GEAR_UP,
	EVENT_PARKING_BRAKES,
	EVENT_CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE,
	EVENT_CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE,
	EVENT_WINDSHIELD_DEICE_OFF,
	EVENT_WINDSHIELD_DEICE_ON,
	EVENT_WINDSHIELD_DEICE_TOGGLE,
	EVENT_TOGGLE_AVIONICS_MASTER,
};
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

// Data
static enum DATA_DEFINE_ID {
	DEFINITION_A320,
	DEFINITION_RUNWAYS,
};
static enum DATA_REQUEST_ID {
	REQUEST_A320,
	REQUEST_AIRPORTS,
	REQUEST_RUNWAYS,
};

// A320
struct DATA_A320 {
	double autopilot_airspeed_hold;
	double autopilot_airspeed_hold_var;
	double autopilot_alt_radio_mode;
	double autopilot_altitude_lock;
	double autopilot_altitude_lock_var;
	double autopilot_approach_active;
	double autopilot_approach_captured;
	double autopilot_approach_hold;
	double autopilot_approach_is_localizer;
	double autopilot_avionics_managed;
	double autopilot_disengaged;
	double autopilot_flight_director_active;
	double autopilot_flight_level_change;
	double autopilot_glideslope_active;
	double autopilot_glideslope_arm;
	double autopilot_glideslope_hold;
	double autopilot_heading_lock;
	double autopilot_heading_lock_dir;
	double autopilot_mach_hold;
	double autopilot_mach_hold_var;
	double autopilot_managed_speed_in_mach;
	double autopilot_managed_throttle_active;
	double autopilot_master;
	double autopilot_takeoff_power_active;
	double autopilot_throttle_arm;
	double autopilot_throttle_max_thrust;
	double autopilot_vertical_hold;
	double autopilot_vertical_hold_var;
	double autobrakes_active;
	double auto_brake_switch_cb;
	double brake_indicator;
	double brake_parking_indicator;
	double rejected_takeoff_brakes_active;
	double gear_damage_by_speed;
	double gear_handle_position;
	double gear_is_on_ground_0;
	double gear_is_on_ground_1;
	double gear_is_on_ground_2;
	double gear_position_0;
	double gear_position_1;
	double gear_position_2;
	double gear_speed_exceeded;
	double gear_warning_0;
	double gear_warning_1;
	double gear_warning_2;
	double wheel_rpm_0;
	double wheel_rpm_1;
	double wheel_rpm_2;
	double aileron_left_deflection;
	double aileron_left_deflection_pct;
	double aileron_right_deflection;
	double aileron_right_deflection_pct;
	double aileron_trim;
	double aileron_trim_disabled;
	double aileron_trim_pct;
	double elevator_deflection;
	double elevator_deflection_pct;
	double elevator_trim_disabled;
	double elevator_trim_pct;
	double elevator_trim_position;
	double elevon_deflection;
	double flap_damage_by_speed;
	double flap_speed_exceeded;
	double flaps_handle_index;
	double flaps_num_handle_positions;
	double rudder_deflection;
	double rudder_deflection_pct;
	double rudder_trim;
	double rudder_trim_disabled;
	double rudder_trim_pct;
	double spoilers_armed;
	double spoilers_handle_position;
	double spoilers_left_position;
	double spoilers_right_position;
	double apu_bleed_pressure_received_by_engine;
	double apu_generator_active;
	double apu_generator_switch;
	double apu_on_fire_detected;
	double apu_pct_rpm;
	double apu_pct_starter;
	double apu_switch;
	double bleed_air_apu;
	double electrical_battery_estimated_capacity_pct;
	double electrical_battery_voltage;
	double electrical_master_battery;
	double external_power_available;
	double external_power_connection_on;
	double external_power_on;
	double bleed_air_engine_1;
	double bleed_air_engine_2;
	double bleed_air_source_control_1;
	double bleed_air_source_control_2;
	double engine_control_select;
	double engine_type;
	double eng_anti_ice_1;
	double eng_anti_ice_2;
	double eng_combustion_1;
	double eng_combustion_2;
	double eng_exhaust_gas_temperature_1;
	double eng_exhaust_gas_temperature_2;
	double eng_failed_1;
	double eng_failed_2;
	double eng_hydraulic_pressure_1;
	double eng_hydraulic_pressure_2;
	double eng_oil_pressure_1;
	double eng_oil_pressure_2;
	double eng_oil_temperature_1;
	double eng_oil_temperature_2;
	double eng_on_fire_1;
	double eng_on_fire_2;
	double general_eng_damage_percent_1;
	double general_eng_damage_percent_2;
	double general_eng_elapsed_time_1;
	double general_eng_elapsed_time_2;
	double general_eng_fire_detected_1;
	double general_eng_fire_detected_2;
	double general_eng_fuel_used_since_start_1;
	double general_eng_fuel_used_since_start_2;
	double general_eng_fuel_valve_1;
	double general_eng_fuel_valve_2;
	double general_eng_generator_active_1;
	double general_eng_generator_active_2;
	double general_eng_generator_switch_1;
	double general_eng_generator_switch_2;
	double general_eng_master_alternator;
	double general_eng_reverse_thrust_engaged;
	double general_eng_starter_1;
	double general_eng_starter_2;
	double general_eng_starter_active_1;
	double general_eng_starter_active_2;
	double general_eng_throttle_lever_position_1;
	double general_eng_throttle_lever_position_2;
	double general_eng_throttle_managed_mode_1;
	double general_eng_throttle_managed_mode_2;
	double master_ignition_switch;
	double number_of_engines;
	double turb_eng_bleed_air_1;
	double turb_eng_bleed_air_2;
	double turb_eng_fuel_available_1;
	double turb_eng_fuel_available_2;
	double turb_eng_fuel_flow_pph_1;
	double turb_eng_fuel_flow_pph_2;
	double turb_eng_ignition_switch_ex1_1;
	double turb_eng_ignition_switch_ex1_2;
	double turb_eng_is_igniting_1;
	double turb_eng_is_igniting_2;
	double turb_eng_n1_1;
	double turb_eng_n1_2;
	double turb_eng_n2_1;
	double turb_eng_n2_2;
	double turb_eng_vibration_1;
	double turb_eng_vibration_2;
	double g_force;
	double empty_weight;
	double total_weight;
	double fuel_cross_feed_l;
	double fuel_cross_feed_r;
	double fuel_selected_quantity_l;
	double fuel_selected_quantity_r;
	double fuel_selected_quantity_percent_l;
	double fuel_selected_quantity_percent_r;
	double fuel_total_quantity;
	double fuel_total_quantity_weight;
	double fuel_transfer_pump_on_l;
	double fuel_transfer_pump_on_r;
	double fuel_weight_per_gallon;
	double on_any_runway;
	double plane_in_parking_state;
	double surface_condition;
	double surface_type;
	double ground_velocity;
	double plane_altitude;
	double plane_alt_above_ground;
	double plane_bank_degrees;
	double plane_heading_degrees_gyro;
	double plane_heading_degrees_magnetic;
	double plane_heading_degrees_true;
	double plane_latitude;
	double plane_longitude;
	double plane_pitch_degrees;
	double plane_touchdown_bank_degrees;
	double plane_touchdown_heading_degrees_magnetic;
	double plane_touchdown_heading_degrees_true;
	double plane_touchdown_latitude;
	double plane_touchdown_longitude;
	double plane_touchdown_normal_velocity;
	double plane_touchdown_pitch_degrees;
	double vertical_speed;
	double airspeed_indicated;
	double airspeed_mach;
	double airspeed_true;
	double gps_ground_speed;
	double gps_ground_true_heading;
	double gps_ground_true_track;
	double gps_position_alt;
	double gps_position_lat;
	double gps_position_lon;
	double radio_height;
	double autothrottle_active;
	double avionics_master_switch;
	double cabin_no_smoking_alert_switch;
	double cabin_seatbelts_alert_switch;
	double gpws_system_active;
	double gpws_warning;
	double gyro_drift_error;
	double heading_indicator;
	double indicated_altitude;
	double indicated_altitude_calibrated;
	double magnetic_compass;
	double overspeed_warning;
	double pitot_ice_pct;
	double pitot_heat;
	double pitot_heat_switch;
	double pressure_altitude;
	double pressurization_cabin_altitude;
	double stall_warning;
	double structural_deice_switch;
	double light_states;
	double hydraulic_pressure_1;
	double hydraulic_pressure_2;
	double hydraulic_switch;
	double warning_fuel;
	double warning_low_height;
	double warning_oil_pressure;
	double warning_vacuum;
	double warning_voltage;
	double sim_on_ground;
	double ambient_pressure;
	double ambient_temperature;
	double ambient_visibility;
	double ambient_wind_direction;
	double ambient_wind_velocity;
	double barometer_pressure;
	double zulu_time;
	double zulu_day_of_week;
	double zulu_day_of_month;
	double zulu_month_of_year;
	double zulu_year;
	double local_time;
	double local_day_of_week;
	double local_day_of_month;
	double local_month_of_year;
	double local_year;
	double time_zone_offset;
	double kohlsman_setting_hg;
	double kohlsman_setting_mb;
	double kohlsman_setting_std;
	char title[256];
	char atc_airline[64];
	char atc_flight_number[8];
	char atc_id[32];
	char atc_model[32];
	char atc_type[64];
	struct DATA_A320* next;
};
void add_definition_a320(HANDLE hSimConnect) {
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT AIRSPEED HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT AIRSPEED HOLD VAR", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT ALT RADIO MODE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT ALTITUDE LOCK", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT ALTITUDE LOCK VAR", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT APPROACH ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT APPROACH CAPTURED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT APPROACH HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT APPROACH IS LOCALIZER", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT AVIONICS MANAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT DISENGAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT FLIGHT DIRECTOR ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT FLIGHT LEVEL CHANGE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT GLIDESLOPE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT GLIDESLOPE ARM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT GLIDESLOPE HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT HEADING LOCK", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT HEADING LOCK DIR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT MACH HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT MACH HOLD VAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT MANAGED SPEED IN MACH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT MANAGED THROTTLE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT MASTER", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT TAKEOFF POWER ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT THROTTLE ARM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT THROTTLE MAX THRUST", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT VERTICAL HOLD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOPILOT VERTICAL HOLD VAR", "Feet/minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOBRAKES ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTO BRAKE SWITCH CB", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BRAKE INDICATOR", "Position");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BRAKE PARKING INDICATOR", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "REJECTED TAKEOFF BRAKES ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR DAMAGE BY SPEED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR HANDLE POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR IS ON GROUND:0", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR IS ON GROUND:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR IS ON GROUND:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR POSITION:0", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR POSITION:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR POSITION:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR SPEED EXCEEDED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR WARNING:0", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR WARNING:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GEAR WARNING:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WHEEL RPM:0", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WHEEL RPM:1", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WHEEL RPM:2", "RPM");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON LEFT DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON LEFT DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON RIGHT DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON RIGHT DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON TRIM", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AILERON TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVATOR DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVATOR DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVATOR TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVATOR TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVATOR TRIM POSITION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELEVON DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FLAP DAMAGE BY SPEED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FLAP SPEED EXCEEDED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FLAPS HANDLE INDEX", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FLAPS NUM HANDLE POSITIONS", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RUDDER DEFLECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RUDDER DEFLECTION PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RUDDER TRIM", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RUDDER TRIM DISABLED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RUDDER TRIM PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SPOILERS ARMED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SPOILERS HANDLE POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SPOILERS LEFT POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SPOILERS RIGHT POSITION", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU BLEED PRESSURE RECEIVED BY ENGINE", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU GENERATOR ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU GENERATOR SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU ON FIRE DETECTED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU PCT RPM", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU PCT STARTER", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "APU SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BLEED AIR APU", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELECTRICAL BATTERY VOLTAGE", "Volts");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ELECTRICAL MASTER BATTERY", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "EXTERNAL POWER AVAILABLE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "EXTERNAL POWER CONNECTION ON", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "EXTERNAL POWER ON", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BLEED AIR ENGINE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BLEED AIR ENGINE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BLEED AIR SOURCE CONTROL:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BLEED AIR SOURCE CONTROL:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENGINE CONTROL SELECT", "Flags");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENGINE TYPE", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG ANTI ICE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG ANTI ICE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG COMBUSTION:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG COMBUSTION:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG EXHAUST GAS TEMPERATURE:1", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG EXHAUST GAS TEMPERATURE:2", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG FAILED:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG FAILED:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG HYDRAULIC PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG HYDRAULIC PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG OIL PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG OIL PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG OIL TEMPERATURE:1", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG OIL TEMPERATURE:2", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG ON FIRE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ENG ON FIRE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG DAMAGE PERCENT:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG DAMAGE PERCENT:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG ELAPSED TIME:1", "Hours");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG ELAPSED TIME:2", "Hours");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FIRE DETECTED:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FIRE DETECTED:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FUEL USED SINCE START:1", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FUEL USED SINCE START:2", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FUEL VALVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG FUEL VALVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG GENERATOR ACTIVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG GENERATOR ACTIVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG GENERATOR SWITCH:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG GENERATOR SWITCH:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG MASTER ALTERNATOR", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG REVERSE THRUST ENGAGED", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG STARTER:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG STARTER:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG STARTER ACTIVE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG STARTER ACTIVE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG THROTTLE LEVER POSITION:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG THROTTLE LEVER POSITION:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG THROTTLE MANAGED MODE:1", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GENERAL ENG THROTTLE MANAGED MODE:2", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "MASTER IGNITION SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "NUMBER OF ENGINES", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG BLEED AIR:1", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG BLEED AIR:2", "psi");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG FUEL AVAILABLE:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG FUEL AVAILABLE:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG FUEL FLOW PPH:1", "Pounds per hour");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG FUEL FLOW PPH:2", "Pounds per hour");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG IGNITION SWITCH EX1:1", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG IGNITION SWITCH EX1:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG IS IGNITING:1", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG IS IGNITING:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG N1:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG N1:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG N2:1", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG N2:2", "Percent");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG VIBRATION:1", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TURB ENG VIBRATION:2", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "G FORCE", "GForce");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "EMPTY WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TOTAL WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL CROSS FEED:2", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL CROSS FEED:3", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL SELECTED QUANTITY:2", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL SELECTED QUANTITY:3", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL SELECTED QUANTITY PERCENT:2", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL SELECTED QUANTITY PERCENT:3", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL TOTAL QUANTITY", "Gallons");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL TOTAL QUANTITY WEIGHT", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL TRANSFER PUMP ON:2", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL TRANSFER PUMP ON:3", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "FUEL WEIGHT PER GALLON", "Pounds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ON ANY RUNWAY", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE IN PARKING STATE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SURFACE CONDITION", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SURFACE TYPE", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GROUND VELOCITY", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE ALT ABOVE GROUND", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE BANK DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE HEADING DEGREES GYRO", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE HEADING DEGREES MAGNETIC", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE HEADING DEGREES TRUE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE LATITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE LONGITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE PITCH DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN BANK DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN HEADING DEGREES MAGNETIC", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN HEADING DEGREES TRUE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN LATITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN LONGITUDE", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN NORMAL VELOCITY", "Feet per minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PLANE TOUCHDOWN PITCH DEGREES", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "VERTICAL SPEED", "Feet per minute");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AIRSPEED INDICATED", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AIRSPEED MACH", "Mach");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AIRSPEED TRUE", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS GROUND SPEED", "Meters per second");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS GROUND TRUE HEADING", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS GROUND TRUE TRACK", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS POSITION ALT", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS POSITION LAT", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPS POSITION LON", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "RADIO HEIGHT", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AUTOTHROTTLE ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AVIONICS MASTER SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "CABIN NO SMOKING ALERT SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "CABIN SEATBELTS ALERT SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPWS SYSTEM ACTIVE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GPWS WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "GYRO DRIFT ERROR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "HEADING INDICATOR", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "INDICATED ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "INDICATED ALTITUDE CALIBRATED", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "MAGNETIC COMPASS", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "OVERSPEED WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PITOT ICE PCT", "Percent Over 100");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PITOT HEAT", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PITOT HEAT SWITCH", "Enum");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PRESSURE ALTITUDE", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "PRESSURIZATION CABIN ALTITUDE", "Feet");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "STALL WARNING", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "STRUCTURAL DEICE SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LIGHT STATES", "Mask");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "HYDRAULIC PRESSURE:1", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "HYDRAULIC PRESSURE:2", "psf");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "HYDRAULIC SWITCH", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WARNING FUEL", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WARNING LOW HEIGHT", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WARNING OIL PRESSURE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WARNING VACUUM", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "WARNING VOLTAGE", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "SIM ON GROUND", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AMBIENT PRESSURE", "inHg");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AMBIENT TEMPERATURE", "Celsius");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AMBIENT VISIBILITY", "Meters");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AMBIENT WIND DIRECTION", "Degrees");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "AMBIENT WIND VELOCITY", "Knots");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "BAROMETER PRESSURE", "Millibars");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ZULU TIME", "Seconds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ZULU DAY OF WEEK", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ZULU DAY OF MONTH", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ZULU MONTH OF YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ZULU YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LOCAL TIME", "Seconds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LOCAL DAY OF WEEK", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LOCAL DAY OF MONTH", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LOCAL MONTH OF YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "LOCAL YEAR", "Number");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TIME ZONE OFFSET", "Seconds");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "KOHLSMAN SETTING HG", "inHg");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "KOHLSMAN SETTING MB", "Millibars");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "KOHLSMAN SETTING STD", "Bool");
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "TITLE", NULL, SIMCONNECT_DATATYPE_STRING256);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ATC AIRLINE", NULL, SIMCONNECT_DATATYPE_STRING64);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ATC FLIGHT NUMBER", NULL, SIMCONNECT_DATATYPE_STRING8);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ATC ID", NULL, SIMCONNECT_DATATYPE_STRING32);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ATC MODEL", NULL, SIMCONNECT_DATATYPE_STRING32);
	SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_A320, "ATC TYPE", NULL, SIMCONNECT_DATATYPE_STRING64);
	SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_A320, DEFINITION_A320, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME);
}

// Database
static const char* DATABASE_TABLE_NAMES[] = {
	"trips",
	"trip_data",
	"trip_events",
	"trip_touchdowns"
};

static const char* DATABASE_TABLE_FIELDS[] = {
	"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
	"title VARCHAR(256) NOT NULL,"
	"atc_airline VARCHAR(64) NOT NULL,"
	"atc_flight_number VARCHAR(8) NOT NULL,"
	"atc_id VARCHAR(32) NOT NULL,"
	"atc_model VARCHAR(32) NOT NULL,"
	"atc_type VARCHAR(64) NOT NULL,"
	"departure_latitude REAL NOT NULL,"
	"departure_longitude REAL NOT NULL,"
	"departure_icao VARCHAR(4),"
	"departure_rwy VARCHAR(3),"
	"departure_zulu_time VARCHAR(32) NOT NULL,"
	"departure_local_time VARCHAR(32) NOT NULL,"
	"destination_latitude REAL,"
	"destination_longitude REAL,"
	"destination_icao VARCHAR(4),"
	"destination_rwy VARCHAR(3),"
	"destination_zulu_time VARCHAR(32),"
	"destination_local_time VARCHAR(32)",

	"trip INTEGER NOT NULL,"
	"bool_group_1 INTEGER NOT NULL,"
	"bool_group_2 INTEGER NOT NULL,"
	"bool_group_3 INTEGER NOT NULL,"
	"eng_exhaust_gas_temperature_1 INTEGER NOT NULL,"
	"eng_exhaust_gas_temperature_2 INTEGER NOT NULL,"
	"eng_oil_temperature_1 INTEGER NOT NULL,"
	"eng_oil_temperature_2 INTEGER NOT NULL,"
	"ambient_temperature INTEGER NOT NULL,"
	"autopilot_heading_lock_dir INTEGER NOT NULL,"
	"aileron_left_deflection REAL NOT NULL,"
	"aileron_right_deflection REAL NOT NULL,"
	"aileron_trim REAL NOT NULL,"
	"elevator_deflection REAL NOT NULL,"
	"elevator_trim_position REAL NOT NULL,"
	"elevon_deflection REAL NOT NULL,"
	"rudder_deflection REAL NOT NULL,"
	"rudder_trim REAL NOT NULL,"
	"plane_bank_degrees REAL NOT NULL,"
	"plane_heading_degrees_gyro INTEGER NOT NULL,"
	"plane_heading_degrees_magnetic INTEGER NOT NULL,"
	"plane_heading_degrees_true INTEGER NOT NULL,"
	"plane_latitude REAL NOT NULL,"
	"plane_longitude REAL NOT NULL,"
	"plane_pitch_degrees REAL NOT NULL,"
	"plane_touchdown_bank_degrees REAL NOT NULL,"
	"plane_touchdown_heading_degrees_magnetic INTEGER NOT NULL,"
	"plane_touchdown_heading_degrees_true INTEGER NOT NULL,"
	"plane_touchdown_latitude REAL NOT NULL,"
	"plane_touchdown_longitude REAL NOT NULL,"
	"plane_touchdown_pitch_degrees REAL NOT NULL,"
	"gps_ground_true_heading INTEGER NOT NULL,"
	"gps_ground_true_track INTEGER NOT NULL,"
	"gps_position_lat REAL NOT NULL,"
	"gps_position_lon REAL NOT NULL,"
	"gyro_drift_error INTEGER NOT NULL,"
	"heading_indicator INTEGER NOT NULL,"
	"magnetic_compass INTEGER NOT NULL,"
	"ambient_wind_direction INTEGER NOT NULL,"
	"gear_position_0 INTEGER NOT NULL,"
	"gear_position_1 INTEGER NOT NULL,"
	"gear_position_2 INTEGER NOT NULL,"
	"gear_warning_0 INTEGER NOT NULL,"
	"gear_warning_1 INTEGER NOT NULL,"
	"gear_warning_2 INTEGER NOT NULL,"
	"bleed_air_source_control_1 INTEGER NOT NULL,"
	"bleed_air_source_control_2 INTEGER NOT NULL,"
	"engine_type INTEGER NOT NULL,"
	"turb_eng_ignition_switch_ex1_1 INTEGER NOT NULL,"
	"turb_eng_ignition_switch_ex1_2 INTEGER NOT NULL,"
	"fuel_cross_feed_l INTEGER NOT NULL,"
	"fuel_cross_feed_r INTEGER NOT NULL,"
	"surface_condition INTEGER NOT NULL,"
	"surface_type INTEGER NOT NULL,"
	"pitot_heat_switch INTEGER NOT NULL,"
	"plane_touchdown_normal_velocity INTEGER NOT NULL,"
	"vertical_speed INTEGER NOT NULL,"
	"autopilot_altitude_lock_var INTEGER NOT NULL,"
	"plane_altitude INTEGER NOT NULL,"
	"plane_alt_above_ground INTEGER NOT NULL,"
	"radio_height INTEGER NOT NULL,"
	"indicated_altitude INTEGER NOT NULL,"
	"indicated_altitude_calibrated INTEGER NOT NULL,"
	"pressurization_cabin_altitude INTEGER NOT NULL,"
	"autopilot_vertical_hold_var INTEGER NOT NULL,"
	"engine_control_select INTEGER NOT NULL,"
	"fuel_selected_quantity_l INTEGER NOT NULL,"
	"fuel_selected_quantity_r INTEGER NOT NULL,"
	"fuel_total_quantity INTEGER NOT NULL,"
	"g_force REAL NOT NULL,"
	"general_eng_elapsed_time_1 REAL NOT NULL,"
	"general_eng_elapsed_time_2 REAL NOT NULL,"
	"ambient_pressure REAL NOT NULL,"
	"kohlsman_setting_hg REAL NOT NULL,"
	"autopilot_airspeed_hold_var INTEGER NOT NULL,"
	"ground_velocity INTEGER NOT NULL,"
	"airspeed_indicated INTEGER NOT NULL,"
	"airspeed_true INTEGER NOT NULL,"
	"ambient_wind_velocity INTEGER NOT NULL,"
	"airspeed_mach REAL NOT NULL,"
	"light_states INTEGER NOT NULL,"
	"gps_ground_speed INTEGER NOT NULL,"
	"gps_position_alt INTEGER NOT NULL,"
	"pressure_altitude INTEGER NOT NULL,"
	"ambient_visibility INTEGER NOT NULL,"
	"barometer_pressure INTEGER NOT NULL,"
	"kohlsman_setting_mb INTEGER NOT NULL,"
	"autopilot_mach_hold_var REAL NOT NULL,"
	"auto_brake_switch_cb INTEGER NOT NULL,"
	"flaps_handle_index INTEGER NOT NULL,"
	"flaps_num_handle_positions INTEGER NOT NULL,"
	"general_eng_throttle_managed_mode_1 REAL NOT NULL,"
	"general_eng_throttle_managed_mode_2 REAL NOT NULL,"
	"number_of_engines INTEGER NOT NULL,"
	"turb_eng_vibration_1 REAL NOT NULL,"
	"turb_eng_vibration_2 REAL NOT NULL,"
	"gear_handle_position REAL NOT NULL,"
	"aileron_left_deflection_pct REAL NOT NULL,"
	"aileron_right_deflection_pct REAL NOT NULL,"
	"aileron_trim_pct REAL NOT NULL,"
	"elevator_deflection_pct REAL NOT NULL,"
	"elevator_trim_pct REAL NOT NULL,"
	"rudder_deflection_pct REAL NOT NULL,"
	"rudder_trim_pct REAL NOT NULL,"
	"spoilers_handle_position REAL NOT NULL,"
	"spoilers_left_position REAL NOT NULL,"
	"spoilers_right_position REAL NOT NULL,"
	"apu_pct_rpm REAL NOT NULL,"
	"apu_pct_starter REAL NOT NULL,"
	"fuel_selected_quantity_percent_l REAL NOT NULL,"
	"fuel_selected_quantity_percent_r REAL NOT NULL,"
	"pitot_ice_pct REAL NOT NULL,"
	"autopilot_throttle_max_thrust REAL NOT NULL,"
	"electrical_battery_estimated_capacity_pct REAL NOT NULL,"
	"general_eng_damage_percent_1 REAL NOT NULL,"
	"general_eng_damage_percent_2 REAL NOT NULL,"
	"general_eng_throttle_lever_position_1 REAL NOT NULL,"
	"general_eng_throttle_lever_position_2 REAL NOT NULL,"
	"turb_eng_n1_1 REAL NOT NULL,"
	"turb_eng_n1_2 REAL NOT NULL,"
	"turb_eng_n2_1 REAL NOT NULL,"
	"turb_eng_n2_2 REAL NOT NULL,"
	"brake_indicator INTEGER NOT NULL,"
	"turb_eng_fuel_flow_pph_1 INTEGER NOT NULL,"
	"turb_eng_fuel_flow_pph_2 INTEGER NOT NULL,"
	"general_eng_fuel_used_since_start_1 INTEGER NOT NULL,"
	"general_eng_fuel_used_since_start_2 INTEGER NOT NULL,"
	"empty_weight INTEGER NOT NULL,"
	"total_weight INTEGER NOT NULL,"
	"fuel_total_quantity_weight INTEGER NOT NULL,"
	"fuel_weight_per_gallon REAL NOT NULL,"
	"eng_hydraulic_pressure_1 INTEGER NOT NULL,"
	"eng_hydraulic_pressure_2 INTEGER NOT NULL,"
	"eng_oil_pressure_1 INTEGER NOT NULL,"
	"eng_oil_pressure_2 INTEGER NOT NULL,"
	"hydraulic_pressure_1 INTEGER NOT NULL,"
	"hydraulic_pressure_2 INTEGER NOT NULL,"
	"apu_bleed_pressure_received_by_engine INTEGER NOT NULL,"
	"turb_eng_bleed_air_1 INTEGER NOT NULL,"
	"turb_eng_bleed_air_2 INTEGER NOT NULL,"
	"wheel_rpm_0 INTEGER NOT NULL,"
	"wheel_rpm_1 INTEGER NOT NULL,"
	"wheel_rpm_2 INTEGER NOT NULL,"
	"electrical_battery_voltage REAL NOT NULL,"
	"zulu_time VARCHAR(32) NOT NULL,"
	"local_time VARCHAR(32) NOT NULL",

	"trip INTEGER NOT NULL,"
	"event VARCHAR(32) NOT NULL,"
	"time_zulu VARCHAR(32) NOT NULL,"
	"time_local VARCHAR(32) NOT NULL",

	"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
	"trip INTEGER NOT NULL,"
	"airspeed_indicated INTEGER NOT NULL,"
	"vertical_speed INTEGER NOT NULL,"
	"g_force REAL NOT NULL,"
	"plane_pitch_degrees REAL NOT NULL,"
	"plane_bank_degrees REAL NOT NULL,"
	"heading_indicator INTEGER NOT NULL,"
	"plane_latitude REAL NOT NULL,"
	"plane_longitude REAL NOT NULL,"
	"icao VARCHAR(4),"
	"runway VARCHAR(3),"
	"time_zulu VARCHAR(32) NOT NULL,"
	"time_local VARCHAR(32) NOT NULL"
};

// Generic
// Data Types
	//pS->zulu_year, pS->zulu_month_of_year, pS->zulu_day_of_month, pS->zulu_time, 0, pS->zulu_day_of_week
struct DATETIME {
	int year = 0;
	int month_of_year = 0;
	int day_of_month = 0;
	double time_day = 0;
	int day_of_week = 0;
	double timezone_offset = 0;
};

struct EVENT_DB {
	char event[32];
	struct DATETIME time_zulu;
	struct DATETIME time_local;
	struct EVENT_DB* next;
};

static enum COORDINATE_CAT {
	LATITUDE,
	LONGITUDE,
};

struct COORDINATE {
	double latitude = 0;
	double longitude = 0;
};

struct RUNWAY {
	struct COORDINATE coordinate;
	float length = 0;
	float width = 0;
	float heading = 0;
	int primary_number = 0;
	int primary_designator = 0;
	int secondary_number = 0;
	int secondary_designator = 0;
};

struct FACILITY_AIRPORT {
	char name[64] = "";
	float magvar = 0;
	int n_runways = 0;
};

struct AIRPORT {
	char icao[5] = "";
	struct FACILITY_AIRPORT extra_info;
	int runway_act_index = -1;
	bool runway_act_primary = TRUE;
	struct RUNWAY* runways = NULL;
};

struct FLIGHT_DATA {
	int heading = 0;
	int altitude = 0;
	int speed = 0;
	int vertical_speed = 0;
	double g_force = 1;
	double pitch = 0;
	double bank = 0;
	struct COORDINATE coordinate;
	char icao[5];
	char runway[4];
	struct DATETIME time_zulu;
	struct DATETIME time_local;
	struct FLIGHT_DATA* next = NULL;
};

struct STATUS {
	bool in_sim = FALSE;
	bool sim_running = FALSE;
	bool paused = FALSE;
	bool recording = FALSE;
	bool quit = FALSE;
	struct DATA_A320* q_data_db_start = NULL;
	struct DATA_A320* q_data_db_end = NULL;
	struct DATA_A320* q_data_end = NULL;
	struct DATA_A320* q_data_last = NULL;
	int q_data_db_length = 0;
	struct EVENT_DB* q_event_start = NULL;
	struct EVENT_DB* q_event_end = NULL;
	HANDLE hSimConnect = NULL;
	sqlite3* sql = NULL;
	std::mutex mutex_db_commit;
	int id_trip = 0;
	bool airborne = FALSE;
	struct FLIGHT_DATA* touchdown_data = NULL;
	struct FLIGHT_DATA* touchdown_data_end = NULL;
	struct FLIGHT_DATA data;
	struct AIRPORT departure;
	struct AIRPORT destination;
};

// Helper Functions
std::string trim(
	const std::string& source
) {
	std::string s(source);
	s.erase(0, s.find_first_not_of(" \n\r\t"));
	s.erase(s.find_last_not_of(" \n\r\t") + 1);
	return s;
}

void format_date_time(
	char* out,
	int len_out,
	int year,
	int month_of_year,
	int day_of_month,
	double time_day,
	double timezone_offset,
	int day_of_week
) {
	if (len_out < 32) {
		printf("ERROR [format_date_time]: size of out buffer (%d) is less than 32.\n", len_out);
		exit(1);
	}
	int hour = (int)time_day / 3600;
	int minute = ((int)time_day - 3600 * hour) / 60;
	double second = time_day - 3600 * hour - 60 * minute;
	timezone_offset /= 3600;
	int timezone_hour = (int)timezone_offset;
	int timezone_minute = (int)((timezone_offset - timezone_hour) * 60);
	char sign = '+';
	if (timezone_offset < 0) {
		sign = '-';
		timezone_hour *= -1;
	}
	sprintf_s(out, len_out, "%04d-%02d-%02dT%02d:%02d:%06.3f%c%02d:%02d_%d", year, month_of_year, day_of_month, hour, minute, second, sign, timezone_hour, timezone_minute, day_of_week);
}

void format_date_time(
	char* out,
	int len_out,
	struct DATETIME datetime
) {
	return format_date_time(out, len_out, datetime.year, datetime.month_of_year, datetime.day_of_month, datetime.time_day, datetime.timezone_offset, datetime.day_of_week);
}

void coordinate_decimal_to_dms(
	char* out,
	int len_out,
	double coordinate,
	enum COORDINATE_CAT cat
) {
	char tmp1 = 'T';
	switch (cat) {
	case LATITUDE:
		if (coordinate < 0)
			tmp1 = 'S';
		else
			tmp1 = 'N';
		break;
	case LONGITUDE:
		if (coordinate < 0)
			tmp1 = 'W';
		else
			tmp1 = 'E';
		break;
	default:
		break;
	}
	coordinate = abs(coordinate);
	int degree = coordinate;
	coordinate -= degree;
	coordinate *= 60;
	int minute = coordinate;
	coordinate -= minute;
	coordinate *= 60;
	int second = coordinate;
	if (len_out < 12) {
		printf("ERROR [coordinate_decimal_to_dms]: size of out buffer (%d) is less than 12.\n", len_out);
		exit(1);
	}
	sprintf_s(out, len_out, "%03d %02d %02d%c", degree, minute, second, tmp1);
}

double distanceInKmBetweenEarthCoordinates(
	struct COORDINATE loc1,
	struct COORDINATE loc2
) {
	double phy1 = loc1.latitude * V_PI / 180.0;
	double phy2 = loc2.latitude * V_PI / 180.0;
	double dPhy = (loc2.latitude - loc1.latitude) * V_PI / 180.0;
	double dLambda = (loc2.longitude - loc1.longitude) * V_PI / 180.0;
	double a = pow(sin(dPhy / 2), 2) + cos(phy1) * cos(phy2) * pow(sin(dLambda / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return EARTHRADIUSKM * c;
}

double bearingBetweenEarchCoordinates(
	struct COORDINATE loc1,
	struct COORDINATE loc2
) {
	double phy1 = loc1.latitude * V_PI / 180.0;
	double phy2 = loc2.latitude * V_PI / 180.0;
	double lambda1 = loc1.longitude * V_PI / 180.0;
	double lambda2 = loc2.longitude * V_PI / 180.0;
	double y = sin(lambda2 - lambda1) * cos(phy2);
	double x = cos(phy1) * sin(phy2) - sin(phy1) * cos(phy2) * cos(lambda2 - lambda1);
	double theta = atan2(y, x);
	double ret = theta * 180 / V_PI;
	if (ret < 0)
		ret += 360;
	return ret;
}

struct COORDINATE destinationWithDistanceAndBearing(
	struct COORDINATE start,
	double distance,
	double bearing
) {
	struct COORDINATE ret;
	double phy = start.latitude * V_PI / 180.0;
	double lambda = start.longitude * V_PI / 180.0;
	double theta = bearing * V_PI / 180.0;
	ret.latitude = asin(sin(phy) * cos(distance / EARTHRADIUSKM) + cos(phy) * sin(distance / EARTHRADIUSKM) * cos(theta));
	ret.longitude = lambda + atan2(sin(theta) * sin(distance / EARTHRADIUSKM) * cos(phy), cos(distance / EARTHRADIUSKM) - sin(phy) * sin(phy));
	ret.latitude *= 180.0 / V_PI;
	ret.longitude *= 180.0 / V_PI;
	return ret;
}

void runway_code_generator(char* out, int len_out, struct AIRPORT airport) {
	if (len_out < 4) {
		printf("ERROR [runway_code_generator]: size of out buffer (%d) is less than 4.\n", len_out);
		exit(1);
	}
	int runway_number = airport.runway_act_primary ? airport.runways[airport.runway_act_index].primary_number : airport.runways[airport.runway_act_index].secondary_number;
	int runway_designator = airport.runway_act_primary ? airport.runways[airport.runway_act_index].primary_designator : airport.runways[airport.runway_act_index].secondary_designator;
	sprintf_s(out, len_out, "%02d", runway_number);
	switch (runway_designator) {
	case 1:
		out[2] = 'L';
		break;
	case 2:
		out[2] = 'R';
		break;
	case 3:
		out[2] = 'C';
		break;
	case 4:
		out[2] = 'W';
		break;
	case 5:
		out[2] = 'A';
		break;
	case 6:
		out[2] = 'B';
		break;
	default:
		break;
	}
}

void db_error(
	const char* stmt_txt,
	int sql_ret,
	char** errmsg
) {
	if (sql_ret != 0)
		printf("db operation \"%s\" failed with error %d\n", stmt_txt, sql_ret);
	if (errmsg != NULL)
		printf("db operation \"%s\" failed with error %s\n", stmt_txt, *errmsg);
	exit(3);
}

void db_bind(
	sqlite3_stmt* stmt,
	const char* stmt_txt,
	int index,
	int value
) {
	int sql_ret = 0;
	sql_ret = sqlite3_bind_int(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(
	sqlite3_stmt* stmt,
	const char* stmt_txt,
	int index,
	double value
) {
	int sql_ret = 0;
	sql_ret = sqlite3_bind_double(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(
	sqlite3_stmt* stmt,
	const char* stmt_txt,
	int index,
	char* value
) {
	int sql_ret = 0;
	sql_ret = sqlite3_bind_text(stmt, index, value, strlen(value), SQLITE_TRANSIENT);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(
	sqlite3_stmt* stmt,
	const char* stmt_txt,
	int index,
	const char* value
) {
	int sql_ret = 0;
	sql_ret = sqlite3_bind_text(stmt, index, value, strlen(value), SQLITE_TRANSIENT);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_query_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux_in,
	void* aux_out,
	void (*func_set_stmt)(
		sqlite3_stmt* stmt,
		const char* stmt_txt,
		void* data,
		struct STATUS* status,
		void* aux
		),
	void (*func_retrieve_data)(
		sqlite3_stmt* stmt,
		const char* stmt_txt,
		struct STATUS* status,
		void* aux
		)
) {
	sqlite3_stmt* stmt = NULL;
	int sql_ret = 0;
	sql_ret = sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, NULL);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
	if (func_set_stmt != NULL)
		func_set_stmt(stmt, stmt_txt, data, status, aux_in);
	while (sqlite3_step(stmt) == SQLITE_ROW)
		func_retrieve_data(stmt, stmt_txt, status, aux_out);
	sqlite3_reset(stmt);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
	sqlite3_finalize(stmt);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_insert_update_table(sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux,
	void (*func)(
		sqlite3_stmt* stmt,
		const char* stmt_txt,
		void* data,
		struct STATUS* status,
		void* aux)
) {
	status->mutex_db_commit.lock();
	sqlite3_stmt* stmt = NULL;
	int sql_ret = 0;
	char* errmsg = NULL;
	sql_ret = sqlite3_exec(sql, "BEGIN TRANSACTION", NULL, NULL, &errmsg);
	if (errmsg != NULL)
		db_error(stmt_txt, 0, &errmsg);
	sql_ret = sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, NULL);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
	func(stmt, stmt_txt, data, status, aux);
	sql_ret = sqlite3_step(stmt);
	sql_ret = sqlite3_reset(stmt);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
	sql_ret = sqlite3_exec(sql, "COMMIT TRANSACTION", NULL, NULL, &errmsg);
	if (errmsg != NULL)
		db_error(stmt_txt, 0, &errmsg);
	sql_ret = sqlite3_finalize(stmt);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
	status->mutex_db_commit.unlock();
}

// Insert data to database
void db_consume(
	STATUS* status
) {
	while (status->q_data_db_start != NULL) {
		struct DATA_A320* pS = status->q_data_db_start;
		db_insert_update_table(status->sql,
			"INSERT INTO trip_data ("
			"trip,"
			"bool_group_1,"
			"bool_group_2,"
			"bool_group_3,"
			"eng_exhaust_gas_temperature_1,"
			"eng_exhaust_gas_temperature_2,"
			"eng_oil_temperature_1,"
			"eng_oil_temperature_2,"
			"ambient_temperature,"
			"autopilot_heading_lock_dir,"
			"aileron_left_deflection,"
			"aileron_right_deflection,"
			"aileron_trim,"
			"elevator_deflection,"
			"elevator_trim_position,"
			"elevon_deflection,"
			"rudder_deflection,"
			"rudder_trim,"
			"plane_bank_degrees,"
			"plane_heading_degrees_gyro,"
			"plane_heading_degrees_magnetic,"
			"plane_heading_degrees_true,"
			"plane_latitude,"
			"plane_longitude,"
			"plane_pitch_degrees,"
			"plane_touchdown_bank_degrees,"
			"plane_touchdown_heading_degrees_magnetic,"
			"plane_touchdown_heading_degrees_true,"
			"plane_touchdown_latitude,"
			"plane_touchdown_longitude,"
			"plane_touchdown_pitch_degrees,"
			"gps_ground_true_heading,"
			"gps_ground_true_track,"
			"gps_position_lat,"
			"gps_position_lon,"
			"gyro_drift_error,"
			"heading_indicator,"
			"magnetic_compass,"
			"ambient_wind_direction,"
			"gear_position_0,"
			"gear_position_1,"
			"gear_position_2,"
			"gear_warning_0,"
			"gear_warning_1,"
			"gear_warning_2,"
			"bleed_air_source_control_1,"
			"bleed_air_source_control_2,"
			"engine_type,"
			"turb_eng_ignition_switch_ex1_1,"
			"turb_eng_ignition_switch_ex1_2,"
			"fuel_cross_feed_l,"
			"fuel_cross_feed_r,"
			"surface_condition,"
			"surface_type,"
			"pitot_heat_switch,"
			"plane_touchdown_normal_velocity,"
			"vertical_speed,"
			"autopilot_altitude_lock_var,"
			"plane_altitude,"
			"plane_alt_above_ground,"
			"radio_height,"
			"indicated_altitude,"
			"indicated_altitude_calibrated,"
			"pressurization_cabin_altitude,"
			"autopilot_vertical_hold_var,"
			"engine_control_select,"
			"fuel_selected_quantity_l,"
			"fuel_selected_quantity_r,"
			"fuel_total_quantity,"
			"g_force,"
			"general_eng_elapsed_time_1,"
			"general_eng_elapsed_time_2,"
			"ambient_pressure,"
			"kohlsman_setting_hg,"
			"autopilot_airspeed_hold_var,"
			"ground_velocity,"
			"airspeed_indicated,"
			"airspeed_true,"
			"ambient_wind_velocity,"
			"airspeed_mach,"
			"light_states,"
			"gps_ground_speed,"
			"gps_position_alt,"
			"pressure_altitude,"
			"ambient_visibility,"
			"barometer_pressure,"
			"kohlsman_setting_mb,"
			"autopilot_mach_hold_var,"
			"auto_brake_switch_cb,"
			"flaps_handle_index,"
			"flaps_num_handle_positions,"
			"general_eng_throttle_managed_mode_1,"
			"general_eng_throttle_managed_mode_2,"
			"number_of_engines,"
			"turb_eng_vibration_1,"
			"turb_eng_vibration_2,"
			"gear_handle_position,"
			"aileron_left_deflection_pct,"
			"aileron_right_deflection_pct,"
			"aileron_trim_pct,"
			"elevator_deflection_pct,"
			"elevator_trim_pct,"
			"rudder_deflection_pct,"
			"rudder_trim_pct,"
			"spoilers_handle_position,"
			"spoilers_left_position,"
			"spoilers_right_position,"
			"apu_pct_rpm,"
			"apu_pct_starter,"
			"fuel_selected_quantity_percent_l,"
			"fuel_selected_quantity_percent_r,"
			"pitot_ice_pct,"
			"autopilot_throttle_max_thrust,"
			"electrical_battery_estimated_capacity_pct,"
			"general_eng_damage_percent_1,"
			"general_eng_damage_percent_2,"
			"general_eng_throttle_lever_position_1,"
			"general_eng_throttle_lever_position_2,"
			"turb_eng_n1_1,"
			"turb_eng_n1_2,"
			"turb_eng_n2_1,"
			"turb_eng_n2_2,"
			"brake_indicator,"
			"turb_eng_fuel_flow_pph_1,"
			"turb_eng_fuel_flow_pph_2,"
			"general_eng_fuel_used_since_start_1,"
			"general_eng_fuel_used_since_start_2,"
			"empty_weight,"
			"total_weight,"
			"fuel_total_quantity_weight,"
			"fuel_weight_per_gallon,"
			"eng_hydraulic_pressure_1,"
			"eng_hydraulic_pressure_2,"
			"eng_oil_pressure_1,"
			"eng_oil_pressure_2,"
			"hydraulic_pressure_1,"
			"hydraulic_pressure_2,"
			"apu_bleed_pressure_received_by_engine,"
			"turb_eng_bleed_air_1,"
			"turb_eng_bleed_air_2,"
			"wheel_rpm_0,"
			"wheel_rpm_1,"
			"wheel_rpm_2,"
			"electrical_battery_voltage,"
			"zulu_time,"
			"local_time"
			") VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);",
			pS,
			status,
			NULL,
			[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
				struct DATA_A320* pS = (struct DATA_A320*)data;
				int bool_group_1 = 0;
				int bool_group_2 = 0;
				int bool_group_3 = 0;
				if (pS->autopilot_airspeed_hold)
					bool_group_1 |= (0x1 << 0);
				if (pS->autopilot_alt_radio_mode)
					bool_group_1 |= (0x1 << 1);
				if (pS->autopilot_altitude_lock)
					bool_group_1 |= (0x1 << 2);
				if (pS->autopilot_approach_active)
					bool_group_1 |= (0x1 << 3);
				if (pS->autopilot_approach_captured)
					bool_group_1 |= (0x1 << 4);
				if (pS->autopilot_approach_hold)
					bool_group_1 |= (0x1 << 5);
				if (pS->autopilot_approach_is_localizer)
					bool_group_1 |= (0x1 << 6);
				if (pS->autopilot_avionics_managed)
					bool_group_1 |= (0x1 << 7);
				if (pS->autopilot_disengaged)
					bool_group_1 |= (0x1 << 8);
				if (pS->autopilot_flight_director_active)
					bool_group_1 |= (0x1 << 9);
				if (pS->autopilot_flight_level_change)
					bool_group_1 |= (0x1 << 10);
				if (pS->autopilot_glideslope_active)
					bool_group_1 |= (0x1 << 11);
				if (pS->autopilot_glideslope_arm)
					bool_group_1 |= (0x1 << 12);
				if (pS->autopilot_glideslope_hold)
					bool_group_1 |= (0x1 << 13);
				if (pS->autopilot_heading_lock)
					bool_group_1 |= (0x1 << 14);
				if (pS->autopilot_mach_hold)
					bool_group_1 |= (0x1 << 15);
				if (pS->autopilot_managed_speed_in_mach)
					bool_group_1 |= (0x1 << 16);
				if (pS->autopilot_managed_throttle_active)
					bool_group_1 |= (0x1 << 17);
				if (pS->autopilot_master)
					bool_group_1 |= (0x1 << 18);
				if (pS->autopilot_takeoff_power_active)
					bool_group_1 |= (0x1 << 19);
				if (pS->autopilot_throttle_arm)
					bool_group_1 |= (0x1 << 20);
				if (pS->autopilot_vertical_hold)
					bool_group_1 |= (0x1 << 21);
				if (pS->autobrakes_active)
					bool_group_1 |= (0x1 << 22);
				if (pS->brake_parking_indicator)
					bool_group_1 |= (0x1 << 23);
				if (pS->rejected_takeoff_brakes_active)
					bool_group_1 |= (0x1 << 24);
				if (pS->gear_damage_by_speed)
					bool_group_1 |= (0x1 << 25);
				if (pS->gear_is_on_ground_0)
					bool_group_1 |= (0x1 << 26);
				if (pS->gear_is_on_ground_1)
					bool_group_1 |= (0x1 << 27);
				if (pS->gear_is_on_ground_2)
					bool_group_1 |= (0x1 << 28);
				if (pS->gear_speed_exceeded)
					bool_group_1 |= (0x1 << 29);
				if (pS->aileron_trim_disabled)
					bool_group_1 |= (0x1 << 30);
				if (pS->elevator_trim_disabled)
					bool_group_1 |= (0x1 << 31);
				if (pS->flap_damage_by_speed)
					bool_group_2 |= (0x1 << 0);
				if (pS->flap_speed_exceeded)
					bool_group_2 |= (0x1 << 1);
				if (pS->rudder_trim_disabled)
					bool_group_2 |= (0x1 << 2);
				if (pS->spoilers_armed)
					bool_group_2 |= (0x1 << 3);
				if (pS->apu_generator_active)
					bool_group_2 |= (0x1 << 4);
				if (pS->apu_generator_switch)
					bool_group_2 |= (0x1 << 5);
				if (pS->apu_on_fire_detected)
					bool_group_2 |= (0x1 << 6);
				if (pS->apu_switch)
					bool_group_2 |= (0x1 << 7);
				if (pS->bleed_air_apu)
					bool_group_2 |= (0x1 << 8);
				if (pS->electrical_master_battery)
					bool_group_2 |= (0x1 << 9);
				if (pS->external_power_available)
					bool_group_2 |= (0x1 << 10);
				if (pS->external_power_connection_on)
					bool_group_2 |= (0x1 << 11);
				if (pS->external_power_on)
					bool_group_2 |= (0x1 << 12);
				if (pS->bleed_air_engine_1)
					bool_group_2 |= (0x1 << 13);
				if (pS->bleed_air_engine_2)
					bool_group_2 |= (0x1 << 14);
				if (pS->eng_anti_ice_1)
					bool_group_2 |= (0x1 << 15);
				if (pS->eng_anti_ice_2)
					bool_group_2 |= (0x1 << 16);
				if (pS->eng_combustion_1)
					bool_group_2 |= (0x1 << 17);
				if (pS->eng_combustion_2)
					bool_group_2 |= (0x1 << 18);
				if (pS->eng_failed_1)
					bool_group_2 |= (0x1 << 19);
				if (pS->eng_failed_2)
					bool_group_2 |= (0x1 << 20);
				if (pS->eng_on_fire_1)
					bool_group_2 |= (0x1 << 21);
				if (pS->eng_on_fire_2)
					bool_group_2 |= (0x1 << 22);
				if (pS->general_eng_fire_detected_1)
					bool_group_2 |= (0x1 << 23);
				if (pS->general_eng_fire_detected_2)
					bool_group_2 |= (0x1 << 24);
				if (pS->general_eng_fuel_valve_1)
					bool_group_2 |= (0x1 << 25);
				if (pS->general_eng_fuel_valve_2)
					bool_group_2 |= (0x1 << 26);
				if (pS->general_eng_generator_active_1)
					bool_group_2 |= (0x1 << 27);
				if (pS->general_eng_generator_active_2)
					bool_group_2 |= (0x1 << 28);
				if (pS->general_eng_generator_switch_1)
					bool_group_2 |= (0x1 << 29);
				if (pS->general_eng_generator_switch_2)
					bool_group_2 |= (0x1 << 30);
				if (pS->general_eng_master_alternator)
					bool_group_2 |= (0x1 << 31);
				if (pS->general_eng_reverse_thrust_engaged)
					bool_group_3 |= (0x1 << 0);
				if (pS->general_eng_starter_1)
					bool_group_3 |= (0x1 << 1);
				if (pS->general_eng_starter_2)
					bool_group_3 |= (0x1 << 2);
				if (pS->general_eng_starter_active_1)
					bool_group_3 |= (0x1 << 3);
				if (pS->general_eng_starter_active_2)
					bool_group_3 |= (0x1 << 4);
				if (pS->master_ignition_switch)
					bool_group_3 |= (0x1 << 5);
				if (pS->turb_eng_fuel_available_1)
					bool_group_3 |= (0x1 << 6);
				if (pS->turb_eng_fuel_available_2)
					bool_group_3 |= (0x1 << 7);
				if (pS->turb_eng_is_igniting_1)
					bool_group_3 |= (0x1 << 8);
				if (pS->turb_eng_is_igniting_2)
					bool_group_3 |= (0x1 << 9);
				if (pS->fuel_transfer_pump_on_l)
					bool_group_3 |= (0x1 << 10);
				if (pS->fuel_transfer_pump_on_r)
					bool_group_3 |= (0x1 << 11);
				if (pS->on_any_runway)
					bool_group_3 |= (0x1 << 12);
				if (pS->plane_in_parking_state)
					bool_group_3 |= (0x1 << 13);
				if (pS->autothrottle_active)
					bool_group_3 |= (0x1 << 14);
				if (pS->avionics_master_switch)
					bool_group_3 |= (0x1 << 15);
				if (pS->cabin_no_smoking_alert_switch)
					bool_group_3 |= (0x1 << 16);
				if (pS->cabin_seatbelts_alert_switch)
					bool_group_3 |= (0x1 << 17);
				if (pS->gpws_system_active)
					bool_group_3 |= (0x1 << 18);
				if (pS->gpws_warning)
					bool_group_3 |= (0x1 << 19);
				if (pS->overspeed_warning)
					bool_group_3 |= (0x1 << 20);
				if (pS->pitot_heat)
					bool_group_3 |= (0x1 << 21);
				if (pS->stall_warning)
					bool_group_3 |= (0x1 << 22);
				if (pS->structural_deice_switch)
					bool_group_3 |= (0x1 << 23);
				if (pS->hydraulic_switch)
					bool_group_3 |= (0x1 << 24);
				if (pS->warning_fuel)
					bool_group_3 |= (0x1 << 25);
				if (pS->warning_low_height)
					bool_group_3 |= (0x1 << 26);
				if (pS->warning_oil_pressure)
					bool_group_3 |= (0x1 << 27);
				if (pS->warning_vacuum)
					bool_group_3 |= (0x1 << 28);
				if (pS->warning_voltage)
					bool_group_3 |= (0x1 << 29);
				if (pS->sim_on_ground)
					bool_group_3 |= (0x1 << 30);
				if (pS->kohlsman_setting_std)
					bool_group_3 |= (0x1 << 31);
				char time_zulu[32];
				char time_local[32];
				memset(time_zulu, 0, sizeof(time_zulu));
				memset(time_local, 0, sizeof(time_local));
				format_date_time(time_zulu, sizeof(time_zulu), pS->zulu_year, pS->zulu_month_of_year, pS->zulu_day_of_month, pS->zulu_time, 0, pS->zulu_day_of_week);
				format_date_time(time_local, sizeof(time_local), pS->local_year, pS->local_month_of_year, pS->local_day_of_month, pS->local_time, pS->time_zone_offset, pS->local_day_of_week);

				db_bind(stmt, stmt_txt, 1, status->id_trip);
				db_bind(stmt, stmt_txt, 2, bool_group_1);
				db_bind(stmt, stmt_txt, 3, bool_group_2);
				db_bind(stmt, stmt_txt, 4, bool_group_3);
				db_bind(stmt, stmt_txt, 5, pS->eng_exhaust_gas_temperature_1);
				db_bind(stmt, stmt_txt, 6, pS->eng_exhaust_gas_temperature_2);
				db_bind(stmt, stmt_txt, 7, pS->eng_oil_temperature_1);
				db_bind(stmt, stmt_txt, 8, pS->eng_oil_temperature_2);
				db_bind(stmt, stmt_txt, 9, pS->ambient_temperature);
				db_bind(stmt, stmt_txt, 10, pS->autopilot_heading_lock_dir);
				db_bind(stmt, stmt_txt, 11, pS->aileron_left_deflection);
				db_bind(stmt, stmt_txt, 12, pS->aileron_right_deflection);
				db_bind(stmt, stmt_txt, 13, pS->aileron_trim);
				db_bind(stmt, stmt_txt, 14, pS->elevator_deflection);
				db_bind(stmt, stmt_txt, 15, pS->elevator_trim_position);
				db_bind(stmt, stmt_txt, 16, pS->elevon_deflection);
				db_bind(stmt, stmt_txt, 17, pS->rudder_deflection);
				db_bind(stmt, stmt_txt, 18, pS->rudder_trim);
				db_bind(stmt, stmt_txt, 19, pS->plane_bank_degrees);
				db_bind(stmt, stmt_txt, 20, pS->plane_heading_degrees_gyro);
				db_bind(stmt, stmt_txt, 21, pS->plane_heading_degrees_magnetic);
				db_bind(stmt, stmt_txt, 22, pS->plane_heading_degrees_true);
				db_bind(stmt, stmt_txt, 23, pS->plane_latitude);
				db_bind(stmt, stmt_txt, 24, pS->plane_longitude);
				db_bind(stmt, stmt_txt, 25, pS->plane_pitch_degrees);
				db_bind(stmt, stmt_txt, 26, pS->plane_touchdown_bank_degrees);
				db_bind(stmt, stmt_txt, 27, pS->plane_touchdown_heading_degrees_magnetic);
				db_bind(stmt, stmt_txt, 28, pS->plane_touchdown_heading_degrees_true);
				db_bind(stmt, stmt_txt, 29, pS->plane_touchdown_latitude);
				db_bind(stmt, stmt_txt, 30, pS->plane_touchdown_longitude);
				db_bind(stmt, stmt_txt, 31, pS->plane_touchdown_pitch_degrees);
				db_bind(stmt, stmt_txt, 32, pS->gps_ground_true_heading);
				db_bind(stmt, stmt_txt, 33, pS->gps_ground_true_track);
				db_bind(stmt, stmt_txt, 34, pS->gps_position_lat);
				db_bind(stmt, stmt_txt, 35, pS->gps_position_lon);
				db_bind(stmt, stmt_txt, 36, pS->gyro_drift_error);
				db_bind(stmt, stmt_txt, 37, pS->heading_indicator);
				db_bind(stmt, stmt_txt, 38, pS->magnetic_compass);
				db_bind(stmt, stmt_txt, 39, pS->ambient_wind_direction);
				db_bind(stmt, stmt_txt, 40, pS->gear_position_0);
				db_bind(stmt, stmt_txt, 41, pS->gear_position_1);
				db_bind(stmt, stmt_txt, 42, pS->gear_position_2);
				db_bind(stmt, stmt_txt, 43, pS->gear_warning_0);
				db_bind(stmt, stmt_txt, 44, pS->gear_warning_1);
				db_bind(stmt, stmt_txt, 45, pS->gear_warning_2);
				db_bind(stmt, stmt_txt, 46, pS->bleed_air_source_control_1);
				db_bind(stmt, stmt_txt, 47, pS->bleed_air_source_control_2);
				db_bind(stmt, stmt_txt, 48, pS->engine_type);
				db_bind(stmt, stmt_txt, 49, pS->turb_eng_ignition_switch_ex1_1);
				db_bind(stmt, stmt_txt, 50, pS->turb_eng_ignition_switch_ex1_2);
				db_bind(stmt, stmt_txt, 51, pS->fuel_cross_feed_l);
				db_bind(stmt, stmt_txt, 52, pS->fuel_cross_feed_r);
				db_bind(stmt, stmt_txt, 53, pS->surface_condition);
				db_bind(stmt, stmt_txt, 54, pS->surface_type);
				db_bind(stmt, stmt_txt, 55, pS->pitot_heat_switch);
				db_bind(stmt, stmt_txt, 56, pS->plane_touchdown_normal_velocity);
				db_bind(stmt, stmt_txt, 57, pS->vertical_speed);
				db_bind(stmt, stmt_txt, 58, pS->autopilot_altitude_lock_var);
				db_bind(stmt, stmt_txt, 59, pS->plane_altitude);
				db_bind(stmt, stmt_txt, 60, pS->plane_alt_above_ground);
				db_bind(stmt, stmt_txt, 61, pS->radio_height);
				db_bind(stmt, stmt_txt, 62, pS->indicated_altitude);
				db_bind(stmt, stmt_txt, 63, pS->indicated_altitude_calibrated);
				db_bind(stmt, stmt_txt, 64, pS->pressurization_cabin_altitude);
				db_bind(stmt, stmt_txt, 65, pS->autopilot_vertical_hold_var);
				db_bind(stmt, stmt_txt, 66, pS->engine_control_select);
				db_bind(stmt, stmt_txt, 67, pS->fuel_selected_quantity_l);
				db_bind(stmt, stmt_txt, 68, pS->fuel_selected_quantity_r);
				db_bind(stmt, stmt_txt, 69, pS->fuel_total_quantity);
				db_bind(stmt, stmt_txt, 70, pS->g_force);
				db_bind(stmt, stmt_txt, 71, pS->general_eng_elapsed_time_1);
				db_bind(stmt, stmt_txt, 72, pS->general_eng_elapsed_time_2);
				db_bind(stmt, stmt_txt, 73, pS->ambient_pressure);
				db_bind(stmt, stmt_txt, 74, pS->kohlsman_setting_hg);
				db_bind(stmt, stmt_txt, 75, pS->autopilot_airspeed_hold_var);
				db_bind(stmt, stmt_txt, 76, pS->ground_velocity);
				db_bind(stmt, stmt_txt, 77, pS->airspeed_indicated);
				db_bind(stmt, stmt_txt, 78, pS->airspeed_true);
				db_bind(stmt, stmt_txt, 79, pS->ambient_wind_velocity);
				db_bind(stmt, stmt_txt, 80, pS->airspeed_mach);
				db_bind(stmt, stmt_txt, 81, pS->light_states);
				db_bind(stmt, stmt_txt, 82, pS->gps_ground_speed);
				db_bind(stmt, stmt_txt, 83, pS->gps_position_alt);
				db_bind(stmt, stmt_txt, 84, pS->pressure_altitude);
				db_bind(stmt, stmt_txt, 85, pS->ambient_visibility);
				db_bind(stmt, stmt_txt, 86, pS->barometer_pressure);
				db_bind(stmt, stmt_txt, 87, pS->kohlsman_setting_mb);
				db_bind(stmt, stmt_txt, 88, pS->autopilot_mach_hold_var);
				db_bind(stmt, stmt_txt, 89, pS->auto_brake_switch_cb);
				db_bind(stmt, stmt_txt, 90, pS->flaps_handle_index);
				db_bind(stmt, stmt_txt, 91, pS->flaps_num_handle_positions);
				db_bind(stmt, stmt_txt, 92, pS->general_eng_throttle_managed_mode_1);
				db_bind(stmt, stmt_txt, 93, pS->general_eng_throttle_managed_mode_2);
				db_bind(stmt, stmt_txt, 94, pS->number_of_engines);
				db_bind(stmt, stmt_txt, 95, pS->turb_eng_vibration_1);
				db_bind(stmt, stmt_txt, 96, pS->turb_eng_vibration_2);
				db_bind(stmt, stmt_txt, 97, pS->gear_handle_position);
				db_bind(stmt, stmt_txt, 98, pS->aileron_left_deflection_pct);
				db_bind(stmt, stmt_txt, 99, pS->aileron_right_deflection_pct);
				db_bind(stmt, stmt_txt, 100, pS->aileron_trim_pct);
				db_bind(stmt, stmt_txt, 101, pS->elevator_deflection_pct);
				db_bind(stmt, stmt_txt, 102, pS->elevator_trim_pct);
				db_bind(stmt, stmt_txt, 103, pS->rudder_deflection_pct);
				db_bind(stmt, stmt_txt, 104, pS->rudder_trim_pct);
				db_bind(stmt, stmt_txt, 105, pS->spoilers_handle_position);
				db_bind(stmt, stmt_txt, 106, pS->spoilers_left_position);
				db_bind(stmt, stmt_txt, 107, pS->spoilers_right_position);
				db_bind(stmt, stmt_txt, 108, pS->apu_pct_rpm);
				db_bind(stmt, stmt_txt, 109, pS->apu_pct_starter);
				db_bind(stmt, stmt_txt, 110, pS->fuel_selected_quantity_percent_l);
				db_bind(stmt, stmt_txt, 111, pS->fuel_selected_quantity_percent_r);
				db_bind(stmt, stmt_txt, 112, pS->pitot_ice_pct);
				db_bind(stmt, stmt_txt, 113, pS->autopilot_throttle_max_thrust);
				db_bind(stmt, stmt_txt, 114, pS->electrical_battery_estimated_capacity_pct);
				db_bind(stmt, stmt_txt, 115, pS->general_eng_damage_percent_1);
				db_bind(stmt, stmt_txt, 116, pS->general_eng_damage_percent_2);
				db_bind(stmt, stmt_txt, 117, pS->general_eng_throttle_lever_position_1);
				db_bind(stmt, stmt_txt, 118, pS->general_eng_throttle_lever_position_2);
				db_bind(stmt, stmt_txt, 119, pS->turb_eng_n1_1);
				db_bind(stmt, stmt_txt, 120, pS->turb_eng_n1_2);
				db_bind(stmt, stmt_txt, 121, pS->turb_eng_n2_1);
				db_bind(stmt, stmt_txt, 122, pS->turb_eng_n2_2);
				db_bind(stmt, stmt_txt, 123, pS->brake_indicator);
				db_bind(stmt, stmt_txt, 124, pS->turb_eng_fuel_flow_pph_1);
				db_bind(stmt, stmt_txt, 125, pS->turb_eng_fuel_flow_pph_2);
				db_bind(stmt, stmt_txt, 126, pS->general_eng_fuel_used_since_start_1);
				db_bind(stmt, stmt_txt, 127, pS->general_eng_fuel_used_since_start_2);
				db_bind(stmt, stmt_txt, 128, pS->empty_weight);
				db_bind(stmt, stmt_txt, 129, pS->total_weight);
				db_bind(stmt, stmt_txt, 130, pS->fuel_total_quantity_weight);
				db_bind(stmt, stmt_txt, 131, pS->fuel_weight_per_gallon);
				db_bind(stmt, stmt_txt, 132, pS->eng_hydraulic_pressure_1);
				db_bind(stmt, stmt_txt, 133, pS->eng_hydraulic_pressure_2);
				db_bind(stmt, stmt_txt, 134, pS->eng_oil_pressure_1);
				db_bind(stmt, stmt_txt, 135, pS->eng_oil_pressure_2);
				db_bind(stmt, stmt_txt, 136, pS->hydraulic_pressure_1);
				db_bind(stmt, stmt_txt, 137, pS->hydraulic_pressure_2);
				db_bind(stmt, stmt_txt, 138, pS->apu_bleed_pressure_received_by_engine);
				db_bind(stmt, stmt_txt, 139, pS->turb_eng_bleed_air_1);
				db_bind(stmt, stmt_txt, 140, pS->turb_eng_bleed_air_2);
				db_bind(stmt, stmt_txt, 141, pS->wheel_rpm_0);
				db_bind(stmt, stmt_txt, 142, pS->wheel_rpm_1);
				db_bind(stmt, stmt_txt, 143, pS->wheel_rpm_2);
				db_bind(stmt, stmt_txt, 144, pS->electrical_battery_voltage);
				db_bind(stmt, stmt_txt, 145, time_zulu);
				db_bind(stmt, stmt_txt, 146, time_local);
			});
		bool fBreak = FALSE;
		if (status->q_data_db_start == status->q_data_db_end)
			fBreak = TRUE;
		status->q_data_db_start = status->q_data_db_start->next;
		if (status->q_data_db_start == NULL) {
			if (status->q_data_last != NULL)
				free(status->q_data_last);
			status->q_data_last = pS;
		}
		else
			free(pS);
		if (fBreak)
			break;
	}
	if (status->q_data_db_start == NULL) {
		status->q_data_end = NULL;
		status->q_data_db_end = NULL;
	}

	int count = 0;
	while (status->q_event_start != NULL) {
		struct EVENT_DB* pS = status->q_event_start;
		db_insert_update_table(
			status->sql,
			"INSERT INTO trip_events ("
			"trip,"
			"event,"
			"time_zulu,"
			"time_local"
			") VALUES (?,?,?,?)",
			pS,
			status,
			NULL,
			[](
				sqlite3_stmt* stmt,
				const char* stmt_txt,
				void* data,
				struct STATUS* status,
				void* aux
				) {
					struct EVENT_DB* pS = (struct EVENT_DB*)data;
					char time_zulu[32];
					char time_local[32];
					memset(time_zulu, 0, sizeof(time_zulu));
					memset(time_local, 0, sizeof(time_local));
					format_date_time(time_zulu, sizeof(time_zulu), pS->time_zulu.year, pS->time_zulu.month_of_year, pS->time_zulu.day_of_month, pS->time_zulu.time_day, 0, pS->time_zulu.day_of_week);
					format_date_time(time_local, sizeof(time_local), pS->time_local.year, pS->time_local.month_of_year, pS->time_local.day_of_month, pS->time_local.time_day, pS->time_local.timezone_offset, pS->time_local.day_of_week);

					db_bind(stmt, stmt_txt, 1, status->id_trip);
					db_bind(stmt, stmt_txt, 2, pS->event);
					db_bind(stmt, stmt_txt, 3, time_zulu);
					db_bind(stmt, stmt_txt, 4, time_local);
			}
		);
		status->q_event_start = status->q_event_start->next;
		free(pS);
		if (status->recording && ++count == Q_DB_LENGTH)
			break;
	}
	if (status->q_event_start == NULL)
		status->q_event_end = NULL;
}

void stop_recording(
	struct STATUS* status
) {
	status->recording = FALSE;
	db_insert_update_table(
		status->sql,
		"UPDATE trips SET destination_latitude=?,destination_longitude=?,destination_zulu_time=?,destination_local_time=? WHERE id=?;",
		status->q_data_end == NULL ? status->q_data_last : status->q_data_end,
		status,
		NULL,
		[](
			sqlite3_stmt* stmt,
			const char* stmt_txt,
			void* data,
			struct STATUS* status,
			void* aux
			) {
				struct DATA_A320* pS = (struct DATA_A320*)data;
				char time_zulu[32];
				char time_local[32];
				memset(time_zulu, 0, sizeof(time_zulu));
				memset(time_local, 0, sizeof(time_local));
				format_date_time(time_zulu, sizeof(time_zulu), pS->zulu_year, pS->zulu_month_of_year, pS->zulu_day_of_month, pS->zulu_time, 0, pS->zulu_day_of_week);
				format_date_time(time_local, sizeof(time_local), pS->local_year, pS->local_month_of_year, pS->local_day_of_month, pS->local_time, pS->time_zone_offset, pS->local_day_of_week);
				db_bind(stmt, stmt_txt, 1, pS->plane_latitude);
				db_bind(stmt, stmt_txt, 2, pS->plane_longitude);
				db_bind(stmt, stmt_txt, 3, time_zulu);
				db_bind(stmt, stmt_txt, 4, time_local);
				db_bind(stmt, stmt_txt, 5, status->id_trip);
		}
	);
	if (status->touchdown_data != NULL) {
		//printf("\n=========================== LANDING SUMMARY ==========================\n");
		//printf("speed   v-speed g-force pitch   bank    heading   latitude   longitude\n");
		while (status->touchdown_data != NULL) {
			struct FLIGHT_DATA* cur = status->touchdown_data;
			db_insert_update_table(
				status->sql,
				"INSERT INTO trip_touchdowns ("
				"trip,"
				"airspeed_indicated,"
				"vertical_speed,"
				"g_force,"
				"plane_pitch_degrees,"
				"plane_bank_degrees,"
				"heading_indicator,"
				"plane_latitude,"
				"plane_longitude,"
				"icao,"
				"runway,"
				"time_zulu,"
				"time_local"
				") VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);",
				cur,
				status,
				NULL,
				[](
					sqlite3_stmt* stmt,
					const char* stmt_txt,
					void* data,
					struct STATUS* status,
					void* aux
					) {
						struct FLIGHT_DATA* pS = (struct FLIGHT_DATA*)data;
						char time_zulu[32];
						char time_local[32];
						memset(time_zulu, 0, sizeof(time_zulu));
						memset(time_local, 0, sizeof(time_local));
						format_date_time(time_zulu, sizeof(time_zulu), pS->time_zulu);
						format_date_time(time_local, sizeof(time_local), pS->time_local);

						db_bind(stmt, stmt_txt, 1, status->id_trip);
						db_bind(stmt, stmt_txt, 2, pS->speed);
						db_bind(stmt, stmt_txt, 3, pS->vertical_speed);
						db_bind(stmt, stmt_txt, 4, pS->g_force);
						db_bind(stmt, stmt_txt, 5, pS->pitch);
						db_bind(stmt, stmt_txt, 6, pS->bank);
						db_bind(stmt, stmt_txt, 7, pS->heading);
						db_bind(stmt, stmt_txt, 8, pS->coordinate.latitude);
						db_bind(stmt, stmt_txt, 9, pS->coordinate.longitude);
						db_bind(stmt, stmt_txt, 10, pS->icao);
						db_bind(stmt, stmt_txt, 11, pS->runway);
						db_bind(stmt, stmt_txt, 12, time_zulu);
						db_bind(stmt, stmt_txt, 13, time_local);
				}
			);
			status->touchdown_data = status->touchdown_data->next;
			free(cur);
		}
		//printf("======================================================================\n\n");
	}
	std::thread thd(db_consume, status);
	thd.join();
	if (status->q_data_last != NULL) {
		free(status->q_data_last);
		status->q_data_last = NULL;
	}
	status->id_trip = 0;
	memset(status->departure.icao, 0, sizeof(status->departure.icao));
	if (status->departure.runways != NULL)
		free(status->departure.runways);
	status->departure.runways = NULL;
	status->departure.extra_info.n_runways = 0;
	status->departure.runway_act_index = -1;
	status->departure.runway_act_primary = TRUE;
	memset(status->departure.extra_info.name, 0, sizeof(status->departure.extra_info.name));
	status->departure.extra_info.magvar = 0;
	memset(status->destination.icao, 0, sizeof(status->destination.icao));
	if (status->destination.runways != NULL)
		free(status->destination.runways);
	status->destination.runways = NULL;
	status->destination.extra_info.n_runways = 0;
	status->destination.runway_act_index = -1;
	status->destination.runway_act_primary = 0;
	memset(status->destination.extra_info.name, 0, sizeof(status->destination.extra_info.name));
	status->destination.extra_info.magvar = 0;
	printf("Recording Stopped\n");
}

void CALLBACK MyDispatchProc(
	SIMCONNECT_RECV* pData,
	DWORD cbData,
	void* pContext
) {
	struct STATUS* status = (struct STATUS*)pContext;
	switch (pData->dwID) {
	case SIMCONNECT_RECV_ID_OPEN:
	{
		SIMCONNECT_RECV_OPEN* openData = (SIMCONNECT_RECV_OPEN*)pData;
		printf("Connected to Microsoft Flight Simulator\n");
	}
	break;
	case SIMCONNECT_RECV_ID_QUIT:
		printf("SIMCONNECT_RECV_ID_QUIT\n");
		status->quit = TRUE;
		break;
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
			printf("Plane crashed!\n");
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
			if (status->id_trip > 0) {
				struct EVENT_DB* pS = (struct EVENT_DB*)malloc(sizeof(struct EVENT_DB));
				memset(pS, 0, sizeof(struct EVENT_DB));
				memcpy(pS->event, EVENT_ID_TXT[evt->uEventID], strlen(EVENT_ID_TXT[evt->uEventID]));
				pS->time_zulu = status->data.time_zulu;
				pS->time_local = status->data.time_local;
				if (status->q_event_end == NULL)
					status->q_event_end = pS;
				else {
					status->q_event_end->next = pS;
					status->q_event_end = pS;
				}
				if (status->q_event_start == NULL)
					status->q_event_start = pS;
			}
			break;
		default:
			printf("Unkown Event: %ld\n", evt->uEventID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
	{
		SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
		switch (pObjData->dwRequestID) {
		case REQUEST_A320:
		{
			struct DATA_A320* tmp = (struct DATA_A320*)&pObjData->dwData;
			status->data.altitude = tmp->plane_altitude;
			status->data.heading = tmp->plane_heading_degrees_magnetic;
			status->data.speed = tmp->airspeed_indicated;
			status->data.vertical_speed = tmp->vertical_speed;
			status->data.bank = tmp->plane_bank_degrees;
			status->data.pitch = tmp->plane_pitch_degrees;
			status->data.g_force = tmp->g_force;
			status->data.coordinate.latitude = tmp->plane_latitude;
			status->data.coordinate.longitude = tmp->plane_longitude;
			status->data.time_zulu.year = tmp->zulu_year;
			status->data.time_zulu.month_of_year = tmp->zulu_month_of_year;
			status->data.time_zulu.day_of_month = tmp->zulu_day_of_month;
			status->data.time_zulu.day_of_week = tmp->zulu_day_of_week;
			status->data.time_zulu.time_day = tmp->zulu_time;
			status->data.time_zulu.timezone_offset = 0;
			status->data.time_local.year = tmp->local_year;
			status->data.time_local.month_of_year = tmp->local_month_of_year;
			status->data.time_local.day_of_month = tmp->local_day_of_month;
			status->data.time_local.day_of_week = tmp->local_day_of_week;
			status->data.time_local.time_day = tmp->local_time;
			status->data.time_local.timezone_offset = tmp->time_zone_offset;
			if (status->sim_running && !status->paused && tmp->surface_type != 255) {
				status->in_sim = TRUE;
				if ((bool)tmp->eng_combustion_1 || (bool)tmp->eng_combustion_2) {
					if (!status->recording) {
						status->recording = TRUE;
						printf("Recording Started\n");

						memset(status->departure.icao, 0, sizeof(status->departure.icao));
						memset(status->destination.icao, 0, sizeof(status->destination.icao));
						status->airborne = !(bool)tmp->sim_on_ground;

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
							tmp,
							status,
							NULL,
							[](
								sqlite3_stmt* stmt,
								const char* stmt_txt,
								void* data,
								struct STATUS* status,
								void* aux
								) {
									struct DATA_A320* pS = (struct DATA_A320*)data;
									char time_zulu[32];
									char time_local[32];
									memset(time_zulu, 0, sizeof(time_zulu));
									memset(time_local, 0, sizeof(time_local));
									format_date_time(time_zulu, sizeof(time_zulu), pS->zulu_year, pS->zulu_month_of_year, pS->zulu_day_of_month, pS->zulu_time, 0, pS->zulu_day_of_week);
									format_date_time(time_local, sizeof(time_local), pS->local_year, pS->local_month_of_year, pS->local_day_of_month, pS->local_time, pS->time_zone_offset, pS->local_day_of_week);

									db_bind(stmt, stmt_txt, 1, pS->title);
									db_bind(stmt, stmt_txt, 2, pS->atc_airline);
									db_bind(stmt, stmt_txt, 3, pS->atc_flight_number);
									db_bind(stmt, stmt_txt, 4, pS->atc_id);
									db_bind(stmt, stmt_txt, 5, pS->atc_model);
									db_bind(stmt, stmt_txt, 6, pS->atc_type);
									db_bind(stmt, stmt_txt, 7, pS->plane_latitude);
									db_bind(stmt, stmt_txt, 8, pS->plane_longitude);
									db_bind(stmt, stmt_txt, 9, time_zulu);
									db_bind(stmt, stmt_txt, 10, time_local);
							}
						);
						db_query_table(
							status->sql,
							"SELECT id FROM trips ORDER BY id DESC LIMIT 1;",
							tmp,
							status,
							NULL,
							NULL,
							NULL,
							[](
								sqlite3_stmt* stmt,
								const char* stmt_txt,
								struct STATUS* status,
								void* aux
								) {
									status->id_trip = sqlite3_column_int(stmt, 0);
							}
						);
					}
				} else {
					if (status->recording)
						stop_recording(status);
				}
			}
			if(status->recording && !status->paused) {
				// Takeoff
				if (!(bool)tmp->sim_on_ground && !status->airborne && strcmp(status->departure.icao, "") == 0)
					SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
				// Landing
				if ((bool)tmp->sim_on_ground && status->airborne) {
					if (status->touchdown_data == NULL) {
						status->touchdown_data = (struct FLIGHT_DATA*)malloc(sizeof(struct FLIGHT_DATA));
						memset(status->touchdown_data, 0, sizeof(struct FLIGHT_DATA));
						status->touchdown_data_end = status->touchdown_data;
					}
					else {
						struct FLIGHT_DATA* tmp = (struct FLIGHT_DATA*)malloc(sizeof(struct FLIGHT_DATA));
						memset(tmp, 0, sizeof(struct FLIGHT_DATA));
						status->touchdown_data_end->next = tmp;
						status->touchdown_data_end = tmp;
					}
					status->touchdown_data_end->heading = tmp->plane_heading_degrees_magnetic;
					status->touchdown_data_end->pitch = tmp->plane_pitch_degrees;
					status->touchdown_data_end->bank = tmp->plane_bank_degrees;
					status->touchdown_data_end->speed = tmp->airspeed_indicated;
					status->touchdown_data_end->vertical_speed = tmp->vertical_speed;
					status->touchdown_data_end->g_force = tmp->g_force;
					status->touchdown_data_end->coordinate.latitude = tmp->plane_latitude;
					status->touchdown_data_end->coordinate.longitude = tmp->plane_longitude;
					status->touchdown_data_end->time_zulu.year = tmp->zulu_year;
					status->touchdown_data_end->time_zulu.month_of_year = tmp->zulu_month_of_year;
					status->touchdown_data_end->time_zulu.day_of_month = tmp->zulu_day_of_month;
					status->touchdown_data_end->time_zulu.day_of_week = tmp->zulu_day_of_week;
					status->touchdown_data_end->time_zulu.time_day = tmp->zulu_time;
					status->touchdown_data_end->time_zulu.timezone_offset = 0;
					status->touchdown_data_end->time_local.year = tmp->local_year;
					status->touchdown_data_end->time_local.month_of_year = tmp->local_month_of_year;
					status->touchdown_data_end->time_local.day_of_month = tmp->local_day_of_month;
					status->touchdown_data_end->time_local.day_of_week = tmp->local_day_of_week;
					status->touchdown_data_end->time_local.time_day = tmp->local_time;
					status->touchdown_data_end->time_local.timezone_offset = tmp->time_zone_offset;

					struct FLIGHT_DATA* cur = status->touchdown_data_end;
					char str_lat[12];
					char str_lon[12];
					char time_zulu[32];
					char time_local[32];
					memset(str_lat, 0, sizeof(str_lat));
					memset(str_lon, 0, sizeof(str_lon));
					memset(time_zulu, 0, sizeof(time_zulu));
					memset(time_local, 0, sizeof(time_local));
					coordinate_decimal_to_dms(str_lat, sizeof(str_lat), cur->coordinate.latitude, LATITUDE);
					coordinate_decimal_to_dms(str_lon, sizeof(str_lon), cur->coordinate.longitude, LONGITUDE);
					format_date_time(time_zulu, sizeof(time_zulu), cur->time_zulu);
					format_date_time(time_local, sizeof(time_local), cur->time_local);
					printf("speed   v-speed g-force pitch   bank    heading         coordinate                     time zulu                         time local\n");
					printf("  %03d\t  %03d\t  %.1f\t%+3.1f\t%+3.1f\t  %03d     %s, %s    %s    %s\n",
						cur->speed,
						cur->vertical_speed,
						cur->g_force,
						cur->pitch,
						cur->bank,
						cur->heading,
						str_lat,
						str_lon,
						time_zulu,
						time_local
					);

					SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
				}
				status->airborne = !(bool)tmp->sim_on_ground;

				double delta_s = DB_SAMPLING_INTERVAL;
				if (status->q_data_end != NULL)
					delta_s = tmp->zulu_time - status->q_data_end->zulu_time;
				else if (status->q_data_last != NULL)
					delta_s = tmp->zulu_time - status->q_data_last->zulu_time;
				if (delta_s < 0)
					delta_s += 86400;
				if (delta_s >= DB_SAMPLING_INTERVAL)
				{
					struct DATA_A320* pS = (struct DATA_A320*)malloc(sizeof(struct DATA_A320));
					memset(pS, 0, sizeof(struct DATA_A320));
					memcpy(pS, tmp, sizeof(struct DATA_A320) - sizeof(struct DATA_A320*));
					if (status->q_data_end == NULL)
						status->q_data_end = pS;
					else {
						status->q_data_end->next = pS;
						status->q_data_end = pS;
					}
					status->q_data_db_length++;
					if (status->q_data_db_start == NULL)
						status->q_data_db_start = pS;
					bool is_db_idle = FALSE;
					if (status->q_data_db_length == Q_DB_LENGTH) {
						status->q_data_db_end = pS;
						status->q_data_db_length = 0;
						std::thread thd(db_consume, status);
						thd.detach();
					}
				}
			}
		}
		break;
		default:
			printf("SIMCONNECT_RECV_SIMOBJECT_DATA: %d\n", pObjData->dwRequestID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_AIRPORT_LIST:
	{
		SIMCONNECT_RECV_AIRPORT_LIST* pWxData = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
		int min_index = -1;
		double min_distance = 100000;
		for (int i = 0; i < pWxData->dwArraySize; i++) {
			struct SIMCONNECT_DATA_FACILITY_AIRPORT airport = pWxData->rgData[i];
			struct COORDINATE airport_loc;
			airport_loc.latitude = airport.Latitude;
			airport_loc.longitude = airport.Longitude;
			double distance = abs(distanceInKmBetweenEarthCoordinates(status->data.coordinate, airport_loc));
			if (distance <= min_distance) {
				min_distance = distance;
				min_index = i;
			}
		}
		if (min_distance < 10) {
			char* ident = pWxData->rgData[min_index].Ident;
			if (strcmp(status->departure.icao, "") == 0)
				memcpy(status->departure.icao, ident, sizeof(ident));
			else
				memcpy(status->destination.icao, ident, sizeof(ident));
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN AIRPORT");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "NAME64");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "MAGVAR");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "N_RUNWAYS");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN RUNWAY");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LATITUDE");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LONGITUDE");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LENGTH");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "WIDTH");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "HEADING");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_NUMBER");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_DESIGNATOR");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_NUMBER");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_DESIGNATOR");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE RUNWAY");
			SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE AIRPORT");
			SimConnect_RequestFacilityData(status->hSimConnect, DEFINITION_RUNWAYS, REQUEST_RUNWAYS, ident);
		}
	}
	break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA:
	{
		SIMCONNECT_RECV_FACILITY_DATA* pWxData = (SIMCONNECT_RECV_FACILITY_DATA*)pData;
		switch (pWxData->Type) {
		case SIMCONNECT_FACILITY_DATA_AIRPORT:
		{
			struct FACILITY_AIRPORT* airport = (struct FACILITY_AIRPORT*)&pWxData->Data;
			struct AIRPORT* tmp = NULL;
			if (status->departure.runway_act_index == -1)
				tmp = &status->departure;
			else
				tmp = &status->destination;
			tmp->extra_info.magvar = airport->magvar;
			tmp->extra_info.n_runways = airport->n_runways;
			tmp->runways = (struct RUNWAY*)malloc(sizeof(struct RUNWAY) * airport->n_runways);
			memset(tmp->runways, 0, sizeof(struct RUNWAY)* airport->n_runways);
			memcpy(tmp->extra_info.name, airport->name, sizeof(airport->name));
		}
		break;
		case SIMCONNECT_FACILITY_DATA_RUNWAY:
		{
			struct RUNWAY* rep = NULL;
			if (status->departure.runway_act_index == -1)
				rep = status->departure.runways;
			else
				rep = status->destination.runways;
			memcpy(&rep[pWxData->ItemIndex], (struct RUNWAY*)&pWxData->Data, sizeof(struct RUNWAY));
		}
		break;
		default:
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA_END:
	{
		struct AIRPORT* rep = NULL;
		if (status->departure.runway_act_index == -1)
			rep = &status->departure;
		else
			rep = &status->destination;
		for (int i = 0; i < rep->extra_info.n_runways; i++) {
			struct RUNWAY rwy = rep->runways[i];
			double rwy_heading = rwy.heading;
			if (rwy.primary_number > 0 && rwy.primary_number < 37) {
				struct COORDINATE dest = destinationWithDistanceAndBearing(rwy.coordinate, rwy.length / 2000, rwy_heading);
				double bearing = bearingBetweenEarchCoordinates(status->data.coordinate, dest) + rep->extra_info.magvar;
				if (bearing > 360)
					bearing -= 360;
				int diff_bearing = abs((int)(bearing / 10) - rwy.primary_number);
				if (diff_bearing > 18)
					diff_bearing = 36 - diff_bearing;
				int diff_heading = abs((int)(status->data.heading / 10) - rwy.primary_number);
				if (diff_heading > 18)
					diff_heading = 36 - diff_heading;
				if (diff_bearing <= 2 && diff_heading <= 3) {
					rep->runway_act_index = i;
					rep->runway_act_primary = TRUE;
				}
			}
			if (rwy.secondary_number > 0 && rwy.secondary_number < 37) {
				rwy_heading = rwy_heading - 180;
				if (rwy_heading < 0)
					rwy_heading += 360;
				struct COORDINATE dest = destinationWithDistanceAndBearing(rwy.coordinate, rwy.length / 2000, rwy_heading);
				double bearing = bearingBetweenEarchCoordinates(status->data.coordinate, dest) + rep->extra_info.magvar;
				if (bearing > 360)
					bearing -= 360;
				int diff_bearing = abs((int)(bearing / 10) - rwy.secondary_number);
				if (diff_bearing > 18)
					diff_bearing = 36 - diff_bearing;
				int diff_heading = abs((int)(status->data.heading / 10) - rwy.secondary_number);
				if (diff_heading > 18)
					diff_heading = 36 - diff_heading;
				if (diff_bearing <= 2 && diff_heading <= 3) {
					rep->runway_act_index = i;
					rep->runway_act_primary = FALSE;
				}
			}
		}
		if (rep->runway_act_index != -1) {
			char strRunway[4];
			memset(strRunway, 0, sizeof(strRunway));
			runway_code_generator(strRunway, sizeof(strRunway), *rep);
			if (rep == &status->departure) {
				printf("Takeoff from %s (%s) runway %s\n", rep->extra_info.name, rep->icao, strRunway);
				db_insert_update_table(status->sql,
					"UPDATE trips SET departure_icao=?,departure_rwy=? WHERE id=?;",
					NULL,
					status,
					strRunway,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->departure.icao);
						db_bind(stmt, stmt_txt, 2, (char*)aux);
						db_bind(stmt, stmt_txt, 3, status->id_trip);
					}
				);
			}
			else {
				printf("Touchdown at %s (%s) runway %s\n", rep->extra_info.name, rep->icao, strRunway);
				db_insert_update_table(status->sql,
					"UPDATE trips SET destination_icao=?,destination_rwy=? WHERE id=?;",
					NULL,
					status,
					strRunway,
					[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
						db_bind(stmt, stmt_txt, 1, status->destination.icao);
						db_bind(stmt, stmt_txt, 2, (char*)aux);
						db_bind(stmt, stmt_txt, 3, status->id_trip);
					}
				);
				struct FLIGHT_DATA* tmp = status->touchdown_data;
				while (tmp != NULL && strcmp(tmp->icao, "") != 0)
					tmp = tmp->next;
				if (tmp != NULL) {
					memcpy(tmp->icao, status->destination.icao, sizeof(status->destination.icao));
					memcpy(tmp->runway, strRunway, sizeof(strRunway));
				}
			}
		}
	}
	break;
	case SIMCONNECT_RECV_ID_EXCEPTION:
	{
		const char* exceptions[] = {
			"SIMCONNECT_EXCEPTION_NONE",
			"SIMCONNECT_EXCEPTION_ERROR",
			"SIMCONNECT_EXCEPTION_SIZE_MISMATCH",
			"SIMCONNECT_EXCEPTION_UNRECOGNIZED_ID",
			"SIMCONNECT_EXCEPTION_UNOPENED",
			"SIMCONNECT_EXCEPTION_VERSION_MISMATCH",
			"SIMCONNECT_EXCEPTION_TOO_MANY_GROUPS",
			"SIMCONNECT_EXCEPTION_NAME_UNRECOGNIZED",
			"SIMCONNECT_EXCEPTION_TOO_MANY_EVENT_NAMES",
			"SIMCONNECT_EXCEPTION_EVENT_ID_DUPLICATE",
			"SIMCONNECT_EXCEPTION_TOO_MANY_MAPS",
			"SIMCONNECT_EXCEPTION_TOO_MANY_OBJECTS",
			"SIMCONNECT_EXCEPTION_TOO_MANY_REQUESTS",
			"SIMCONNECT_EXCEPTION_WEATHER_INVALID_PORT",
			"SIMCONNECT_EXCEPTION_WEATHER_INVALID_METAR",
			"SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_GET_OBSERVATION",
			"SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_CREATE_STATION",
			"SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_REMOVE_STATION",
			"SIMCONNECT_EXCEPTION_INVALID_DATA_TYPE",
			"SIMCONNECT_EXCEPTION_INVALID_DATA_SIZE",
			"SIMCONNECT_EXCEPTION_DATA_ERROR",
			"SIMCONNECT_EXCEPTION_INVALID_ARRAY",
			"SIMCONNECT_EXCEPTION_CREATE_OBJECT_FAILED",
			"SIMCONNECT_EXCEPTION_LOAD_FLIGHTPLAN_FAILED",
			"SIMCONNECT_EXCEPTION_OPERATION_INVALID_FOR_OBJECT_TYPE",
			"SIMCONNECT_EXCEPTION_ILLEGAL_OPERATION",
			"SIMCONNECT_EXCEPTION_ALREADY_SUBSCRIBED",
			"SIMCONNECT_EXCEPTION_INVALID_ENUM",
			"SIMCONNECT_EXCEPTION_DEFINITION_ERROR",
			"SIMCONNECT_EXCEPTION_DUPLICATE_ID",
			"SIMCONNECT_EXCEPTION_DATUM_ID",
			"SIMCONNECT_EXCEPTION_OUT_OF_BOUNDS",
			"SIMCONNECT_EXCEPTION_ALREADY_CREATED",
			"SIMCONNECT_EXCEPTION_OBJECT_OUTSIDE_REALITY_BUBBLE",
			"SIMCONNECT_EXCEPTION_OBJECT_CONTAINER",
			"SIMCONNECT_EXCEPTION_OBJECT_AI",
			"SIMCONNECT_EXCEPTION_OBJECT_ATC",
			"SIMCONNECT_EXCEPTION_OBJECT_SCHEDULE",
			"SIMCONNECT_EXCEPTION_JETWAY_DATA",
			"SIMCONNECT_EXCEPTION_ACTION_NOT_FOUND",
			"SIMCONNECT_EXCEPTION_NOT_AN_ACTION",
			"SIMCONNECT_EXCEPTION_INCORRECT_ACTION_PARAMS",
			"SIMCONNECT_EXCEPTION_GET_INPUT_EVENT_FAILED",
			"SIMCONNECT_EXCEPTION_SET_INPUT_EVENT_FAILED",
		};
		//SIMCONNECT_RECV_EXCEPTION* except = (SIMCONNECT_RECV_EXCEPTION*)pData;
		//printf("%s\n", exceptions[except->dwException]);
	}
	break;
	default:
		printf("SIMCONNECT_RECV: %d\n", pData->dwID);
		break;
	}
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
	//SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_1, SIMCONNECT_GROUP_PRIORITY_HIGHEST);
}

void connect_db(struct STATUS* status) {
	char* fn_db = (char*)malloc(strlen(DATABASE_NAME) + 4);
	memset(fn_db, 0, strlen(DATABASE_NAME) + 4);
	memcpy(fn_db, DATABASE_NAME, strlen(DATABASE_NAME));
	memcpy(fn_db + strlen(DATABASE_NAME), ".db", 4);
	if (sqlite3_open_v2(fn_db, &status->sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, NULL) == SQLITE_OK)
		printf("Opened database %s\n", DATABASE_NAME);
	else {
		printf("Can't open database: %s\n", sqlite3_errmsg(status->sql));
		exit(1);
	}
	free(fn_db);

	const char* stmt_txt_start = "CREATE TABLE IF NOT EXISTS ";
	const char* stmt_txt_sep_1 = " (";
	const char* stmt_txt_sep_2 = ");";
	int len_start = strlen(stmt_txt_start);
	int len_sep_1 = strlen(stmt_txt_sep_1);
	int len_sep_2 = strlen(stmt_txt_sep_2);
	sqlite3_stmt* stmt = NULL;
	for (int i = 0; i < sizeof(DATABASE_TABLE_NAMES) / sizeof(char*); i++) {
		int len_name = strlen(DATABASE_TABLE_NAMES[i]);
		int len_fields = strlen(DATABASE_TABLE_FIELDS[i]);
		char* stmt_txt = (char*)malloc(len_start + len_name + len_sep_1 + len_fields + len_sep_2 + 2);
		memset(stmt_txt, 0, len_start + len_name + len_sep_1 + len_fields + len_sep_2 + 2);
		memcpy(stmt_txt, stmt_txt_start, strlen(stmt_txt_start));
		memcpy(stmt_txt + len_start, DATABASE_TABLE_NAMES[i], strlen(DATABASE_TABLE_NAMES[i]));
		memcpy(stmt_txt + len_start + len_name, stmt_txt_sep_1, strlen(stmt_txt_sep_1));
		memcpy(stmt_txt + len_start + len_name + len_sep_1, DATABASE_TABLE_FIELDS[i], strlen(DATABASE_TABLE_FIELDS[i]));
		memcpy(stmt_txt + len_start + len_name + len_sep_1 + len_fields, stmt_txt_sep_2, strlen(stmt_txt_sep_2));
		if (sqlite3_prepare_v2(status->sql, stmt_txt, -1, &stmt, NULL) == SQLITE_OK)
			sqlite3_step(stmt);
		else {
			printf("Incorrect db operation \"%s\"\n", stmt_txt);
			exit(2);
		}
		sqlite3_finalize(stmt);
		free(stmt_txt);
	}
}

int main() {
	struct STATUS status;
	SimConnect_Open(&status.hSimConnect, "Flight Data Recorder", NULL, 0, 0, SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_SIM, "Sim");
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_PAUSE, "Pause");
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_CRASHED, "Crashed");
	add_client_events(status.hSimConnect);
	add_definition_a320(status.hSimConnect);

	connect_db(&status);
	while (!status.quit)
		SimConnect_CallDispatch(status.hSimConnect, MyDispatchProc, &status);
	SimConnect_Close(status.hSimConnect);
	sqlite3_close_v2(status.sql);

	return 0;
}

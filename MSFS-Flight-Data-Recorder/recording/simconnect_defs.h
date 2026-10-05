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

struct FLIGHT_DATA_RECORD {
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
	// Read only to start/stop the trip (flight_phase.cpp); trip_data's bool
	// groups have no free bit to store them in.
	double eng_combustion_3;
	double eng_combustion_4;
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
	// Engines 1..MAX_ENGINES, in ENGINE_POWER_SIMVARS order (sim_link.cpp);
	// enginePowerFromRecord() picks which ones a sample records.
	double general_eng_rpm[MAX_ENGINES];
	double recip_eng_manifold_pressure[MAX_ENGINES];
	double turb_eng_n1[MAX_ENGINES];
	double turb_eng_n2[MAX_ENGINES];
	double turb_eng_max_torque_percent[MAX_ENGINES];
	double prop_rpm[MAX_ENGINES];
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
	COORDINATE plane_coordinate;
	double plane_pitch_degrees;
	double plane_touchdown_bank_degrees;
	double plane_touchdown_heading_degrees_magnetic;
	double plane_touchdown_heading_degrees_true;
	COORDINATE plane_touchdown_coordinate;
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
	COORDINATE gps_position_coordinate;
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
	double kohlsman_setting_hg;
	double kohlsman_setting_mb;
	double kohlsman_setting_std;
	char title[256];
	char atc_airline[64];
	char atc_flight_number[8];
	char atc_id[32];
	char atc_model[32];
	char atc_type[64];
	DATETIME time_local;
	DATETIME time_zulu;
};

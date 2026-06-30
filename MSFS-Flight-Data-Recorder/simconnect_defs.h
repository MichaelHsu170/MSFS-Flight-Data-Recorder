#pragma once

#include "types.h"
#include "SimConnect.h"

enum GROUP_ID {
	GROUP_1,
};

enum EVENT_ID {
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
	struct FLIGHT_DATA_RECORD* next;
};

#pragma once

#include "types.h"

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
	"departure_name VARCHAR(64),"
	"departure_region VARCHAR(2),"
	"departure_rwy VARCHAR(3),"
	"departure_zulu_time VARCHAR(32) NOT NULL,"
	"departure_local_time VARCHAR(32) NOT NULL,"
	"destination_latitude REAL,"
	"destination_longitude REAL,"
	"destination_icao VARCHAR(4),"
	"destination_name VARCHAR(64),"
	"destination_region VARCHAR(2),"
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
	"airport_name VARCHAR(64),"
	"runway VARCHAR(3),"
	"distance_length REAL,"
	"distance_width REAL,"
	"distance_length_percent REAL,"
	"distance_width_percent REAL,"
	"wind_direction INTEGER,"
	"wind_velocity INTEGER,"
	"time_zulu VARCHAR(32) NOT NULL,"
	"time_local VARCHAR(32) NOT NULL,"
	"analysis_report TEXT"
};

struct db_exception {
	std::string message;
	db_exception(const std::string& msg) : message(msg) {}
};

void db_error(const char* stmt_txt, int sql_ret, char** errmsg);

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, int value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, double value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, char* value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value);

void db_query_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux_in,
	void* aux_out,
	void (*func_set_stmt)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*),
	void (*func_retrieve_data)(sqlite3_stmt*, const char*, struct STATUS*, void*)
);

void db_insert_update_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux,
	void (*func)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*)
);

// Immediately writes one event row to trip_events. Safe to call from the
// SimConnect dispatch callback (main thread) while db_consume runs on a worker
// thread — both paths serialize through STATUS::mutex_db_commit.
void db_insert_event(STATUS* status, const char* event, const char* time_zulu, const char* time_local);

void db_consume(STATUS* status);

// Creates the schema and migrates any missing columns on an ephemeral R/W
// connection. Called at app startup so read-only queries always see the
// current schema, even when the simulator has never connected this session.
void migrate_db();

void connect_db(struct STATUS* status);
sqlite3* connect_db_readonly();
// Read-write connection for explicit GUI write operations (e.g. deleting a
// trip). Does NOT create the database (SQLITE_OPEN_READWRITE only — no CREATE),
// so it fails cleanly if no database exists yet. Caller must sqlite3_close().
sqlite3* connect_db_readwrite();

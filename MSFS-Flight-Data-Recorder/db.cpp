#include "db.h"
#include "simconnect_defs.h"
#include <thread>

void db_error(const char* stmt_txt, int sql_ret, char** errmsg) {
	std::string msg;
	if (sql_ret != 0) {
		msg = std::string("db operation \"") + stmt_txt + "\" failed with error " + std::to_string(sql_ret);
		printf("%s\n", msg.c_str());
	}
	if (errmsg != NULL && *errmsg != NULL) {
		msg = std::string("db operation \"") + stmt_txt + "\" failed with error " + *errmsg;
		printf("%s\n", msg.c_str());
		sqlite3_free(*errmsg);
		*errmsg = NULL;
	}
	throw db_exception(msg);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, int value) {
	int sql_ret = sqlite3_bind_int(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, double value) {
	int sql_ret = sqlite3_bind_double(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, char* value) {
	int sql_ret = sqlite3_bind_text(stmt, index, value, (int)strlen(value), SQLITE_TRANSIENT);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value) {
	int sql_ret = sqlite3_bind_text(stmt, index, value, (int)strlen(value), SQLITE_TRANSIENT);
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
	void (*func_set_stmt)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*),
	void (*func_retrieve_data)(sqlite3_stmt*, const char*, struct STATUS*, void*)
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

void db_insert_update_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux,
	void (*func)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*)
) {
	status->mutex_db_commit.lock();
	sqlite3_stmt* stmt = NULL;
	int sql_ret = 0;
	char* errmsg = NULL;
	try {
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
	}
	catch (const db_exception&) {
		if (stmt != NULL)
			sqlite3_finalize(stmt);
		sqlite3_exec(sql, "ROLLBACK TRANSACTION", NULL, NULL, NULL);
		status->mutex_db_commit.unlock();
		throw;
	}
	status->mutex_db_commit.unlock();
}

void db_consume(STATUS* status) {
	while (status->q_data_db_start != NULL) {
		struct FLIGHT_DATA_RECORD* pS = status->q_data_db_start;
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
				struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
				int bool_group_1 = 0;
				int bool_group_2 = 0;
				int bool_group_3 = 0;
				if (pS->autopilot_airspeed_hold)        bool_group_1 |= (0x1 << 0);
				if (pS->autopilot_alt_radio_mode)        bool_group_1 |= (0x1 << 1);
				if (pS->autopilot_altitude_lock)         bool_group_1 |= (0x1 << 2);
				if (pS->autopilot_approach_active)       bool_group_1 |= (0x1 << 3);
				if (pS->autopilot_approach_captured)     bool_group_1 |= (0x1 << 4);
				if (pS->autopilot_approach_hold)         bool_group_1 |= (0x1 << 5);
				if (pS->autopilot_approach_is_localizer) bool_group_1 |= (0x1 << 6);
				if (pS->autopilot_avionics_managed)      bool_group_1 |= (0x1 << 7);
				if (pS->autopilot_disengaged)            bool_group_1 |= (0x1 << 8);
				if (pS->autopilot_flight_director_active)bool_group_1 |= (0x1 << 9);
				if (pS->autopilot_flight_level_change)   bool_group_1 |= (0x1 << 10);
				if (pS->autopilot_glideslope_active)     bool_group_1 |= (0x1 << 11);
				if (pS->autopilot_glideslope_arm)        bool_group_1 |= (0x1 << 12);
				if (pS->autopilot_glideslope_hold)       bool_group_1 |= (0x1 << 13);
				if (pS->autopilot_heading_lock)          bool_group_1 |= (0x1 << 14);
				if (pS->autopilot_mach_hold)             bool_group_1 |= (0x1 << 15);
				if (pS->autopilot_managed_speed_in_mach) bool_group_1 |= (0x1 << 16);
				if (pS->autopilot_managed_throttle_active)bool_group_1|= (0x1 << 17);
				if (pS->autopilot_master)                bool_group_1 |= (0x1 << 18);
				if (pS->autopilot_takeoff_power_active)  bool_group_1 |= (0x1 << 19);
				if (pS->autopilot_throttle_arm)          bool_group_1 |= (0x1 << 20);
				if (pS->autopilot_vertical_hold)         bool_group_1 |= (0x1 << 21);
				if (pS->autobrakes_active)               bool_group_1 |= (0x1 << 22);
				if (pS->brake_parking_indicator)         bool_group_1 |= (0x1 << 23);
				if (pS->rejected_takeoff_brakes_active)  bool_group_1 |= (0x1 << 24);
				if (pS->gear_damage_by_speed)            bool_group_1 |= (0x1 << 25);
				if (pS->gear_is_on_ground_0)             bool_group_1 |= (0x1 << 26);
				if (pS->gear_is_on_ground_1)             bool_group_1 |= (0x1 << 27);
				if (pS->gear_is_on_ground_2)             bool_group_1 |= (0x1 << 28);
				if (pS->gear_speed_exceeded)             bool_group_1 |= (0x1 << 29);
				if (pS->aileron_trim_disabled)           bool_group_1 |= (0x1 << 30);
				if (pS->elevator_trim_disabled)          bool_group_1 |= (0x1 << 31);
				if (pS->flap_damage_by_speed)            bool_group_2 |= (0x1 << 0);
				if (pS->flap_speed_exceeded)             bool_group_2 |= (0x1 << 1);
				if (pS->rudder_trim_disabled)            bool_group_2 |= (0x1 << 2);
				if (pS->spoilers_armed)                  bool_group_2 |= (0x1 << 3);
				if (pS->apu_generator_active)            bool_group_2 |= (0x1 << 4);
				if (pS->apu_generator_switch)            bool_group_2 |= (0x1 << 5);
				if (pS->apu_on_fire_detected)            bool_group_2 |= (0x1 << 6);
				if (pS->apu_switch)                      bool_group_2 |= (0x1 << 7);
				if (pS->bleed_air_apu)                   bool_group_2 |= (0x1 << 8);
				if (pS->electrical_master_battery)       bool_group_2 |= (0x1 << 9);
				if (pS->external_power_available)        bool_group_2 |= (0x1 << 10);
				if (pS->external_power_connection_on)    bool_group_2 |= (0x1 << 11);
				if (pS->external_power_on)               bool_group_2 |= (0x1 << 12);
				if (pS->bleed_air_engine_1)              bool_group_2 |= (0x1 << 13);
				if (pS->bleed_air_engine_2)              bool_group_2 |= (0x1 << 14);
				if (pS->eng_anti_ice_1)                  bool_group_2 |= (0x1 << 15);
				if (pS->eng_anti_ice_2)                  bool_group_2 |= (0x1 << 16);
				if (pS->eng_combustion_1)                bool_group_2 |= (0x1 << 17);
				if (pS->eng_combustion_2)                bool_group_2 |= (0x1 << 18);
				if (pS->eng_failed_1)                    bool_group_2 |= (0x1 << 19);
				if (pS->eng_failed_2)                    bool_group_2 |= (0x1 << 20);
				if (pS->eng_on_fire_1)                   bool_group_2 |= (0x1 << 21);
				if (pS->eng_on_fire_2)                   bool_group_2 |= (0x1 << 22);
				if (pS->general_eng_fire_detected_1)     bool_group_2 |= (0x1 << 23);
				if (pS->general_eng_fire_detected_2)     bool_group_2 |= (0x1 << 24);
				if (pS->general_eng_fuel_valve_1)        bool_group_2 |= (0x1 << 25);
				if (pS->general_eng_fuel_valve_2)        bool_group_2 |= (0x1 << 26);
				if (pS->general_eng_generator_active_1)  bool_group_2 |= (0x1 << 27);
				if (pS->general_eng_generator_active_2)  bool_group_2 |= (0x1 << 28);
				if (pS->general_eng_generator_switch_1)  bool_group_2 |= (0x1 << 29);
				if (pS->general_eng_generator_switch_2)  bool_group_2 |= (0x1 << 30);
				if (pS->general_eng_master_alternator)   bool_group_2 |= (0x1 << 31);
				if (pS->general_eng_reverse_thrust_engaged) bool_group_3 |= (0x1 << 0);
				if (pS->general_eng_starter_1)           bool_group_3 |= (0x1 << 1);
				if (pS->general_eng_starter_2)           bool_group_3 |= (0x1 << 2);
				if (pS->general_eng_starter_active_1)    bool_group_3 |= (0x1 << 3);
				if (pS->general_eng_starter_active_2)    bool_group_3 |= (0x1 << 4);
				if (pS->master_ignition_switch)          bool_group_3 |= (0x1 << 5);
				if (pS->turb_eng_fuel_available_1)       bool_group_3 |= (0x1 << 6);
				if (pS->turb_eng_fuel_available_2)       bool_group_3 |= (0x1 << 7);
				if (pS->turb_eng_is_igniting_1)          bool_group_3 |= (0x1 << 8);
				if (pS->turb_eng_is_igniting_2)          bool_group_3 |= (0x1 << 9);
				if (pS->fuel_transfer_pump_on_l)         bool_group_3 |= (0x1 << 10);
				if (pS->fuel_transfer_pump_on_r)         bool_group_3 |= (0x1 << 11);
				if (pS->on_any_runway)                   bool_group_3 |= (0x1 << 12);
				if (pS->plane_in_parking_state)          bool_group_3 |= (0x1 << 13);
				if (pS->autothrottle_active)             bool_group_3 |= (0x1 << 14);
				if (pS->avionics_master_switch)          bool_group_3 |= (0x1 << 15);
				if (pS->cabin_no_smoking_alert_switch)   bool_group_3 |= (0x1 << 16);
				if (pS->cabin_seatbelts_alert_switch)    bool_group_3 |= (0x1 << 17);
				if (pS->gpws_system_active)              bool_group_3 |= (0x1 << 18);
				if (pS->gpws_warning)                    bool_group_3 |= (0x1 << 19);
				if (pS->overspeed_warning)               bool_group_3 |= (0x1 << 20);
				if (pS->pitot_heat)                      bool_group_3 |= (0x1 << 21);
				if (pS->stall_warning)                   bool_group_3 |= (0x1 << 22);
				if (pS->structural_deice_switch)         bool_group_3 |= (0x1 << 23);
				if (pS->hydraulic_switch)                bool_group_3 |= (0x1 << 24);
				if (pS->warning_fuel)                    bool_group_3 |= (0x1 << 25);
				if (pS->warning_low_height)              bool_group_3 |= (0x1 << 26);
				if (pS->warning_oil_pressure)            bool_group_3 |= (0x1 << 27);
				if (pS->warning_vacuum)                  bool_group_3 |= (0x1 << 28);
				if (pS->warning_voltage)                 bool_group_3 |= (0x1 << 29);
				if (pS->sim_on_ground)                   bool_group_3 |= (0x1 << 30);
				if (pS->kohlsman_setting_std)            bool_group_3 |= (0x1 << 31);

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
				db_bind(stmt, stmt_txt, 23, pS->plane_coordinate.latitude);
				db_bind(stmt, stmt_txt, 24, pS->plane_coordinate.longitude);
				db_bind(stmt, stmt_txt, 25, pS->plane_pitch_degrees);
				db_bind(stmt, stmt_txt, 26, pS->plane_touchdown_bank_degrees);
				db_bind(stmt, stmt_txt, 27, pS->plane_touchdown_heading_degrees_magnetic);
				db_bind(stmt, stmt_txt, 28, pS->plane_touchdown_heading_degrees_true);
				db_bind(stmt, stmt_txt, 29, pS->plane_touchdown_coordinate.latitude);
				db_bind(stmt, stmt_txt, 30, pS->plane_touchdown_coordinate.longitude);
				db_bind(stmt, stmt_txt, 31, pS->plane_touchdown_pitch_degrees);
				db_bind(stmt, stmt_txt, 32, pS->gps_ground_true_heading);
				db_bind(stmt, stmt_txt, 33, pS->gps_ground_true_track);
				db_bind(stmt, stmt_txt, 34, pS->gps_position_coordinate.latitude);
				db_bind(stmt, stmt_txt, 35, pS->gps_position_coordinate.longitude);
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
				db_bind(stmt, stmt_txt, 145, pS->time_zulu.format_date_time().c_str());
				db_bind(stmt, stmt_txt, 146, pS->time_local.format_date_time().c_str());
			});
		bool fBreak = FALSE;
		if (status->q_data_db_start == status->q_data_db_end)
			fBreak = TRUE;
		status->q_data_db_start = status->q_data_db_start->next;
		if (status->q_data_db_start == NULL) {
			if (status->q_data_last != NULL)
				free(status->q_data_last);
			status->q_data_last = pS;
		} else
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
			[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
				struct EVENT_DB* pS = (struct EVENT_DB*)data;
				db_bind(stmt, stmt_txt, 1, status->id_trip);
				db_bind(stmt, stmt_txt, 2, pS->event);
				db_bind(stmt, stmt_txt, 3, pS->time_zulu.format_date_time().c_str());
				db_bind(stmt, stmt_txt, 4, pS->time_local.format_date_time().c_str());
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

static void resolve_db_path(char* fn_db, size_t len) {
#ifdef _DEBUG
	snprintf(fn_db, len, "%s.db", DATABASE_NAME);
#else
	char exe_path[MAX_PATH];
	GetModuleFileNameA(NULL, exe_path, MAX_PATH);
	char* last_slash = strrchr(exe_path, '\\');
	if (last_slash)
		*last_slash = '\0';
	snprintf(fn_db, len, "%s\\%s.db", exe_path, DATABASE_NAME);
#endif
}

// A second, independent connection for read-only history queries (Trip History
// feature). connect_db()'s connection is opened with SQLITE_OPEN_NOMUTEX, so it
// is only safe to use from the single thread that owns it (the dispatch loop /
// db_consume worker) — it must never be shared with a background query thread.
// This connection is opened without NOMUTEX, so SQLite's own per-connection
// mutex makes it safe to call from whichever single thread is using it at a time.
sqlite3* connect_db_readwrite() {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);
	sqlite3* sql = NULL;
	if (sqlite3_open_v2(fn_db, &sql, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
		if (sql != NULL)
			sqlite3_close(sql);
		return NULL;
	}
	sqlite3_busy_timeout(sql, 5000);
	return sql;
}

sqlite3* connect_db_readonly() {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);

	sqlite3* sql = NULL;
	if (sqlite3_open_v2(fn_db, &sql, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
		if (sql != NULL)
			sqlite3_close(sql);
		return NULL;
	}
	sqlite3_busy_timeout(sql, 2000);
	return sql;
}

void connect_db(struct STATUS* status) {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);

	if (sqlite3_open_v2(fn_db, &status->sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, NULL) == SQLITE_OK)
		printf("Opened database %s\n", fn_db);
	else {
		printf("Can't open database: %s\n", sqlite3_errmsg(status->sql));
		exit(1);
	}

	const char* stmt_txt_start = "CREATE TABLE IF NOT EXISTS ";
	const char* stmt_txt_sep_1 = " (";
	const char* stmt_txt_sep_2 = ");";
	int len_start  = (int)strlen(stmt_txt_start);
	int len_sep_1  = (int)strlen(stmt_txt_sep_1);
	int len_sep_2  = (int)strlen(stmt_txt_sep_2);
	sqlite3_stmt* stmt = NULL;
	for (int i = 0; i < (int)(sizeof(DATABASE_TABLE_NAMES) / sizeof(char*)); i++) {
		int len_name   = (int)strlen(DATABASE_TABLE_NAMES[i]);
		int len_fields = (int)strlen(DATABASE_TABLE_FIELDS[i]);
		char* stmt_txt = (char*)malloc(len_start + len_name + len_sep_1 + len_fields + len_sep_2 + 2);
		memset(stmt_txt, 0, len_start + len_name + len_sep_1 + len_fields + len_sep_2 + 2);
		memcpy(stmt_txt, stmt_txt_start, len_start);
		memcpy(stmt_txt + len_start, DATABASE_TABLE_NAMES[i], len_name);
		memcpy(stmt_txt + len_start + len_name, stmt_txt_sep_1, len_sep_1);
		memcpy(stmt_txt + len_start + len_name + len_sep_1, DATABASE_TABLE_FIELDS[i], len_fields);
		memcpy(stmt_txt + len_start + len_name + len_sep_1 + len_fields, stmt_txt_sep_2, len_sep_2);
		if (sqlite3_prepare_v2(status->sql, stmt_txt, -1, &stmt, NULL) == SQLITE_OK)
			sqlite3_step(stmt);
		else {
			printf("Incorrect db operation \"%s\"\n", stmt_txt);
			exit(2);
		}
		sqlite3_finalize(stmt);
		free(stmt_txt);
	}

	// trip_data/trip_events/trip_touchdowns are all queried with "WHERE trip = ?"
	// (db_history.cpp) -- without an index that's a full table scan across every
	// sample ever recorded, for every trip load. trips itself has no such index
	// need: its only query is an unfiltered "ORDER BY id DESC" over the whole
	// table, and id is already the PRIMARY KEY. IF NOT EXISTS makes this safe to
	// run on every connect, same as the CREATE TABLE loop above; the one-time
	// build cost for an existing large database is paid back on every load after.
	static const char* index_stmts[] = {
		"CREATE INDEX IF NOT EXISTS idx_trip_data_trip ON trip_data(trip);",
		"CREATE INDEX IF NOT EXISTS idx_trip_events_trip ON trip_events(trip);",
		"CREATE INDEX IF NOT EXISTS idx_trip_touchdowns_trip ON trip_touchdowns(trip);",
	};
	for (int i = 0; i < (int)(sizeof(index_stmts) / sizeof(char*)); i++) {
		if (sqlite3_prepare_v2(status->sql, index_stmts[i], -1, &stmt, NULL) == SQLITE_OK)
			sqlite3_step(stmt);
		else {
			printf("Incorrect db operation \"%s\"\n", index_stmts[i]);
			exit(2);
		}
		sqlite3_finalize(stmt);
	}
}

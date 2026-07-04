#pragma once

#include <QString>
#include <QStringList>

// Turns a trip_data column / FLIGHT_DATA_RECORD member name like
// "plane_touchdown_latitude" into a display label like "Plane Touchdown
// Latitude". Shared by both producers below so the label shown in
// DataTablePanel never depends on whether the point came from a live sample
// or a historical DB row.
inline QString tripFieldLabel(const char* name) {
	QStringList words = QString::fromLatin1(name).split('_', Qt::SkipEmptyParts);
	for (QString& word : words) {
		if (!word.isEmpty())
			word[0] = word[0].toUpper();
	}
	return words.join(' ');
}

// Canonical list of every trip_data column that represents flight telemetry
// (i.e. every column except the `id`/`trip` key columns and `zulu_time`/
// `local_time`, which DataTablePanel shows in their own dedicated rows
// instead). Expanded via X-macros so the field list is written exactly once
// and shared by three consumers that must agree on the same field set and
// order: recorder_bridge.cpp (live path fills TripSamplePoint::rawNums from
// FLIGHT_DATA_RECORD), db_history.cpp (historical path fills rawNums from
// sqlite3_column_double), and data_table_panel.cpp (formats rawNums/boolGroups
// to display strings using these same macros in showPoint).
//
// TRIP_DATA_NUM_FIELDS(X): X(dbColumnName, recordMemberExpr) -- a plain
// numeric trip_data column. recordMemberExpr is the FLIGHT_DATA_RECORD
// expression (dotted for the COORDINATE-typed fields) that supplies it live.
//
// TRIP_DATA_BOOL_FIELDS(X): X(name, boolGroup, bitIndex) -- a boolean that's
// bit-packed into bool_group_<boolGroup> at bit <bitIndex> for storage (see
// db.cpp's db_consume()); `name` is both the trip_data column's FLIGHT_DATA_
// RECORD member name (live path reads it as a plain bool) and the storage
// bit's label (historical path unpacks it from the matching bool_group).

#define TRIP_DATA_NUM_FIELDS(X) \
	X(eng_exhaust_gas_temperature_1, eng_exhaust_gas_temperature_1) \
	X(eng_exhaust_gas_temperature_2, eng_exhaust_gas_temperature_2) \
	X(eng_oil_temperature_1, eng_oil_temperature_1) \
	X(eng_oil_temperature_2, eng_oil_temperature_2) \
	X(ambient_temperature, ambient_temperature) \
	X(autopilot_heading_lock_dir, autopilot_heading_lock_dir) \
	X(aileron_left_deflection, aileron_left_deflection) \
	X(aileron_right_deflection, aileron_right_deflection) \
	X(aileron_trim, aileron_trim) \
	X(elevator_deflection, elevator_deflection) \
	X(elevator_trim_position, elevator_trim_position) \
	X(elevon_deflection, elevon_deflection) \
	X(rudder_deflection, rudder_deflection) \
	X(rudder_trim, rudder_trim) \
	X(plane_bank_degrees, plane_bank_degrees) \
	X(plane_heading_degrees_gyro, plane_heading_degrees_gyro) \
	X(plane_heading_degrees_magnetic, plane_heading_degrees_magnetic) \
	X(plane_heading_degrees_true, plane_heading_degrees_true) \
	X(plane_latitude, plane_coordinate.latitude) \
	X(plane_longitude, plane_coordinate.longitude) \
	X(plane_pitch_degrees, plane_pitch_degrees) \
	X(plane_touchdown_bank_degrees, plane_touchdown_bank_degrees) \
	X(plane_touchdown_heading_degrees_magnetic, plane_touchdown_heading_degrees_magnetic) \
	X(plane_touchdown_heading_degrees_true, plane_touchdown_heading_degrees_true) \
	X(plane_touchdown_latitude, plane_touchdown_coordinate.latitude) \
	X(plane_touchdown_longitude, plane_touchdown_coordinate.longitude) \
	X(plane_touchdown_pitch_degrees, plane_touchdown_pitch_degrees) \
	X(gps_ground_true_heading, gps_ground_true_heading) \
	X(gps_ground_true_track, gps_ground_true_track) \
	X(gps_position_lat, gps_position_coordinate.latitude) \
	X(gps_position_lon, gps_position_coordinate.longitude) \
	X(gyro_drift_error, gyro_drift_error) \
	X(heading_indicator, heading_indicator) \
	X(magnetic_compass, magnetic_compass) \
	X(ambient_wind_direction, ambient_wind_direction) \
	X(gear_position_0, gear_position_0) \
	X(gear_position_1, gear_position_1) \
	X(gear_position_2, gear_position_2) \
	X(gear_warning_0, gear_warning_0) \
	X(gear_warning_1, gear_warning_1) \
	X(gear_warning_2, gear_warning_2) \
	X(bleed_air_source_control_1, bleed_air_source_control_1) \
	X(bleed_air_source_control_2, bleed_air_source_control_2) \
	X(engine_type, engine_type) \
	X(turb_eng_ignition_switch_ex1_1, turb_eng_ignition_switch_ex1_1) \
	X(turb_eng_ignition_switch_ex1_2, turb_eng_ignition_switch_ex1_2) \
	X(fuel_cross_feed_l, fuel_cross_feed_l) \
	X(fuel_cross_feed_r, fuel_cross_feed_r) \
	X(surface_condition, surface_condition) \
	X(surface_type, surface_type) \
	X(pitot_heat_switch, pitot_heat_switch) \
	X(plane_touchdown_normal_velocity, plane_touchdown_normal_velocity) \
	X(vertical_speed, vertical_speed) \
	X(autopilot_altitude_lock_var, autopilot_altitude_lock_var) \
	X(plane_altitude, plane_altitude) \
	X(plane_alt_above_ground, plane_alt_above_ground) \
	X(radio_height, radio_height) \
	X(indicated_altitude, indicated_altitude) \
	X(indicated_altitude_calibrated, indicated_altitude_calibrated) \
	X(pressurization_cabin_altitude, pressurization_cabin_altitude) \
	X(autopilot_vertical_hold_var, autopilot_vertical_hold_var) \
	X(engine_control_select, engine_control_select) \
	X(fuel_selected_quantity_l, fuel_selected_quantity_l) \
	X(fuel_selected_quantity_r, fuel_selected_quantity_r) \
	X(fuel_total_quantity, fuel_total_quantity) \
	X(g_force, g_force) \
	X(general_eng_elapsed_time_1, general_eng_elapsed_time_1) \
	X(general_eng_elapsed_time_2, general_eng_elapsed_time_2) \
	X(ambient_pressure, ambient_pressure) \
	X(kohlsman_setting_hg, kohlsman_setting_hg) \
	X(autopilot_airspeed_hold_var, autopilot_airspeed_hold_var) \
	X(ground_velocity, ground_velocity) \
	X(airspeed_indicated, airspeed_indicated) \
	X(airspeed_true, airspeed_true) \
	X(ambient_wind_velocity, ambient_wind_velocity) \
	X(airspeed_mach, airspeed_mach) \
	X(light_states, light_states) \
	X(gps_ground_speed, gps_ground_speed) \
	X(gps_position_alt, gps_position_alt) \
	X(pressure_altitude, pressure_altitude) \
	X(ambient_visibility, ambient_visibility) \
	X(barometer_pressure, barometer_pressure) \
	X(kohlsman_setting_mb, kohlsman_setting_mb) \
	X(autopilot_mach_hold_var, autopilot_mach_hold_var) \
	X(auto_brake_switch_cb, auto_brake_switch_cb) \
	X(flaps_handle_index, flaps_handle_index) \
	X(flaps_num_handle_positions, flaps_num_handle_positions) \
	X(general_eng_throttle_managed_mode_1, general_eng_throttle_managed_mode_1) \
	X(general_eng_throttle_managed_mode_2, general_eng_throttle_managed_mode_2) \
	X(number_of_engines, number_of_engines) \
	X(turb_eng_vibration_1, turb_eng_vibration_1) \
	X(turb_eng_vibration_2, turb_eng_vibration_2) \
	X(gear_handle_position, gear_handle_position) \
	X(aileron_left_deflection_pct, aileron_left_deflection_pct) \
	X(aileron_right_deflection_pct, aileron_right_deflection_pct) \
	X(aileron_trim_pct, aileron_trim_pct) \
	X(elevator_deflection_pct, elevator_deflection_pct) \
	X(elevator_trim_pct, elevator_trim_pct) \
	X(rudder_deflection_pct, rudder_deflection_pct) \
	X(rudder_trim_pct, rudder_trim_pct) \
	X(spoilers_handle_position, spoilers_handle_position) \
	X(spoilers_left_position, spoilers_left_position) \
	X(spoilers_right_position, spoilers_right_position) \
	X(apu_pct_rpm, apu_pct_rpm) \
	X(apu_pct_starter, apu_pct_starter) \
	X(fuel_selected_quantity_percent_l, fuel_selected_quantity_percent_l) \
	X(fuel_selected_quantity_percent_r, fuel_selected_quantity_percent_r) \
	X(pitot_ice_pct, pitot_ice_pct) \
	X(autopilot_throttle_max_thrust, autopilot_throttle_max_thrust) \
	X(electrical_battery_estimated_capacity_pct, electrical_battery_estimated_capacity_pct) \
	X(general_eng_damage_percent_1, general_eng_damage_percent_1) \
	X(general_eng_damage_percent_2, general_eng_damage_percent_2) \
	X(general_eng_throttle_lever_position_1, general_eng_throttle_lever_position_1) \
	X(general_eng_throttle_lever_position_2, general_eng_throttle_lever_position_2) \
	X(turb_eng_n1_1, turb_eng_n1_1) \
	X(turb_eng_n1_2, turb_eng_n1_2) \
	X(turb_eng_n2_1, turb_eng_n2_1) \
	X(turb_eng_n2_2, turb_eng_n2_2) \
	X(brake_indicator, brake_indicator) \
	X(turb_eng_fuel_flow_pph_1, turb_eng_fuel_flow_pph_1) \
	X(turb_eng_fuel_flow_pph_2, turb_eng_fuel_flow_pph_2) \
	X(general_eng_fuel_used_since_start_1, general_eng_fuel_used_since_start_1) \
	X(general_eng_fuel_used_since_start_2, general_eng_fuel_used_since_start_2) \
	X(empty_weight, empty_weight) \
	X(total_weight, total_weight) \
	X(fuel_total_quantity_weight, fuel_total_quantity_weight) \
	X(fuel_weight_per_gallon, fuel_weight_per_gallon) \
	X(eng_hydraulic_pressure_1, eng_hydraulic_pressure_1) \
	X(eng_hydraulic_pressure_2, eng_hydraulic_pressure_2) \
	X(eng_oil_pressure_1, eng_oil_pressure_1) \
	X(eng_oil_pressure_2, eng_oil_pressure_2) \
	X(hydraulic_pressure_1, hydraulic_pressure_1) \
	X(hydraulic_pressure_2, hydraulic_pressure_2) \
	X(apu_bleed_pressure_received_by_engine, apu_bleed_pressure_received_by_engine) \
	X(turb_eng_bleed_air_1, turb_eng_bleed_air_1) \
	X(turb_eng_bleed_air_2, turb_eng_bleed_air_2) \
	X(wheel_rpm_0, wheel_rpm_0) \
	X(wheel_rpm_1, wheel_rpm_1) \
	X(wheel_rpm_2, wheel_rpm_2) \
	X(electrical_battery_voltage, electrical_battery_voltage)

#define TRIP_DATA_BOOL_FIELDS(X) \
	X(autopilot_airspeed_hold, 1, 0) \
	X(autopilot_alt_radio_mode, 1, 1) \
	X(autopilot_altitude_lock, 1, 2) \
	X(autopilot_approach_active, 1, 3) \
	X(autopilot_approach_captured, 1, 4) \
	X(autopilot_approach_hold, 1, 5) \
	X(autopilot_approach_is_localizer, 1, 6) \
	X(autopilot_avionics_managed, 1, 7) \
	X(autopilot_disengaged, 1, 8) \
	X(autopilot_flight_director_active, 1, 9) \
	X(autopilot_flight_level_change, 1, 10) \
	X(autopilot_glideslope_active, 1, 11) \
	X(autopilot_glideslope_arm, 1, 12) \
	X(autopilot_glideslope_hold, 1, 13) \
	X(autopilot_heading_lock, 1, 14) \
	X(autopilot_mach_hold, 1, 15) \
	X(autopilot_managed_speed_in_mach, 1, 16) \
	X(autopilot_managed_throttle_active, 1, 17) \
	X(autopilot_master, 1, 18) \
	X(autopilot_takeoff_power_active, 1, 19) \
	X(autopilot_throttle_arm, 1, 20) \
	X(autopilot_vertical_hold, 1, 21) \
	X(autobrakes_active, 1, 22) \
	X(brake_parking_indicator, 1, 23) \
	X(rejected_takeoff_brakes_active, 1, 24) \
	X(gear_damage_by_speed, 1, 25) \
	X(gear_is_on_ground_0, 1, 26) \
	X(gear_is_on_ground_1, 1, 27) \
	X(gear_is_on_ground_2, 1, 28) \
	X(gear_speed_exceeded, 1, 29) \
	X(aileron_trim_disabled, 1, 30) \
	X(elevator_trim_disabled, 1, 31) \
	X(flap_damage_by_speed, 2, 0) \
	X(flap_speed_exceeded, 2, 1) \
	X(rudder_trim_disabled, 2, 2) \
	X(spoilers_armed, 2, 3) \
	X(apu_generator_active, 2, 4) \
	X(apu_generator_switch, 2, 5) \
	X(apu_on_fire_detected, 2, 6) \
	X(apu_switch, 2, 7) \
	X(bleed_air_apu, 2, 8) \
	X(electrical_master_battery, 2, 9) \
	X(external_power_available, 2, 10) \
	X(external_power_connection_on, 2, 11) \
	X(external_power_on, 2, 12) \
	X(bleed_air_engine_1, 2, 13) \
	X(bleed_air_engine_2, 2, 14) \
	X(eng_anti_ice_1, 2, 15) \
	X(eng_anti_ice_2, 2, 16) \
	X(eng_combustion_1, 2, 17) \
	X(eng_combustion_2, 2, 18) \
	X(eng_failed_1, 2, 19) \
	X(eng_failed_2, 2, 20) \
	X(eng_on_fire_1, 2, 21) \
	X(eng_on_fire_2, 2, 22) \
	X(general_eng_fire_detected_1, 2, 23) \
	X(general_eng_fire_detected_2, 2, 24) \
	X(general_eng_fuel_valve_1, 2, 25) \
	X(general_eng_fuel_valve_2, 2, 26) \
	X(general_eng_generator_active_1, 2, 27) \
	X(general_eng_generator_active_2, 2, 28) \
	X(general_eng_generator_switch_1, 2, 29) \
	X(general_eng_generator_switch_2, 2, 30) \
	X(general_eng_master_alternator, 2, 31) \
	X(general_eng_reverse_thrust_engaged, 3, 0) \
	X(general_eng_starter_1, 3, 1) \
	X(general_eng_starter_2, 3, 2) \
	X(general_eng_starter_active_1, 3, 3) \
	X(general_eng_starter_active_2, 3, 4) \
	X(master_ignition_switch, 3, 5) \
	X(turb_eng_fuel_available_1, 3, 6) \
	X(turb_eng_fuel_available_2, 3, 7) \
	X(turb_eng_is_igniting_1, 3, 8) \
	X(turb_eng_is_igniting_2, 3, 9) \
	X(fuel_transfer_pump_on_l, 3, 10) \
	X(fuel_transfer_pump_on_r, 3, 11) \
	X(on_any_runway, 3, 12) \
	X(plane_in_parking_state, 3, 13) \
	X(autothrottle_active, 3, 14) \
	X(avionics_master_switch, 3, 15) \
	X(cabin_no_smoking_alert_switch, 3, 16) \
	X(cabin_seatbelts_alert_switch, 3, 17) \
	X(gpws_system_active, 3, 18) \
	X(gpws_warning, 3, 19) \
	X(overspeed_warning, 3, 20) \
	X(pitot_heat, 3, 21) \
	X(stall_warning, 3, 22) \
	X(structural_deice_switch, 3, 23) \
	X(hydraulic_switch, 3, 24) \
	X(warning_fuel, 3, 25) \
	X(warning_low_height, 3, 26) \
	X(warning_oil_pressure, 3, 27) \
	X(warning_vacuum, 3, 28) \
	X(warning_voltage, 3, 29) \
	X(sim_on_ground, 3, 30) \
	X(kohlsman_setting_std, 3, 31)

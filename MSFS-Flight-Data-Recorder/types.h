#pragma once

#include <algorithm>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <Windows.h>
#include "sqlite3.h"

#define Q_DB_LENGTH 200
#define DB_SAMPLING_INTERVAL 0.3
#define DATABASE_NAME "flight_data"
#define V_PI 3.14159265358979323846
#define M_2_FT 3.2808399
#define EARTHRADIUSKM 6371.0

class DATETIME {
public:
	double year;
	double month_of_year;
	double day_of_month;
	double day_of_week;
	double time_day;
	double timezone_offset;

	DATETIME() { clear(); }
	~DATETIME() { clear(); }

	void clear() {
		year = 0;
		month_of_year = 0;
		day_of_month = 0;
		time_day = 0;
		day_of_week = 0;
		timezone_offset = 0;
	}

	std::string format_date_time() {
		int hour = (int)time_day / 3600;
		int minute = ((int)time_day - 3600 * hour) / 60;
		double second = time_day - 3600 * hour - 60 * minute;

		char sign = '+';
		if (timezone_offset < 0)
			sign = '-';
		double timezone = abs(timezone_offset);
		timezone /= 3600;
		int timezone_hour = (int)timezone;
		int timezone_minute = (int)((timezone - timezone_hour) * 60);

		char ret[32];
		memset(ret, 0, sizeof(ret));
		snprintf(ret, sizeof(ret), "%04.0f-%02.0f-%02.0fT%02d:%02d:%06.3f%c%02d:%02d_%1.0f",
			year, month_of_year, day_of_month, hour, minute, second,
			sign, timezone_hour, timezone_minute, day_of_week);
		return std::string(ret);
	}
};

class COORDINATE {
public:
	enum COORDINATE_CAT {
		LATITUDE,
		LONGITUDE,
	};

	double latitude;
	double longitude;

	COORDINATE() { clear(); }
	~COORDINATE() { clear(); }

	void clear() {
		latitude = 360;
		longitude = 360;
	}

	std::string coordinate_decimal_to_dms(enum COORDINATE_CAT cat) {
		double coordinate = 0;
		char tmp1 = 'T';
		switch (cat) {
		case LATITUDE:
			coordinate = latitude;
			tmp1 = (coordinate < 0) ? 'S' : 'N';
			break;
		case LONGITUDE:
			coordinate = longitude;
			tmp1 = (coordinate < 0) ? 'W' : 'E';
			break;
		default:
			break;
		}
		coordinate = abs(coordinate);
		int degree = (int)coordinate;
		coordinate -= degree;
		coordinate *= 60;
		int minute = (int)coordinate;
		coordinate -= minute;
		coordinate *= 60;
		int second = (int)coordinate;
		char ret[12];
		memset(ret, 0, sizeof(ret));
		snprintf(ret, sizeof(ret), "%03d %02d %02d%c", degree, minute, second, tmp1);
		return std::string(ret);
	}

	double distanceInKm2Coordinate(COORDINATE loc) {
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double dPhy = (loc.latitude - latitude) * V_PI / 180.0;
		double dLambda = (loc.longitude - longitude) * V_PI / 180.0;
		double a = pow(sin(dPhy / 2), 2) + cos(phy1) * cos(phy2) * pow(sin(dLambda / 2), 2);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));
		return EARTHRADIUSKM * c;
	}

	double bearing2Coordinate(COORDINATE loc) {
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double lambda1 = longitude * V_PI / 180.0;
		double lambda2 = loc.longitude * V_PI / 180.0;
		double y = sin(lambda2 - lambda1) * cos(phy2);
		double x = cos(phy1) * sin(phy2) - sin(phy1) * cos(phy2) * cos(lambda2 - lambda1);
		double theta = atan2(y, x);
		double ret = theta * 180 / V_PI;
		if (ret <= 0)
			ret += 360;
		return ret;
	}

	COORDINATE destinationWithDistanceAndBearing(double distance, double bearing) {
		COORDINATE ret;
		double phy = latitude * V_PI / 180.0;
		double lambda = longitude * V_PI / 180.0;
		double theta = bearing * V_PI / 180.0;
		ret.latitude = asin(sin(phy) * cos(distance / EARTHRADIUSKM) + cos(phy) * sin(distance / EARTHRADIUSKM) * cos(theta));
		ret.longitude = lambda + atan2(sin(theta) * sin(distance / EARTHRADIUSKM) * cos(phy), cos(distance / EARTHRADIUSKM) - sin(phy) * sin(phy));
		ret.latitude *= 180.0 / V_PI;
		ret.longitude *= 180.0 / V_PI;
		return ret;
	}

	COORDINATE intersectionCoordinate(double bearing1, COORDINATE loc, double bearing2) {
		COORDINATE ret;
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double lambda1 = longitude * V_PI / 180.0;
		double lambda2 = loc.longitude * V_PI / 180.0;
		double theta1 = bearing1 * V_PI / 180.0;
		double theta2 = bearing2 * V_PI / 180.0;

		double delta12 = 2 * asin(sqrt((pow(sin((phy2 - phy1) / 2), 2) + cos(phy1) * cos(phy2) * pow(sin((lambda2 - lambda1) / 2), 2))));
		double thetaa = acos((sin(phy2) - sin(phy1) * cos(delta12)) / (sin(delta12) * cos(phy1)));
		double thetab = acos((sin(phy1) - sin(phy2) * cos(delta12)) / (sin(delta12) * cos(phy2)));
		double theta12 = 0;
		double theta21 = 0;
		if (sin(lambda2 - lambda1) > 0) {
			theta12 = thetaa;
			theta21 = 2 * V_PI - thetab;
		} else {
			theta12 = 2 * V_PI - thetaa;
			theta21 = thetab;
		}
		double alpha1 = theta1 - theta12;
		double alpha2 = theta21 - theta2;
		if ((sin(alpha1) == 0 && sin(alpha2) == 0) || sin(alpha1) * sin(alpha2) < 0) {
			ret.clear();
		} else {
			double alpha3 = acos(-1 * cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2) * cos(delta12));
			double delta1 = atan2(sin(delta12) * sin(alpha1) * sin(alpha2), cos(alpha2) + cos(alpha1) * cos(alpha3));
			double phy3 = asin(sin(phy1) * cos(delta1) + cos(phy1) * sin(delta1) * cos(theta1));
			double delta_lambda1 = atan2(sin(theta1) * sin(delta1) * cos(phy1), cos(delta1) - sin(phy1) * sin(phy3));
			double lambda3 = lambda1 + delta_lambda1;
			ret.latitude = phy3 * 180.0 / V_PI;
			ret.longitude = lambda3 * 180.0 / V_PI;
		}
		return ret;
	}
};

class RUNWAY {
public:
	char placeholder[4];
	float length;
	float width;
	float heading;
	int numbers[2];
	int designators[2];
	COORDINATE coordinate;
	COORDINATE start_points[2];

	RUNWAY() { clear(); }
	~RUNWAY() { clear(); }

	void clear() {
		length = 0;
		width = 0;
		heading = 0;
		for (int i = 0; i < (int)(sizeof(numbers) / sizeof(int)); i++)
			numbers[i] = -1;
		for (int i = 0; i < (int)(sizeof(designators) / sizeof(int)); i++)
			designators[i] = -1;
		coordinate.clear();
		for (int i = 0; i < 2; i++)
			start_points[i].clear();
	}

	std::string runway_code_generator(bool is_primary) {
		int runway_number = is_primary ? numbers[0] : numbers[1];
		int runway_designator = is_primary ? designators[0] : designators[1];
		char designator = 0;
		switch (runway_designator) {
		case 1: designator = 'L'; break;
		case 2: designator = 'R'; break;
		case 3: designator = 'C'; break;
		case 4: designator = 'W'; break;
		case 5: designator = 'A'; break;
		case 6: designator = 'B'; break;
		default: break;
		}
		std::vector<std::string> numbers_dir = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
		char ret[4];
		memset(ret, 0, sizeof(ret));
		if (runway_number > 0 && runway_number <= 36)
			snprintf(ret, sizeof(ret), "%02d%c", runway_number, designator);
		else if (runway_number >= 37 && runway_number <= 44)
			snprintf(ret, sizeof(ret), "%s%c", numbers_dir[runway_number - 37].c_str(), designator);
		return std::string(ret);
	}
};

struct RUNWAY_OPERATION {
	int index = -1;
	bool is_primary = TRUE;
	double diff_bearing_pos = 0;
	double diff_bearing_tra = 0;
	double distances[2];
	double distances_percent[2];
};

class AIRPORT {
public:
	char name[64];
	float magvar;
	int n_runways;
	RUNWAY* runways;
	struct RUNWAY_OPERATION runway_act;
	char icao[5];
	char region[3];

	AIRPORT() {
		runways = NULL;
		clear();
	}

	~AIRPORT() { clear(); }

	void clear() {
		memset(icao, 0, sizeof(icao));
		memset(region, 0, sizeof(region));
		memset(name, 0, sizeof(name));
		magvar = 0;
		n_runways = 0;
		if (runways != NULL)
			free(runways);
		runways = NULL;
		runway_act.index = -1;
		runway_act.is_primary = TRUE;
		runway_act.diff_bearing_pos = 0;
		runway_act.diff_bearing_tra = 0;
		for (int i = 0; i < (int)(sizeof(runway_act.distances) / sizeof(double)); i++)
			runway_act.distances[i] = -1;
		for (int i = 0; i < (int)(sizeof(runway_act.distances_percent) / sizeof(double)); i++)
			runway_act.distances_percent[i] = -1;
	}

	void copy(AIRPORT* src) {
		memcpy(name, src->name, sizeof(src->name));
		memcpy(icao, src->icao, sizeof(src->icao));
		memcpy(region, src->region, sizeof(src->region));
		magvar = src->magvar;
		n_runways = src->n_runways;
		if (src->runways != NULL) {
			runways = (RUNWAY*)malloc(sizeof(RUNWAY) * n_runways);
			memcpy(runways, src->runways, sizeof(RUNWAY) * n_runways);
		}
		runway_act = src->runway_act;
	}

	std::string runway_code_generator() {
		if (runway_act.index > -1)
			return runways[runway_act.index].runway_code_generator(runway_act.is_primary);
		return "";
	}
};

struct FLIGHT_DATA {
	int heading = 0;
	int altitude = 0;
	int speed = 0;
	int vertical_speed = 0;
	double g_force = 1;
	double pitch = 0;
	double bank = 0;
	COORDINATE coordinate;
	DATETIME time_zulu;
	DATETIME time_local;
};

struct TOUCHDOWN_DATA {
	struct FLIGHT_DATA flight_data;
	AIRPORT airport;
	struct TOUCHDOWN_DATA* next = NULL;
};

struct EVENT_DB {
	char event[32];
	DATETIME time_zulu;
	DATETIME time_local;
	struct EVENT_DB* next;
};

// Forward declaration — full definition in simconnect_defs.h
struct FLIGHT_DATA_RECORD;

struct STATUS {
	bool in_sim = FALSE;
	bool sim_running = FALSE;
	bool paused = FALSE;
	bool recording = FALSE;
	bool quit = FALSE;
	struct FLIGHT_DATA_RECORD* q_data_db_start = NULL;
	struct FLIGHT_DATA_RECORD* q_data_db_end = NULL;
	struct FLIGHT_DATA_RECORD* q_data_end = NULL;
	struct FLIGHT_DATA_RECORD* q_data_last = NULL;
	int q_data_db_length = 0;
	struct EVENT_DB* q_event_start = NULL;
	struct EVENT_DB* q_event_end = NULL;
	HANDLE hSimConnect = NULL;
	sqlite3* sql = NULL;
	std::mutex mutex_db_commit;
	int id_trip = -1;
	bool airborne = FALSE;
	TOUCHDOWN_DATA* touchdown_data = NULL;
	TOUCHDOWN_DATA* touchdown_data_end = NULL;
	FLIGHT_DATA data;
	COORDINATE loc_dh;
	AIRPORT departure;
	AIRPORT destination;
	void* gui_context = nullptr;
};

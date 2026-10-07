#include "engine_power.h"
#include "simconnect_defs.h"

#include <algorithm>
#include <climits>

namespace {

// Each ENGINE TYPE that shows engine power: what its speed/load are.
struct EngineTypeEntry {
	int engineType;
	EnginePowerSpec spec;
};

const EngineTypeEntry ENGINE_TYPES[] = {
	{ 0, { { TRIP_ENGINE_general_eng_rpm, "RPM", "rpm", 0, "RPM", 0 },
	       { TRIP_ENGINE_recip_eng_manifold_pressure, "MP", "inHg", 1, "Manifold Pressure (inHg)", 0 } } },
	{ 1, { { TRIP_ENGINE_turb_eng_n1, "N1", "%", 1, "N1 (%)", 110 },
	       { TRIP_ENGINE_turb_eng_n2, "N2", "%", 1, "N2 (%)", 110 } } },
	{ 3, { { TRIP_ENGINE_turb_eng_n1, "N1", "%", 1, "N1 (%)", 110 },
	       { TRIP_ENGINE_turb_eng_max_torque_percent, "Torque", "%", 1, "Torque (%)", 0 } } },
	{ 5, { { TRIP_ENGINE_prop_rpm, "Prop RPM", "rpm", 0, "Prop RPM", 0 },
	       { TRIP_ENGINE_turb_eng_max_torque_percent, "Torque", "%", 1, "Torque (%)", 0 } } },
};

// A SimVar as an int, or fallback for one no int holds (NaN, out of range),
// which a cast would turn into undefined behavior.
int simVarInt(double value, int fallback) {
	return value >= INT_MIN && value <= INT_MAX ? (int)value : fallback; // NaN fails both
}

}

int engineCount(const FLIGHT_DATA_RECORD& r) {
	return std::clamp(simVarInt(r.number_of_engines, 0), 0, SIM_ENGINE_INDEXES);
}

bool anyEngineCombusting(const FLIGHT_DATA_RECORD& r) {
	return std::any_of(r.eng_combustion, r.eng_combustion + engineCount(r), [](double c) { return c != 0; });
}

const EnginePowerSpec* enginePowerSpec(int engineType) {
	for (const EngineTypeEntry& entry : ENGINE_TYPES)
		if (entry.engineType == engineType)
			return &entry.spec;
	return nullptr;
}

#include "engine_power.h"
#include "simconnect_defs.h"

#include <algorithm>
#include <climits>

namespace {

const EnginePowerSpec PISTON = { TRIP_ENGINE_general_eng_rpm, TRIP_ENGINE_prop_rpm, "RPM", "Prop RPM", "rpm", 0, "RPM", 0 };
const EnginePowerSpec TURBINE = { TRIP_ENGINE_turb_eng_n1, TRIP_ENGINE_turb_eng_n2, "N1", "N2", "%", 1, "N1 / N2 (%)", 110 };

// Each ENGINE TYPE that shows engine power: its N1/N2 lines.
struct EngineTypeEntry {
	int engineType;
	const EnginePowerSpec* spec;
};

const EngineTypeEntry ENGINE_TYPES[] = {
	{ 0, &PISTON },
	{ 1, &TURBINE },  // jet
	{ 3, &TURBINE },  // helo turbine
	{ 5, &TURBINE },  // turboprop
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
			return entry.spec;
	return nullptr;
}

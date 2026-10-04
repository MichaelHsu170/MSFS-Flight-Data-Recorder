#include "engine_power.h"
#include "simconnect_defs.h"

#include <algorithm>
#include <climits>
#include <cstring>

namespace {

using EngineVar = double (FLIGHT_DATA_RECORD::*)[MAX_ENGINES];

// Each ENGINE TYPE that records engine power: what its speed/load are and
// the FLIGHT_DATA_RECORD SimVars they come from.
struct EngineTypeEntry {
	int engineType;
	EnginePowerSpec spec;
	EngineVar speed;
	EngineVar load;
};

const EngineTypeEntry ENGINE_TYPES[] = {
	{ 0, { { "RPM", "rpm", 0, "RPM", 0 }, { "MP", "inHg", 1, "Manifold Pressure (inHg)", 0 } },
		&FLIGHT_DATA_RECORD::general_eng_rpm, &FLIGHT_DATA_RECORD::recip_eng_manifold_pressure },
	{ 1, { { "N1", "%", 1, "N1 (%)", 110 }, { "N2", "%", 1, "N2 (%)", 110 } },
		&FLIGHT_DATA_RECORD::turb_eng_n1, &FLIGHT_DATA_RECORD::turb_eng_n2 },
	{ 3, { { "N1", "%", 1, "N1 (%)", 110 }, { "Torque", "%", 1, "Torque (%)", 0 } },
		&FLIGHT_DATA_RECORD::turb_eng_n1, &FLIGHT_DATA_RECORD::turb_eng_max_torque_percent },
	{ 5, { { "Prop RPM", "rpm", 0, "Prop RPM", 0 }, { "Torque", "%", 1, "Torque (%)", 0 } },
		&FLIGHT_DATA_RECORD::prop_rpm, &FLIGHT_DATA_RECORD::turb_eng_max_torque_percent },
};

const EngineTypeEntry* findEngineType(int engineType) {
	for (const EngineTypeEntry& entry : ENGINE_TYPES)
		if (entry.engineType == engineType)
			return &entry;
	return nullptr;
}

// A SimVar as an int, or fallback for one no int holds (NaN, out of range),
// which a cast would turn into undefined behavior.
int simVarInt(double value, int fallback) {
	return value >= INT_MIN && value <= INT_MAX ? (int)value : fallback; // NaN fails both
}

}

const EnginePowerSpec* enginePowerSpec(int engineType) {
	const EngineTypeEntry* entry = findEngineType(engineType);
	return entry ? &entry->spec : nullptr;
}

EnginePower enginePowerFromRecord(const FLIGHT_DATA_RECORD& r) {
	EnginePower power;
	power.engineType = simVarInt(r.engine_type, -1);
	const EngineTypeEntry* entry = findEngineType(power.engineType);
	if (!entry)
		return power;
	power.count = std::clamp(simVarInt(r.number_of_engines, 0), 0, MAX_ENGINES);
	for (int i = 0; i < power.count; ++i) {
		power.speed[i] = (float)(r.*entry->speed)[i];
		power.load[i] = (float)(r.*entry->load)[i];
	}
	return power;
}

std::string_view packEngineValues(const std::array<float, MAX_ENGINES>& values, int count) {
	return { reinterpret_cast<const char*>(values.data()), std::clamp(count, 0, MAX_ENGINES) * sizeof(float) };
}

int unpackEngineValues(const void* blob, int bytes, std::array<float, MAX_ENGINES>& out) {
	const int count = blob ? std::clamp(bytes / (int)sizeof(float), 0, MAX_ENGINES) : 0;
	out.fill(0.0f);
	if (count > 0)
		std::memcpy(out.data(), blob, count * sizeof(float));
	return count;
}

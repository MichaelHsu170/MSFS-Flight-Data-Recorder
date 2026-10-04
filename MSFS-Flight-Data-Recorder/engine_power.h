#pragma once

#include <array>
#include <string_view>

struct FLIGHT_DATA_RECORD;

// Engine power: one "speed" and one "load" value per engine, whose meaning
// depends on the aircraft's ENGINE TYPE SimVar -- N1/N2 for a jet, RPM and
// manifold pressure for a piston, and so on (see enginePowerSpec()). Recorded
// to trip_data.engine_speed/engine_load and shown in the first chart and the
// Data Table.

// The documented maximum of the NUMBER OF ENGINES SimVar. FLIGHT_DATA_RECORD
// receives every engine SimVar used here for engines 1..MAX_ENGINES.
constexpr int MAX_ENGINES = 4;

struct EngineQuantity {
	const char* label;      // "N1" -- shown as "N1 #2" per engine
	const char* unit;       // "%"
	int decimals;
	const char* axisTitle;  // "N1 (%)"
	double axisMax;         // fixed axis max unless the data goes past it, or 0: niceAxisMax()
};

struct EnginePowerSpec {
	EngineQuantity speed;
	EngineQuantity load;
};

// What speed and load mean for an ENGINE TYPE value (0 piston, 1 jet,
// 3 helo turbine, 5 turboprop); nullptr for a type that records neither
// (2 none, 4 unsupported, 6 electric, or anything else).
const EnginePowerSpec* enginePowerSpec(int engineType);

struct EnginePower {
	int engineType = -1;
	// Engines with values in speed/load; 0 means not recorded (no spec for
	// engineType, no engines, or a trip recorded before this existed).
	int count = 0;
	std::array<float, MAX_ENGINES> speed{};
	std::array<float, MAX_ENGINES> load{};
};

// Picks the SimVars that enginePowerSpec(r.engine_type) names, for engines
// 1..NUMBER OF ENGINES (clamped to 0..MAX_ENGINES). An ENGINE TYPE that no
// int holds (NaN, out of range) reads as -1, such an engine count as 0.
EnginePower enginePowerFromRecord(const FLIGHT_DATA_RECORD& r);

// trip_data.engine_speed/engine_load store engines 1..count as consecutive
// float32 values in native (x64: little-endian) byte order, i.e. the raw
// bytes of EnginePower::speed/load; NULL = not recorded.
//
// Packs values' engines 1..count (clamped to 0..MAX_ENGINES) as such a blob:
// a view of values' bytes, empty when count is 0, which is stored as NULL.
std::string_view packEngineValues(const std::array<float, MAX_ENGINES>& values, int count);
// Unpacks one into out (engines past its count set to 0) and returns its
// engine count (bytes / 4, clamped to MAX_ENGINES); 0 for a NULL or empty blob.
int unpackEngineValues(const void* blob, int bytes, std::array<float, MAX_ENGINES>& out);

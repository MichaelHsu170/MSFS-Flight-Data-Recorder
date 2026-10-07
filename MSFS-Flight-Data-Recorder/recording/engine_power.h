#pragma once

#include "trip_data_fields.h"

struct FLIGHT_DATA_RECORD;

// Engine power as the first chart shows it: one "speed" and one "load" value
// per engine, whose meaning depends on the aircraft's ENGINE TYPE SimVar --
// N1/N2 for a jet, RPM and manifold pressure for a piston, and so on (see
// enginePowerSpec()). Each is one of the per-engine values every sample
// records to trip_engine_data (TRIP_ENGINE_FIELDS).

// The engine indexes FLIGHT_DATA_RECORD receives every ENGINES SimVar for:
// "SimVar:1" to "SimVar:16", the index range the MSFS SimVar documentation
// gives. A sample records engines 1..engineCount() of them.
constexpr int SIM_ENGINE_INDEXES = 16;

struct EngineQuantity {
	TripEngineField field;  // the trip_engine_data value it is
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
// 3 helo turbine, 5 turboprop); nullptr for a type that shows neither
// (2 none, 4 unsupported, 6 electric, or anything else).
const EnginePowerSpec* enginePowerSpec(int engineType);

// The aircraft's NUMBER OF ENGINES, clamped to 0..SIM_ENGINE_INDEXES; 0 for
// a value no int holds (NaN, out of range). The engines a sample records.
int engineCount(const FLIGHT_DATA_RECORD& r);

// Whether ENG COMBUSTION is set for any of engines 1..engineCount(r), which
// is what keeps a trip recording (flight_phase.cpp).
bool anyEngineCombusting(const FLIGHT_DATA_RECORD& r);

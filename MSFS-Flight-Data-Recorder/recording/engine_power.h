#pragma once

#include "trip_data_fields.h"

struct FLIGHT_DATA_RECORD;

// Engine power as the first chart shows it: an N1 and an N2 line per engine,
// or their equivalent for the aircraft's ENGINE TYPE SimVar (see
// enginePowerSpec()). N1 is the core speed, which combustion keeps turning;
// N2 is the second spool, which on a turboprop or helicopter turbine is the
// power turbine driving the prop or rotor. A piston's equivalents are its
// crankshaft RPM and prop RPM. Each is one of the per-engine values every
// sample records to trip_engine_data (TRIP_ENGINE_FIELDS).

// The engine indexes FLIGHT_DATA_RECORD receives every ENGINES SimVar for:
// "SimVar:1" to "SimVar:16", the index range the MSFS SimVar documentation
// gives. A sample records engines 1..engineCount() of them.
constexpr int SIM_ENGINE_INDEXES = 16;

// An engine type's N1 and N2 lines, which share a unit and one Y axis.
struct EnginePowerSpec {
	TripEngineField n1Field;  // the trip_engine_data values they are
	TripEngineField n2Field;
	const char* n1Label;      // "N1" -- shown as "N1 #2" per engine
	const char* n2Label;
	const char* unit;         // "%"
	int decimals;
	const char* axisTitle;    // "N1 / N2 (%)"
	double axisMax;           // fixed axis max unless the data goes past it, or 0: niceAxisMax()
};

// The N1/N2 lines for an ENGINE TYPE value (0 piston, 1 jet, 3 helo turbine,
// 5 turboprop); nullptr for a type that shows none (2 none, 4 unsupported,
// 6 electric, or anything else).
const EnginePowerSpec* enginePowerSpec(int engineType);

// The aircraft's NUMBER OF ENGINES, clamped to 0..SIM_ENGINE_INDEXES; 0 for
// a value no int holds (NaN, out of range). The engines a sample records.
int engineCount(const FLIGHT_DATA_RECORD& r);

// Whether ENG COMBUSTION is set for any of engines 1..engineCount(r), which
// is what keeps a trip recording (flight_phase.cpp).
bool anyEngineCombusting(const FLIGHT_DATA_RECORD& r);

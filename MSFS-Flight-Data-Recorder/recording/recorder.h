#pragma once

#include "types.h"
#include "simconnect_defs.h"

// Flushes the events the flood filter still holds back, then signals both
// write workers (samples, then events) to drain and exit and joins each.
// Callers must call this before closing/nulling status->sql.
void wait_for_db_writers(struct STATUS* status);

// SimConnect dispatch callback; pContext is the STATUS. Handles connect/quit,
// sim stop (ends a recording trip), cockpit events (through the flood filter),
// flight samples (flight_on_sample()), the airport lookup's responses and
// SimConnect exceptions (logged, and passed to the airport lookup).
// Never throws: any exception is logged as a warning and dropped.
void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext);

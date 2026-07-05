#pragma once

#include "types.h"
#include "simconnect_defs.h"

void add_flight_definition(HANDLE hSimConnect);
void add_client_events(HANDLE hSimConnect);

void stop_recording(struct STATUS* status);

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext);

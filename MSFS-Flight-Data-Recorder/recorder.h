#pragma once

#include "types.h"
#include "simconnect_defs.h"

std::string trim(const std::string& source);

void add_flight_definition(HANDLE hSimConnect);
void add_client_events(HANDLE hSimConnect);

void stop_recording(struct STATUS* status);
void displayTouchdownData(struct TOUCHDOWN_DATA* tmp);

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext);

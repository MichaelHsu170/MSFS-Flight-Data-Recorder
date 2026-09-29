#pragma once

#include "types.h"
#include "simconnect_defs.h"

void wait_for_db_writers(struct STATUS* status);

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext);

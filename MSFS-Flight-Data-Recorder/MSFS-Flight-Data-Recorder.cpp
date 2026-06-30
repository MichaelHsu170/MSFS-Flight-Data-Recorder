#include "recorder.h"
#include "db.h"

int main() {
	struct STATUS status;
	SimConnect_Open(&status.hSimConnect, "Flight Data Recorder", NULL, 0, 0, SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_SIM, "Sim");
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_PAUSE, "Pause");
	SimConnect_SubscribeToSystemEvent(status.hSimConnect, EVENT_CRASHED, "Crashed");
	add_client_events(status.hSimConnect);
	add_flight_definition(status.hSimConnect);

	connect_db(&status);
	while (!status.quit)
		SimConnect_CallDispatch(status.hSimConnect, MyDispatchProc, &status);
	SimConnect_Close(status.hSimConnect);
	sqlite3_close_v2(status.sql);

	return 0;
}

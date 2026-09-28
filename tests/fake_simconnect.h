#pragma once

#include "simconnect_defs.h"

#include <string>
#include <vector>

// Test double for SimConnect.lib (see fake_simconnect.cpp): implements every
// SimConnect_* function the app calls, records each outgoing call, and hands
// queued inbound packets to the dispatch callback from SimConnect_CallDispatch
// -- so the app's real recorder code can be driven with made-up sim data.
namespace FakeSim {

struct DataDefinition {
	DWORD defineId;
	std::string datumName;
	std::string unitsName;
	SIMCONNECT_DATATYPE datumType;
};

struct MappedEvent {
	DWORD eventId;
	std::string simEventName;
};

struct FacilityDataRequest {
	DWORD sendId;
	std::string icao;
	std::string region;
};

struct State {
	// Behavior switches.
	bool openFails = false;
	bool dispatchFails = false;

	// Recorded outgoing calls.
	int openCalls = 0;
	int closeCalls = 0;
	DWORD lastSendId = 0;
	std::vector<std::string> systemEvents;
	std::vector<MappedEvent> mappedEvents;
	std::vector<DWORD> notificationGroupEvents;
	std::vector<DataDefinition> dataDefinitions;
	int dataRequests = 0;
	std::vector<std::string> facilityDefinitionFields;
	std::vector<DWORD> facilitiesListRequests; // SendID of each request
	std::vector<FacilityDataRequest> facilityDataRequests;

	// Inbound packets, delivered in order by SimConnect_CallDispatch.
	std::vector<std::vector<char>> inbound;
};

State& state();
void reset();
void queue(std::vector<char> packet);

}

#include "fake_simconnect.h"

// Every SimConnect_* call gets the next SendID, like the real client library
// does, so SimConnect_GetLastSentPacketID() can be matched against an
// exception's dwSendID.
namespace {

HANDLE const kHandle = reinterpret_cast<HANDLE>(static_cast<intptr_t>(0x5157));

DWORD nextSendId() {
	return ++FakeSim::state().lastSendId;
}

}

namespace FakeSim {

State& state() {
	static State s;
	return s;
}

void reset() {
	state() = State();
}

void queue(std::vector<char> packet) {
	state().inbound.push_back(std::move(packet));
}

}

SIMCONNECTAPI SimConnect_Open(HANDLE* phSimConnect, LPCSTR, HWND, DWORD, HANDLE, DWORD) {
	FakeSim::state().openCalls++;
	if (FakeSim::state().openFails)
		return E_FAIL;
	*phSimConnect = kHandle;
	return S_OK;
}

SIMCONNECTAPI SimConnect_Close(HANDLE) {
	FakeSim::state().closeCalls++;
	return S_OK;
}

SIMCONNECTAPI SimConnect_CallDispatch(HANDLE, DispatchProc pfcnDispatch, void* pContext) {
	if (FakeSim::state().dispatchFails)
		return E_FAIL;
	// Packets queued while dispatching (none today) would be delivered on the
	// next call, like new messages arriving after this dispatch returned.
	std::vector<std::vector<char>> packets;
	packets.swap(FakeSim::state().inbound);
	for (std::vector<char>& packet : packets)
		pfcnDispatch(reinterpret_cast<SIMCONNECT_RECV*>(packet.data()), static_cast<DWORD>(packet.size()), pContext);
	return S_OK;
}

SIMCONNECTAPI SimConnect_GetLastSentPacketID(HANDLE, DWORD* pdwError) {
	*pdwError = FakeSim::state().lastSendId;
	return S_OK;
}

SIMCONNECTAPI SimConnect_SubscribeToSystemEvent(HANDLE, SIMCONNECT_CLIENT_EVENT_ID, const char* SystemEventName) {
	nextSendId();
	FakeSim::state().systemEvents.push_back(SystemEventName);
	return S_OK;
}

SIMCONNECTAPI SimConnect_MapClientEventToSimEvent(HANDLE, SIMCONNECT_CLIENT_EVENT_ID EventID, const char* EventName) {
	nextSendId();
	FakeSim::state().mappedEvents.push_back({ static_cast<DWORD>(EventID), EventName });
	return S_OK;
}

SIMCONNECTAPI SimConnect_AddClientEventToNotificationGroup(HANDLE, SIMCONNECT_NOTIFICATION_GROUP_ID, SIMCONNECT_CLIENT_EVENT_ID EventID, BOOL) {
	nextSendId();
	FakeSim::state().notificationGroupEvents.push_back(static_cast<DWORD>(EventID));
	return S_OK;
}

SIMCONNECTAPI SimConnect_AddToDataDefinition(HANDLE, SIMCONNECT_DATA_DEFINITION_ID DefineID, const char* DatumName, const char* UnitsName, SIMCONNECT_DATATYPE DatumType, float, DWORD) {
	nextSendId();
	FakeSim::state().dataDefinitions.push_back({ static_cast<DWORD>(DefineID), DatumName, UnitsName ? UnitsName : "", DatumType });
	return S_OK;
}

SIMCONNECTAPI SimConnect_RequestDataOnSimObject(HANDLE, SIMCONNECT_DATA_REQUEST_ID, SIMCONNECT_DATA_DEFINITION_ID, SIMCONNECT_OBJECT_ID, SIMCONNECT_PERIOD, SIMCONNECT_DATA_REQUEST_FLAG, DWORD, DWORD, DWORD) {
	nextSendId();
	FakeSim::state().dataRequests++;
	return S_OK;
}

SIMCONNECTAPI SimConnect_AddToFacilityDefinition(HANDLE, SIMCONNECT_DATA_DEFINITION_ID, const char* FieldName) {
	nextSendId();
	FakeSim::state().facilityDefinitionFields.push_back(FieldName);
	return S_OK;
}

SIMCONNECTAPI SimConnect_RequestFacilitiesList_EX1(HANDLE, SIMCONNECT_FACILITY_LIST_TYPE, SIMCONNECT_DATA_REQUEST_ID) {
	FakeSim::state().facilitiesListRequests.push_back(nextSendId());
	return S_OK;
}

SIMCONNECTAPI SimConnect_RequestFacilityData_EX1(HANDLE, SIMCONNECT_DATA_DEFINITION_ID, SIMCONNECT_DATA_REQUEST_ID, const char* ICAO, const char* Region, char) {
	FakeSim::state().facilityDataRequests.push_back({ nextSendId(), ICAO, Region ? Region : "" });
	return S_OK;
}

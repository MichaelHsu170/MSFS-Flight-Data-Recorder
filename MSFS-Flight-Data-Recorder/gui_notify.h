#pragma once

// Qt-free notification hooks called from recorder.cpp at the same points it already
// printf()s/transitions state. Implemented in recorder_bridge.cpp, the only file that
// depends on both the plain-C++ recording core and Qt.

struct STATUS;
struct FLIGHT_DATA_RECORD;

void gui_notify_log(struct STATUS* status, const char* text);
void gui_log_printf(struct STATUS* status, const char* fmt, ...);
void gui_notify_connection_changed(struct STATUS* status, bool connected);
void gui_notify_recording_changed(struct STATUS* status, bool recording, int tripId);
void gui_notify_sample(struct STATUS* status, const struct FLIGHT_DATA_RECORD* sample);
// Fired whenever the trips row for the live trip changes mid-recording (departure
// airport resolved, touchdown inserted, destination airport resolved, etc.) so the
// trip history panel can refresh without waiting for the recording to end.
void gui_notify_trip_updated(struct STATUS* status);

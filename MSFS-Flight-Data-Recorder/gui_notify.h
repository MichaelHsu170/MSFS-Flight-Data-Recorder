#pragma once

// Qt-free notification hooks called from recorder.cpp at the same points it
// already transitions state. Implemented in recorder_bridge.cpp, the only
// file that depends on both the plain-C++ recording core and Qt.

struct STATUS;
struct FLIGHT_DATA_RECORD;

// C-compatible log level. Integer values match Logger::Level in logger.h so
// recorder_bridge.cpp can cast directly with no runtime conversion table.
typedef enum GuiLogLevel {
    GUI_LOG_FATAL   = 0,
    GUI_LOG_WARNING = 1,
    GUI_LOG_INFO    = 2,
    GUI_LOG_PROFILE = 3
} GuiLogLevel;

void gui_notify_log(struct STATUS* status, GuiLogLevel level, const char* text);
void gui_log_printf(struct STATUS* status, GuiLogLevel level, const char* fmt, ...);
void gui_notify_connection_changed(struct STATUS* status, bool connected);
void gui_notify_recording_changed(struct STATUS* status, bool recording, int tripId);
void gui_notify_sample(struct STATUS* status, const struct FLIGHT_DATA_RECORD* sample);
// Fired whenever the trips row for the live trip changes mid-recording (departure
// airport resolved, touchdown inserted, destination airport resolved, etc.) so the
// trip history panel can refresh without waiting for the recording to end.
void gui_notify_trip_updated(struct STATUS* status);

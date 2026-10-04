#pragma once

#include <cstddef>

// Qt-free notification hooks called from recorder.cpp, flight_phase.cpp,
// airport_lookup.cpp and db.cpp at the points where they transition state. Implemented in
// recorder_bridge.cpp, the only file that depends on both the plain-C++
// recording core and Qt.

struct STATUS;

// C-compatible log level. Integer values match Logger::Level in logger.h so
// recorder_bridge.cpp can cast directly with no runtime conversion table.
typedef enum GuiLogLevel {
    GUI_LOG_FATAL   = 0,
    GUI_LOG_WARNING = 1,
    GUI_LOG_INFO    = 2,
    GUI_LOG_TRACE   = 3,
    GUI_LOG_PROFILE = 4
} GuiLogLevel;

void gui_notify_log(struct STATUS* status, GuiLogLevel level, const char* text);
void gui_log_printf(struct STATUS* status, GuiLogLevel level, const char* fmt, ...);
void gui_notify_connection_changed(struct STATUS* status, bool connected);
void gui_notify_recording_changed(struct STATUS* status, bool recording, int tripId);
// A sample was just queued for trip_data.
void gui_notify_sample(struct STATUS* status);
// Fired whenever the trips row for the live trip changes mid-recording (departure
// airport resolved, touchdown inserted, destination airport resolved, etc.) so the
// trip history panel can refresh without waiting for the recording to end.
void gui_notify_trip_updated(struct STATUS* status);
// Fired from commit_event() (recorder.cpp) for every occurrence queued for
// writing to trip_events, before the write itself happens. It carries the
// event_seq assigned to that row so the Live Status list can associate its
// new line with the seq; if the write later fails, event_write_worker()
// (db.cpp) retracts the line by that seq via gui_notify_events_retracted()
// below. Distinct from gui_notify_log so the UI-side item and its seq are
// recorded atomically in one slot, instead of relying on a log line and a
// separate seq notification always arriving in the same order. tripId is
// commit_event()'s own trip_id parameter -- the trip this occurrence was
// captured against, which can already be stale (a tier-1/tier-2-delayed
// occurrence flushed after that trip ended) by the time this fires -- so the
// UI side can tell a late-arriving event for an already-ended trip apart from
// one for the trip it's currently showing as live; see LiveStatusPanel::
// onEventCommitted's recordingTripId_ check, the same guard onTripEnded()
// already uses for the analogous late-tripEnded() race.
void gui_notify_event_committed(struct STATUS* status, int tripId, unsigned long long seq, const char* name);
// Fired from event_output() (recorder.cpp) when the flood filter confirms a
// slow flood, to pull its already-committed occurrences back out of the Live
// Status list (a no-op for any that were never shown in the first place --
// e.g. a no-active-trip occurrence) by the same event_seq values just queued
// for deletion from trip_events (db_delete_events()) -- see event_filter.h.
// Also fired from event_write_worker() (db.cpp) when a queued Insert's DB
// write fails, to pull that one occurrence back out since it never actually
// made it into trip_events. That second caller runs on the event-write worker
// thread, as do some gui_log_printf() calls and db_write_worker()'s
// gui_notify_recording_changed(), not the thread SimConnect dispatches on --
// safe because each of these ends in a Qt signal emit, which queues across
// threads.
void gui_notify_events_retracted(struct STATUS* status, const unsigned long long* seqs, size_t count);

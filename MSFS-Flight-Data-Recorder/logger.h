#pragma once

#include <QString>

// Unified application logger. All log output goes to msfs_fdr_debug.log.
// The verbose level configured in settings.ini controls which messages
// are written; messages above the configured level are silently dropped.
//
// Levels (ordered lowest → highest verbosity):
//   Fatal   – unrecoverable errors that halt the app
//   Warning – unexpected/unhandled conditions that don't stop execution
//   Info    – user-visible events (connect, record, takeoff, touchdown, …)
//   Profile – elapsed-time/performance measurements ONLY (ms/µs durations).
//             Non-timing diagnostic detail belongs at Info or Warning instead,
//             so a Profile-filtered log stays a clean timing trace.
//
// Example: verbose=INFO writes Fatal + Warning + Info but not Profile.
// Thread-safe: log() and logf() may be called from any thread after init().
namespace Logger {

enum Level { Fatal = 0, Warning = 1, Info = 2, Profile = 3 };

// Open the log file at filePath and set the maximum verbosity level.
// Must be called once from the main thread before any background work starts.
// The previous run's log (if any) is preserved as filePath + ".old" instead of
// being overwritten, and a header line stamped with appVersion, the process
// ID, and the start time is written first so each run is easy to identify.
void init(Level maxLevel, const QString& filePath, const QString& appVersion = QString());

// Parse a level name case-insensitively ("FATAL", "WARNING", "INFO", "PROFILE").
// Returns Info for unrecognised strings.
Level levelFromString(const QString& s);

// Write one log line if level <= the configured maximum.
// module is printed as a left-justified 8-char column, e.g. "Charts  ".
void log(Level level, const char* module, const QString& msg);

// printf-style convenience wrapper around log().
void logf(Level level, const char* module, const char* fmt, ...);

// Crash/terminate-handler-safe variants of log()/logf(). The regular functions
// block indefinitely on a non-recursive mutex; if the faulting thread is the
// one that already holds it (e.g. the fault occurred inside log() itself),
// that would deadlock the handler instead of recording the crash. These try
// to acquire the mutex for a bounded time and fall back to a best-effort
// unlocked write rather than never writing the diagnostic at all.
void logCrash(Level level, const char* module, const QString& msg);
void logCrashf(Level level, const char* module, const char* fmt, ...);

}

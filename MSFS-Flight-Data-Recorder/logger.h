#pragma once

#include <QString>

// Unified application logger. All log output goes to msfs_fdr_debug.log.
// The verbose level configured in settings.ini controls which messages
// are written; messages above the configured level are silently dropped.
//
// Levels (ordered lowest → highest verbosity):
//   Fatal   – unrecoverable errors that halt the app
//   Warning – unexpected conditions that don't stop execution
//   Info    – user-visible events (connect, record, takeoff, touchdown, …)
//   Profile – performance timing and internal pipeline detail
//
// Example: verbose=INFO writes Fatal + Warning + Info but not Profile.
// Thread-safe: log() and logf() may be called from any thread after init().
namespace Logger {

enum Level { Fatal = 0, Warning = 1, Info = 2, Profile = 3 };

// Open the log file at filePath and set the maximum verbosity level.
// Must be called once from the main thread before any background work starts.
void init(Level maxLevel, const QString& filePath);

// Parse a level name case-insensitively ("FATAL", "WARNING", "INFO", "PROFILE").
// Returns Info for unrecognised strings.
Level levelFromString(const QString& s);

// Write one log line if level <= the configured maximum.
// module is printed as a left-justified 8-char column, e.g. "Charts  ".
void log(Level level, const char* module, const QString& msg);

// printf-style convenience wrapper around log().
void logf(Level level, const char* module, const char* fmt, ...);

}

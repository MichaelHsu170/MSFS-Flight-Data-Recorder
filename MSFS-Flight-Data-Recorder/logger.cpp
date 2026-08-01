#include "logger.h"
#include "logger_c.h"

#include <QDateTime>
#include <QFile>
#include <QMutex>
#include <QMutexLocker>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <Windows.h>

namespace {

// Opened with FILE_APPEND_DATA (not plain GENERIC_WRITE) so every WriteFile
// call is an OS-level atomic append: Windows positions each write at the
// current end-of-file itself, as part of the write, rather than relying on a
// handle-cached position that a seek established once. That guarantee is
// what makes logCrash()'s lock-free fallback below safe -- two handles (or
// two threads sharing this one) issuing WriteFile concurrently can never
// land at overlapping offsets and corrupt/truncate each other's bytes, which
// a QFile opened in QIODevice::Append (a one-time seek-to-EOF at open(), not
// a per-write OS append) cannot promise.
// atomic, not plain HANDLE: init() writes this once under g_mutex, but log(),
// logCrash() and writeLine() all read it without taking the lock (logCrash()
// deliberately, to stay lock-free -- see its own comment below; log()/writeLine()
// as a fast pre-lock bail-out before logging is enabled). Without atomicity that
// unsynchronized read/write pair across threads is a data race.
std::atomic<HANDLE> g_fileHandle{INVALID_HANDLE_VALUE};
QMutex              g_mutex;
std::atomic<int>    g_maxLevel{static_cast<int>(Logger::Info)};

const char* levelTag(Logger::Level level) {
    switch (level) {
    case Logger::Fatal:   return "FATAL";
    case Logger::Warning: return "WARN ";
    case Logger::Info:    return "INFO ";
    case Logger::Profile: return "PROF ";
    }
    return "?    ";
}

void writeLine(const QString& line) {
    HANDLE h = g_fileHandle.load(std::memory_order_acquire);
    if (h == INVALID_HANDLE_VALUE)
        return;
    QByteArray utf8 = line.toUtf8();
    DWORD written = 0;
    if (!WriteFile(h, utf8.constData(), static_cast<DWORD>(utf8.size()), &written, nullptr)) {
        // The log file itself is the usual place failures get reported, so a
        // WriteFile failure here has nowhere else to go -- surface it to a
        // debugger (if attached) rather than dropping it silently.
        OutputDebugStringA("Logger::writeLine: WriteFile failed\n");
    }
}

}

namespace Logger {

void init(Level maxLevel, const QString& filePath, const QString& appVersion) {
    g_maxLevel.store(static_cast<int>(maxLevel));
    QMutexLocker lock(&g_mutex);
    if (g_fileHandle.load(std::memory_order_relaxed) != INVALID_HANDLE_VALUE)
        return;
    // Preserve the previous run's log instead of truncating it, so a crash
    // followed by a manual relaunch doesn't erase the only record of it.
    const QString oldPath = filePath + QStringLiteral(".old");
    QFile::remove(oldPath);
    QFile::rename(filePath, oldPath);
    // OPEN_ALWAYS, not CREATE_ALWAYS: CREATE_ALWAYS truncates an existing file,
    // which per CreateFile's documented access-right requirements needs
    // GENERIC_WRITE/FILE_WRITE_DATA -- access this handle deliberately doesn't
    // request (see the FILE_APPEND_DATA-only rationale above). If the rename
    // above fails (e.g. .old locked by an AV scanner or a second app instance)
    // filePath still exists, and CREATE_ALWAYS against it would fail with
    // ERROR_ACCESS_DENIED, leaving logging silently disabled for the whole
    // session. OPEN_ALWAYS has no such requirement: it creates the file if
    // absent (the normal case, after a successful rename) or just opens it
    // without truncating if present, so a failed rotate degrades to appending
    // after the old content instead of losing all logging.
    HANDLE h = CreateFileW(
        reinterpret_cast<const wchar_t*>(filePath.utf16()),
        FILE_APPEND_DATA,
        FILE_SHARE_READ,
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (h == INVALID_HANDLE_VALUE)
        return;
    g_fileHandle.store(h, std::memory_order_release);
    QString header = QStringLiteral("==== MSFS Flight Data Recorder")
        + (appVersion.isEmpty() ? QString() : QStringLiteral(" v%1").arg(appVersion))
        + QStringLiteral(" started (PID %1) at %2 ====\n")
              .arg(GetCurrentProcessId())
              .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")));
    writeLine(header);
}

Level levelFromString(const QString& s) {
    const QString u = s.toUpper().trimmed();
    if (u == QLatin1String("FATAL"))   return Fatal;
    if (u == QLatin1String("WARNING")) return Warning;
    if (u == QLatin1String("INFO"))    return Info;
    if (u == QLatin1String("PROFILE")) return Profile;
    return Info;
}

void log(Level level, const char* module, const QString& msg) {
    if (static_cast<int>(level) > g_maxLevel.load(std::memory_order_relaxed))
        return;
    if (g_fileHandle.load(std::memory_order_acquire) == INVALID_HANDLE_VALUE)
        return;
    QString line = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        + QStringLiteral(" [") + QLatin1String(levelTag(level)) + QStringLiteral("] [")
        + QString::fromUtf8(module).leftJustified(8) + QStringLiteral("] ") + msg + QStringLiteral("\n");
    QMutexLocker lock(&g_mutex);
    writeLine(line);
}

void logf(Level level, const char* module, const char* fmt, ...) {
    if (static_cast<int>(level) > g_maxLevel.load(std::memory_order_relaxed))
        return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log(level, module, QString::fromUtf8(buf));
}

void logCrash(Level level, const char* module, const QString& msg) {
    if (static_cast<int>(level) > g_maxLevel.load(std::memory_order_relaxed))
        return;
    if (g_fileHandle.load(std::memory_order_acquire) == INVALID_HANDLE_VALUE)
        return;
    QString line = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        + QStringLiteral(" [") + QLatin1String(levelTag(level)) + QStringLiteral("] [")
        + QString::fromUtf8(module).leftJustified(8) + QStringLiteral("] ") + msg + QStringLiteral("\n");
    // Deliberately skips g_mutex: g_fileHandle was opened with FILE_APPEND_DATA,
    // so each WriteFile call atomically appends at EOF at the OS level -- safe
    // to call concurrently with log()'s own writeLine() even if the thread that
    // crashed died while holding g_mutex, which would otherwise deadlock the
    // handler (waiting on a lock its own thread can never release) instead of
    // recording the crash.
    writeLine(line);
}

void logCrashf(Level level, const char* module, const char* fmt, ...) {
    if (static_cast<int>(level) > g_maxLevel.load(std::memory_order_relaxed))
        return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    logCrash(level, module, QString::fromUtf8(buf));
}

}

extern "C" {

void log_c(int level, const char* module, const char* msg) {
    Logger::log(static_cast<Logger::Level>(level), module, QString::fromUtf8(msg));
}

void log_cf(int level, const char* module, const char* fmt, ...) {
    if (level > static_cast<int>(Logger::Profile))
        return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_c(level, module, buf);
}

}

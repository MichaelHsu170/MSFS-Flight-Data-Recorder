#include "logger.h"
#include "logger_c.h"

#include <QDateTime>
#include <QFile>
#include <QMutex>
#include <QMutexLocker>
#include <QTextStream>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <Windows.h>

namespace {

QFile*             g_file;
QMutex             g_mutex;
std::atomic<int>   g_maxLevel{static_cast<int>(Logger::Info)};

const char* levelTag(Logger::Level level) {
    switch (level) {
    case Logger::Fatal:   return "FATAL";
    case Logger::Warning: return "WARN ";
    case Logger::Info:    return "INFO ";
    case Logger::Profile: return "PROF ";
    }
    return "?    ";
}

}

namespace Logger {

void init(Level maxLevel, const QString& filePath, const QString& appVersion) {
    g_maxLevel.store(static_cast<int>(maxLevel));
    QMutexLocker lock(&g_mutex);
    if (g_file)
        return;
    // Preserve the previous run's log instead of truncating it, so a crash
    // followed by a manual relaunch doesn't erase the only record of it.
    const QString oldPath = filePath + QStringLiteral(".old");
    QFile::remove(oldPath);
    QFile::rename(filePath, oldPath);
    g_file = new QFile(filePath);
    if (!g_file->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        delete g_file;
        g_file = nullptr;
        return;
    }
    QTextStream out(g_file);
    out << QStringLiteral("==== MSFS Flight Data Recorder")
        << (appVersion.isEmpty() ? QString() : QStringLiteral(" v%1").arg(appVersion))
        << QStringLiteral(" started (PID %1) at %2 ====\n")
              .arg(GetCurrentProcessId())
              .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")));
    out.flush();
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
    QMutexLocker lock(&g_mutex);
    if (!g_file)
        return;
    QTextStream out(g_file);
    out << QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        << " [" << levelTag(level) << "] ["
        << QString::fromUtf8(module).leftJustified(8)
        << "] " << msg << '\n';
    out.flush();
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

#include <QApplication>
#include <QIcon>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QQuickStyle>

#include "logger.h"
#include "db.h"
#include "recorder_bridge.h"
#include "main_window.h"
#include "types.h"

#include <Windows.h>
#include <exception>
#include <malloc.h>

// Last-resort diagnostics: neither of these can recover the process, but they
// give the log a chance to say why it died instead of leaving the last line
// before a crash as the only clue.
static LONG WINAPI crashHandler(EXCEPTION_POINTERS* info) {
    if (info->ExceptionRecord->ExceptionCode == EXCEPTION_STACK_OVERFLOW) {
        // Only a guard-page's worth of stack remains at this point -- Logger::logf's
        // QString formatting and heap allocation would very likely re-fault before
        // writing anything. _resetstkoflw() restores the guard page first so the
        // rest of this handler has normal stack space to run in.
        _resetstkoflw();
    }
    Logger::logCrashf(Logger::Fatal, "Crash",
        "Unhandled exception 0x%08lX at address %p",
        info->ExceptionRecord->ExceptionCode,
        info->ExceptionRecord->ExceptionAddress);
    return EXCEPTION_CONTINUE_SEARCH;
}

static void terminateHandler() {
    QString detail;
    if (std::exception_ptr eptr = std::current_exception()) {
        try {
            std::rethrow_exception(eptr);
        } catch (const std::exception& e) {
            detail = QString::fromUtf8(e.what());
        } catch (...) {
            detail = QStringLiteral("non-standard exception");
        }
    } else {
        detail = QStringLiteral("no active exception");
    }
    Logger::logCrash(Logger::Fatal, "Crash", QStringLiteral("std::terminate called: %1").arg(detail));
    std::abort();
}

static void logMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
    // Suppress high-volume Qt-internal noise that drowns out app messages.
    if (msg.startsWith(QLatin1String("QML debugging"))
        || msg.startsWith(QLatin1String("QFont::"))
        || msg.contains(QLatin1String("qt.qpa."))
        || msg.contains(QLatin1String("QStandardPaths:"))
        || msg.startsWith(QLatin1String("libpng warning"))
        || msg.contains(QLatin1String("is not installed")))
        return;

    Logger::Level level = Logger::Trace;
    switch (type) {
    case QtInfoMsg:     level = Logger::Info;    break;
    case QtWarningMsg:  level = Logger::Warning; break;
    case QtCriticalMsg: level = Logger::Warning; break;
    case QtFatalMsg:    level = Logger::Fatal;   break;
    default:            level = Logger::Trace;   break;
    }

    QString text = msg;
    if (type >= QtWarningMsg && ctx.file && ctx.line > 0)
        text = QStringLiteral("%1:%2 | %3").arg(QLatin1String(ctx.file)).arg(ctx.line).arg(msg);

    Logger::log(level, "Qt", text);
}

int main(int argc, char* argv[]) {
    // Determine base directory for log and settings files.
    // Debug: use cwd so each project checkout is self-contained.
    // Release: use the exe's directory so they follow the installation.
#ifdef _DEBUG
    const QString baseDir = QDir::current().absolutePath();
#else
    const QString baseDir = (argc > 0)
        ? QFileInfo(QString::fromLocal8Bit(argv[0])).absoluteDir().absolutePath()
        : QDir::current().absolutePath();
#endif

    // Read verbose level before QApplication: QSettings with an explicit file
    // path works without QCoreApplication, and so does QDir::current().
    {
        QSettings s(QDir(baseDir).filePath(QStringLiteral("settings.ini")), QSettings::IniFormat);
        Logger::Level lvl = Logger::levelFromString(
            s.value(QStringLiteral("logging/verbose"), QStringLiteral("INFO")).toString());
        Logger::init(lvl, baseDir + QStringLiteral("/msfs_fdr_debug.log"), QStringLiteral(APP_VERSION));
    }
    qInstallMessageHandler(logMessageHandler);
    SetUnhandledExceptionFilter(crashHandler);
    std::set_terminate(terminateHandler);

    Logger::logf(Logger::Trace, "Qt", "Base directory resolved to %s", qUtf8Printable(baseDir));

    // Must be called before QApplication: QQC2 auto-detects the style from
    // the QWidget app's QStyle, which resolves to "Fusion" in a Widgets context
    // and triggers a warning when the Fusion QML module isn't deployed.
    QQuickStyle::setStyle(QStringLiteral("Windows"));

    // Required by QWebEngineView (trajectory map) before QApplication exists.
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/app_icon.ico"));
    Logger::log(Logger::Trace, "Qt", QStringLiteral("QApplication constructed"));

    migrate_db();
    Logger::log(Logger::Trace, "Qt", QStringLiteral("Database migration checked/applied"));

    RecorderBridge bridge;
    MainWindow window(bridge);
    // 900x600 was too small to give the charts panel (QQuickWidget, stacked
    // below the map/table row) enough room to be visible.
    window.resize(1320, 900);
    window.setMinimumSize(1000, 700);
    window.show();
    Logger::log(Logger::Trace, "Qt", QStringLiteral("Main window shown"));

    return app.exec();
}

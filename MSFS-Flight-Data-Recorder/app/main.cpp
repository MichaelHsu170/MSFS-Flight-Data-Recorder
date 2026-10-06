#include <QApplication>
#include <QIcon>
#include <QFileInfo>
#include <QQuickStyle>

#include "app_paths.h"
#include "app_settings.h"
#include "logger.h"
#include "recorder_bridge.h"
#include "main_window.h"
#include "version.h"

#include <Windows.h>
#include <exception>

// Last-resort diagnostics: neither of these can recover the process, but they
// give the log a chance to say why it died instead of leaving the last line
// before a crash as the only clue.
static LONG WINAPI crashHandler(EXCEPTION_POINTERS* info) {
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

// Ensures at most one instance of this app runs per Windows login session.
// A named kernel mutex is used rather than a lock file so a crash or kill
// leaves nothing stale behind -- the OS releases the mutex automatically
// when the process exits, however it exits, with no cleanup/staleness check
// needed on the next launch.
static bool acquireSingleInstanceLock() {
	// "Local\" scopes the name to this login session, matching one instance
	// per logged-in user -- "Global\" would also block a second instance
	// running under a different user or RDP session, which isn't what a
	// per-user desktop app wants.
	HANDLE mutex = CreateMutexW(nullptr, FALSE, L"Local\\MSFS-Flight-Data-Recorder-SingleInstance");
	if (mutex == nullptr)
		return true; // Unexpected failure to even create the mutex -- fail open rather than block launch entirely.
	if (GetLastError() == ERROR_ALREADY_EXISTS) {
		CloseHandle(mutex);
		return false;
	}
	// Deliberately never closed: held for the life of this process so the OS
	// frees it (unblocking a future launch) on exit, normal or crash.
	return true;
}

static void logMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
	// Suppress high-volume Qt-internal noise that drowns out app messages.
	if (msg.startsWith(QLatin1String("QML debugging"))
		|| msg.startsWith(QLatin1String("QFont::"))
		|| msg.contains(QLatin1String("qt.qpa."))
		|| msg.contains(QLatin1String("QStandardPaths:"))
		|| msg.startsWith(QLatin1String("libpng warning"))
		|| msg.contains(QLatin1String("is not installed"))
		// Qt Graphs warns each time a series re-adds an axis to the graph it
		// already belongs to, so engine load series 2-4, sharing series 1's
		// right-hand axis (charts_panel.qml), log it once each; harmless.
		// tst_charts_panel matches the same text and fails on any more of
		// them, which would be a real axis wiring mistake that this filter
		// would otherwise hide.
		|| msg.contains(QLatin1String("axis already associated with")))
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
	// Checked before anything else (log init included): logging opens its
	// file with FILE_SHARE_READ only and rotates the previous run's log on
	// every launch (see Logger::init), so letting a second instance reach
	// that code would fight the first instance over the same log file
	// instead of being turned away cleanly here.
	if (!acquireSingleInstanceLock()) {
		MessageBoxW(nullptr,
			L"MSFS Flight Data Recorder is already running.",
			L"MSFS Flight Data Recorder",
			MB_OK | MB_ICONINFORMATION);
		return 0;
	}

	// Before QApplication, so everything from here on is logged: neither
	// app_file_path() nor AppSettings::logLevel() needs a QCoreApplication.
	const QString logPath = QString::fromStdString(app_file_path("msfs_fdr_debug.log"));
	Logger::init(Logger::levelFromString(AppSettings::logLevel()), logPath, QStringLiteral(APP_VERSION));
	qInstallMessageHandler(logMessageHandler);
	// A stack overflow leaves only a guard page's worth of stack, which the
	// crash handler's formatting would overflow again before logging
	// anything; this keeps 64 KB in reserve for it. It covers the main
	// thread (where the UI runs) only: the guarantee is per thread.
	ULONG stackGuarantee = 64 * 1024;
	SetThreadStackGuarantee(&stackGuarantee);
	SetUnhandledExceptionFilter(crashHandler);
	std::set_terminate(terminateHandler);

	Logger::logf(Logger::Trace, "Qt", "Base directory resolved to %s", qUtf8Printable(QFileInfo(logPath).absolutePath()));

	// Must be called before QApplication: QQC2 auto-detects the style from
	// the QWidget app's QStyle, which resolves to "Fusion" in a Widgets context
	// and triggers a warning when the Fusion QML module isn't deployed.
	QQuickStyle::setStyle(QStringLiteral("Windows"));

	// Required by QWebEngineView (trajectory map) before QApplication exists.
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	app.setWindowIcon(QIcon(":/app_icon.ico"));
	Logger::log(Logger::Trace, "Qt", QStringLiteral("QApplication constructed"));

	RecorderBridge bridge;
	MainWindow window(bridge);
	// Large enough for the charts panel (QQuickWidget, stacked below the
	// map/table row) to get visible room.
	window.resize(1320, 900);
	window.setMinimumSize(1000, 700);
	window.show();
	Logger::log(Logger::Trace, "Qt", QStringLiteral("Main window shown"));

	return app.exec();
}

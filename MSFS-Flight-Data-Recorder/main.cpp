#include <QApplication>
#include <QIcon>
#include <QDebug>
#include <QFile>
#include <QMutex>
#include <QTextStream>
#include <QDateTime>
#include <QStandardPaths>
#include <QDir>
#include <QQuickStyle>

#include "recorder_bridge.h"
#include "main_window.h"

static QFile* g_logFile = nullptr;
static QMutex g_logMutex;

static void logMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
    // Suppress high-volume Qt-internal noise that drowns out app messages.
    if (msg.startsWith(QLatin1String("QML debugging"))
        || msg.startsWith(QLatin1String("QFont::"))
        || msg.contains(QLatin1String("qt.qpa."))
        || msg.contains(QLatin1String("QStandardPaths:"))
        || msg.startsWith(QLatin1String("libpng warning"))
        // QtQuick.Controls qmldir probes all known style modules; deleted styles
        // produce "not installed" warnings that are expected and harmless.
        || msg.contains(QLatin1String("is not installed")))
        return;

    const char* level = "DBG";
    switch (type) {
    case QtInfoMsg:     level = "INF"; break;
    case QtWarningMsg:  level = "WRN"; break;
    case QtCriticalMsg: level = "CRT"; break;
    case QtFatalMsg:    level = "FAT"; break;
    default: break;
    }

    QMutexLocker locker(&g_logMutex);
    if (!g_logFile || !g_logFile->isOpen())
        return;

    QTextStream out(g_logFile);
    out << QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        << " [" << level << "] ";
    // Include source location for warnings and above (ctx.file is null in release builds).
    if (type >= QtWarningMsg && ctx.file && ctx.line > 0)
        out << ctx.file << ":" << ctx.line << " | ";
    out << msg << "\n";
    out.flush();
}

int main(int argc, char* argv[]) {
    // Debug: log next to where the command is run (cwd set by VS Code launch / cmake).
    // Release: log next to the exe so it's always findable regardless of cwd.
#ifdef _DEBUG
    QString logDir = QDir::current().absolutePath();
#else
    QString logDir = (argc > 0)
        ? QFileInfo(QString::fromLocal8Bit(argv[0])).absoluteDir().absolutePath()
        : QDir::current().absolutePath();
#endif
    g_logFile = new QFile(logDir + QStringLiteral("/msfs_fdr_debug.log"));
    if (g_logFile->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        qInstallMessageHandler(logMessageHandler);

	// Must be called before QApplication: QQC2 auto-detects the style from
	// the QWidget app's QStyle, which resolves to "Fusion" in a Widgets context
	// and triggers a warning when the Fusion QML module isn't deployed.
	QQuickStyle::setStyle(QStringLiteral("Windows"));

	// Required by QWebEngineView (trajectory map) before QApplication exists.
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	app.setWindowIcon(QIcon(":/app_icon.ico"));

	RecorderBridge bridge;
	MainWindow window(bridge);
	// 900x600 was too small to give the charts panel (QQuickWidget, stacked
	// below the map/table row) enough room to be visible.
	window.resize(1320, 900);
	window.setMinimumSize(1000, 700);
	window.show();

	return app.exec();
}

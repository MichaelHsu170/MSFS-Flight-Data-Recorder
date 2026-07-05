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

static void logMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
    // Suppress high-volume Qt-internal noise that drowns out app messages.
    if (msg.startsWith(QLatin1String("QML debugging"))
        || msg.startsWith(QLatin1String("QFont::"))
        || msg.contains(QLatin1String("qt.qpa."))
        || msg.contains(QLatin1String("QStandardPaths:"))
        || msg.startsWith(QLatin1String("libpng warning"))
        || msg.contains(QLatin1String("is not installed")))
        return;

    Logger::Level level = Logger::Profile;
    switch (type) {
    case QtInfoMsg:     level = Logger::Info;    break;
    case QtWarningMsg:  level = Logger::Warning; break;
    case QtCriticalMsg: level = Logger::Warning; break;
    case QtFatalMsg:    level = Logger::Fatal;   break;
    default:            level = Logger::Profile; break;
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
        Logger::init(lvl, baseDir + QStringLiteral("/msfs_fdr_debug.log"));
    }
    qInstallMessageHandler(logMessageHandler);

    // Must be called before QApplication: QQC2 auto-detects the style from
    // the QWidget app's QStyle, which resolves to "Fusion" in a Widgets context
    // and triggers a warning when the Fusion QML module isn't deployed.
    QQuickStyle::setStyle(QStringLiteral("Windows"));

    // Required by QWebEngineView (trajectory map) before QApplication exists.
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/app_icon.ico"));

    migrate_db();

    RecorderBridge bridge;
    MainWindow window(bridge);
    // 900x600 was too small to give the charts panel (QQuickWidget, stacked
    // below the map/table row) enough room to be visible.
    window.resize(1320, 900);
    window.setMinimumSize(1000, 700);
    window.show();

    return app.exec();
}

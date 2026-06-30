#include <QApplication>
#include <QIcon>

#include "recorder_bridge.h"
#include "main_window.h"

int main(int argc, char* argv[]) {
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

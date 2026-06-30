#include "logger.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>

namespace Logger {

void log(const QString& text) {
	QString path = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("flight_data_recorder.log"));
	QFile file(path);
	if (!file.open(QIODevice::Append | QIODevice::Text))
		return;
	QTextStream stream(&file);
	stream << QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")) << " " << text << Qt::endl;
}

}

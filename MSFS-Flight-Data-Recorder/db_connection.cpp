#include "db_connection.h"

#include "logger.h"

#include <QMessageBox>

DbConnection openForReading(const QString& what) {
	DbConnection sql = DbConnection::readOnly();
	if (!sql)
		Logger::logf(Logger::Warning, "DB", "%s: failed to open read-only connection", qUtf8Printable(what));
	return sql;
}

DbConnection openForWriting(QWidget* parent, const char* action) {
	DbConnection sql = DbConnection::readWrite();
	if (!sql) {
		Logger::logf(Logger::Warning, "DB", "Cannot %s: failed to open the database for writing", action);
		QMessageBox::critical(parent, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
	}
	return sql;
}

#include "db_connection.h"

#include "logger.h"

#include <QMessageBox>

DbConnection openForWriting(QWidget* parent, const char* action) {
	DbConnection sql = DbConnection::readWrite();
	if (!sql) {
		Logger::logf(Logger::Warning, "DB", "Cannot %s: failed to open the database for writing", action);
		QMessageBox::critical(parent, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
	}
	return sql;
}

// Statement plumbing (db_query.cpp) on its own: success and genuine SQLite
// failure paths (malformed SQL, constraint violations, a locked database,
// nested transactions, a failing commit) for every helper, driven against a
// real sqlite3 connection rather than a mock. Each failure must also log a
// warning that names the caller's context.
#include "test_support.h"

#include "db_query.h"
#include "logger.h"

#include "sqlite3.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QThread>
#include <QtTest>

#include <vector>

using TestSupport::exec;

namespace {

sqlite3* openMemoryDb() {
	sqlite3* db = nullptr;
	sqlite3_open(":memory:", &db);
	return db;
}

int countRows(sqlite3* db) {
	sqlite3_stmt* check = nullptr;
	sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM t", -1, &check, nullptr);
	sqlite3_step(check);
	const int count = sqlite3_column_int(check, 0);
	sqlite3_finalize(check);
	return count;
}

}

class TstDbQuery : public QObject {
	Q_OBJECT

	QTemporaryDir logDir_;
	QString logPath_;

	bool warningLogged(const QStringList& parts) { return TestSupport::warningLogged(logPath_, parts); }

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		logPath_ = logDir_.filePath(QStringLiteral("db_query.log"));
		Logger::init(Logger::Warning, logPath_);
	}

	void columnTextReadsTextOrNull() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT, b TEXT)");
		exec(db, "INSERT INTO t VALUES ('hello', NULL)");
		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "SELECT a, b FROM t", -1, &stmt, nullptr);
		QCOMPARE(sqlite3_step(stmt), SQLITE_ROW);
		QCOMPARE(columnText(stmt, 0), QStringLiteral("hello"));
		QVERIFY(columnText(stmt, 1).isEmpty());
		sqlite3_finalize(stmt);
		sqlite3_close(db);
	}

	void prepareStatementGivesARunnableStatement() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT)");
		exec(db, "INSERT INTO t VALUES ('row')");
		sqlite3_stmt* stmt = prepareStatement(db, "SELECT a FROM t", QStringLiteral("ctx"));
		QVERIFY(stmt != nullptr);
		QCOMPARE(sqlite3_step(stmt), SQLITE_ROW);
		QCOMPARE(columnText(stmt, 0), QStringLiteral("row"));
		sqlite3_finalize(stmt);
		sqlite3_close(db);
	}

	void prepareStatementFailsOnMalformedSqlAndLogs() {
		sqlite3* db = openMemoryDb();
		sqlite3_stmt* stmt = prepareStatement(db, "NOT VALID SQL", QStringLiteral("prepareCtx"));
		QVERIFY(stmt == nullptr);
		QVERIFY(warningLogged({ QStringLiteral("prepareCtx"), QStringLiteral("prepare failed"), QStringLiteral("NOT VALID SQL") }));
		sqlite3_close(db);
	}

	void forEachRowVisitsEveryRowThenFinalizes() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		exec(db, "INSERT INTO t VALUES (1), (2), (3)");
		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t ORDER BY a", -1, &stmt, nullptr);
		std::vector<int> seen;
		forEachRow(db, stmt, QStringLiteral("ctx"), [&seen](sqlite3_stmt* s) {
			seen.push_back(sqlite3_column_int(s, 0));
			return true;
		});
		QCOMPARE(seen, (std::vector<int>{ 1, 2, 3 }));
		QCOMPARE(sqlite3_close(db), SQLITE_OK); // SQLITE_BUSY if stmt were left unfinalized
	}

	void forEachRowStopsEarlyWhenOnRowReturnsFalse() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		exec(db, "INSERT INTO t VALUES (1), (2), (3)");
		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t ORDER BY a", -1, &stmt, nullptr);
		std::vector<int> seen;
		forEachRow(db, stmt, QStringLiteral("stopCtx"), [&seen](sqlite3_stmt* s) {
			seen.push_back(sqlite3_column_int(s, 0));
			return seen.size() < 2;
		});
		QCOMPARE(seen, (std::vector<int>{ 1, 2 }));
		QVERIFY(!warningLogged({ QStringLiteral("stopCtx") })); // stopping early isn't a failure
		QCOMPARE(sqlite3_close(db), SQLITE_OK);
	}

	void forEachRowLogsAndFinalizesOnAGenuineStepFailure() {
		// A second connection holding an EXCLUSIVE lock on the same file-backed
		// db makes the first connection's step fail with SQLITE_BUSY instead
		// of SQLITE_ROW/DONE -- a real failure, not a simulated one.
		const QString path = QDir::temp().filePath(QStringLiteral("fdr_test_db_query_%1.sqlite").arg(reinterpret_cast<quintptr>(QThread::currentThread())));
		QFile::remove(path);
		sqlite3* db = nullptr;
		sqlite3_open(qUtf8Printable(path), &db);
		exec(db, "CREATE TABLE t (a INTEGER)");
		exec(db, "INSERT INTO t VALUES (1), (2)");

		sqlite3* locker = nullptr;
		sqlite3_open(qUtf8Printable(path), &locker);
		exec(locker, "BEGIN EXCLUSIVE");

		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t", -1, &stmt, nullptr);
		bool onRowCalled = false;
		forEachRow(db, stmt, QStringLiteral("stepCtx"), [&onRowCalled](sqlite3_stmt*) {
			onRowCalled = true;
			return true;
		});
		QVERIFY(!onRowCalled);  // the very first step failed with SQLITE_BUSY
		QVERIFY(warningLogged({ QStringLiteral("stepCtx"), QStringLiteral("step failed") }));

		exec(locker, "ROLLBACK");
		sqlite3_close(locker);
		QCOMPARE(sqlite3_close(db), SQLITE_OK);
		QFile::remove(path);
	}

	void bindTextBindsUtf8() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT)");
		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "INSERT INTO t VALUES (?)", -1, &stmt, nullptr);
		bindText(stmt, 1, QStringLiteral("café"));
		QCOMPARE(sqlite3_step(stmt), SQLITE_DONE);
		sqlite3_finalize(stmt);
		sqlite3_stmt* check = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t", -1, &check, nullptr);
		sqlite3_step(check);
		QCOMPARE(columnText(check, 0), QStringLiteral("café"));
		sqlite3_finalize(check);
		sqlite3_close(db);
	}

	void execStatementRunsTheBoundStatement() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT)");
		const bool ok = execStatement(db, "INSERT INTO t VALUES (?)", QStringLiteral("ctx"),
			[](sqlite3_stmt* stmt) { bindText(stmt, 1, QStringLiteral("x")); });
		QVERIFY(ok);
		sqlite3_stmt* check = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t", -1, &check, nullptr);
		QCOMPARE(sqlite3_step(check), SQLITE_ROW);
		QCOMPARE(columnText(check, 0), QStringLiteral("x"));
		sqlite3_finalize(check);
		QCOMPARE(sqlite3_close(db), SQLITE_OK);
	}

	void execStatementFailsOnMalformedSqlAndLogs() {
		sqlite3* db = openMemoryDb();
		const bool ok = execStatement(db, "NOT VALID SQL", QStringLiteral("execPrepareCtx"), [](sqlite3_stmt*) {});
		QVERIFY(!ok);
		QVERIFY(warningLogged({ QStringLiteral("execPrepareCtx"), QStringLiteral("NOT VALID SQL") }));
		sqlite3_close(db);
	}

	void execStatementFailsOnConstraintViolationAndLogs() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT UNIQUE)");
		exec(db, "INSERT INTO t VALUES ('dup')");
		const bool ok = execStatement(db, "INSERT INTO t VALUES (?)", QStringLiteral("execStepCtx"),
			[](sqlite3_stmt* stmt) { bindText(stmt, 1, QStringLiteral("dup")); });
		QVERIFY(!ok);
		QVERIFY(warningLogged({ QStringLiteral("execStepCtx"), QStringLiteral("INSERT INTO t VALUES (?)") }));
		QCOMPARE(sqlite3_close(db), SQLITE_OK);
	}

	void inTransactionCommitsOnSuccess() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		const bool ok = inTransaction(db, QStringLiteral("ctx"), [&db]() {
			return execStatement(db, "INSERT INTO t VALUES (1)", QStringLiteral("ctx"), [](sqlite3_stmt*) {});
		});
		QVERIFY(ok);
		QCOMPARE(countRows(db), 1);
		QVERIFY(sqlite3_get_autocommit(db) != 0); // the transaction is closed
		sqlite3_close(db);
	}

	void inTransactionRollsBackWhenBodyFails() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		const bool ok = inTransaction(db, QStringLiteral("ctx"), [&db]() {
			exec(db, "INSERT INTO t VALUES (1)");
			return false;
		});
		QVERIFY(!ok);
		QCOMPARE(countRows(db), 0);  // rolled back
		QVERIFY(sqlite3_get_autocommit(db) != 0);
		sqlite3_close(db);
	}

	void inTransactionRollsBackAndLogsWhenTheCommitFails() {
		// A deferred foreign key is only checked at COMMIT, so the commit
		// itself fails genuinely.
		sqlite3* db = openMemoryDb();
		exec(db, "PRAGMA foreign_keys = ON");
		exec(db, "CREATE TABLE p (id INTEGER PRIMARY KEY)");
		exec(db, "CREATE TABLE t (a INTEGER REFERENCES p(id) DEFERRABLE INITIALLY DEFERRED)");
		const bool ok = inTransaction(db, QStringLiteral("commitCtx"), [&db]() {
			exec(db, "INSERT INTO t VALUES (5)");
			return true;
		});
		QVERIFY(!ok);
		QVERIFY(warningLogged({ QStringLiteral("commitCtx"), QStringLiteral("COMMIT") }));
		QVERIFY(sqlite3_get_autocommit(db) != 0); // not left open
		QCOMPARE(countRows(db), 0);
		sqlite3_close(db);
	}

	void inTransactionFailsAndLogsWhenAlreadyInOne() {
		sqlite3* db = openMemoryDb();
		exec(db, "BEGIN TRANSACTION");
		// The real sqlite3_exec("BEGIN TRANSACTION", ...) call inside
		// inTransaction() fails genuinely: a transaction is already active.
		bool bodyRan = false;
		const bool ok = inTransaction(db, QStringLiteral("beginCtx"), [&bodyRan]() { bodyRan = true; return true; });
		QVERIFY(!ok);
		QVERIFY(!bodyRan);
		QVERIFY(warningLogged({ QStringLiteral("beginCtx"), QStringLiteral("BEGIN") }));
		exec(db, "ROLLBACK");
		sqlite3_close(db);
	}
};

QTEST_APPLESS_MAIN(TstDbQuery)
#include "tst_db_query.moc"

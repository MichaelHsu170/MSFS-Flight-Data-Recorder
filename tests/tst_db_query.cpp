// Statement plumbing (db_query.cpp) on its own: success and genuine SQLite
// failure paths (malformed SQL, constraint violations, a locked database,
// nested transactions) for every helper, driven against a real sqlite3
// connection rather than a mock.
#include "db_query.h"

#include "sqlite3.h"

#include <QDir>
#include <QFile>
#include <QThread>
#include <QtTest>

#include <vector>

namespace {

sqlite3* openMemoryDb() {
	sqlite3* db = nullptr;
	sqlite3_open(":memory:", &db);
	return db;
}

void exec(sqlite3* db, const char* sql) {
	sqlite3_exec(db, sql, nullptr, nullptr, nullptr);
}

}

class TstDbQuery : public QObject {
	Q_OBJECT

private slots:
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

	void prepareStatementSucceeds() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT)");
		sqlite3_stmt* stmt = prepareStatement(db, "SELECT a FROM t", QStringLiteral("ctx"));
		QVERIFY(stmt != nullptr);
		sqlite3_finalize(stmt);
		sqlite3_close(db);
	}

	void prepareStatementFailsOnMalformedSql() {
		sqlite3* db = openMemoryDb();
		sqlite3_stmt* stmt = prepareStatement(db, "NOT VALID SQL", QStringLiteral("ctx"));
		QVERIFY(stmt == nullptr);
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
		sqlite3_close(db);
	}

	void forEachRowStopsEarlyWhenOnRowReturnsFalse() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		exec(db, "INSERT INTO t VALUES (1), (2), (3)");
		sqlite3_stmt* stmt = nullptr;
		sqlite3_prepare_v2(db, "SELECT a FROM t ORDER BY a", -1, &stmt, nullptr);
		std::vector<int> seen;
		forEachRow(db, stmt, QStringLiteral("ctx"), [&seen](sqlite3_stmt* s) {
			seen.push_back(sqlite3_column_int(s, 0));
			return seen.size() < 2;
		});
		QCOMPARE(seen, (std::vector<int>{ 1, 2 }));
		sqlite3_close(db);
	}

	void forEachRowLogsOnAGenuineStepFailure() {
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
		forEachRow(db, stmt, QStringLiteral("ctx"), [&onRowCalled](sqlite3_stmt*) {
			onRowCalled = true;
			return true;
		});
		QVERIFY(!onRowCalled);  // the very first step failed with SQLITE_BUSY

		exec(locker, "ROLLBACK");
		sqlite3_close(locker);
		sqlite3_close(db);
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

	void execStatementSucceeds() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT)");
		const bool ok = execStatement(db, "INSERT INTO t VALUES (?)", QStringLiteral("ctx"),
			[](sqlite3_stmt* stmt) { bindText(stmt, 1, QStringLiteral("x")); });
		QVERIFY(ok);
		sqlite3_close(db);
	}

	void execStatementFailsOnMalformedSql() {
		sqlite3* db = openMemoryDb();
		const bool ok = execStatement(db, "NOT VALID SQL", QStringLiteral("ctx"), [](sqlite3_stmt*) {});
		QVERIFY(!ok);
		sqlite3_close(db);
	}

	void execStatementFailsOnConstraintViolation() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a TEXT UNIQUE)");
		exec(db, "INSERT INTO t VALUES ('dup')");
		const bool ok = execStatement(db, "INSERT INTO t VALUES (?)", QStringLiteral("ctx"),
			[](sqlite3_stmt* stmt) { bindText(stmt, 1, QStringLiteral("dup")); });
		QVERIFY(!ok);
		sqlite3_close(db);
	}

	void inTransactionCommitsOnSuccess() {
		sqlite3* db = openMemoryDb();
		exec(db, "CREATE TABLE t (a INTEGER)");
		const bool ok = inTransaction(db, QStringLiteral("ctx"), [&db]() {
			return execStatement(db, "INSERT INTO t VALUES (1)", QStringLiteral("ctx"), [](sqlite3_stmt*) {});
		});
		QVERIFY(ok);
		sqlite3_stmt* check = nullptr;
		sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM t", -1, &check, nullptr);
		sqlite3_step(check);
		QCOMPARE(sqlite3_column_int(check, 0), 1);
		sqlite3_finalize(check);
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
		sqlite3_stmt* check = nullptr;
		sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM t", -1, &check, nullptr);
		sqlite3_step(check);
		QCOMPARE(sqlite3_column_int(check, 0), 0);  // rolled back
		sqlite3_finalize(check);
		sqlite3_close(db);
	}

	void inTransactionFailsWhenAlreadyInOne() {
		sqlite3* db = openMemoryDb();
		exec(db, "BEGIN TRANSACTION");
		// The real sqlite3_exec("BEGIN TRANSACTION", ...) call inside
		// inTransaction() fails genuinely: a transaction is already active.
		const bool ok = inTransaction(db, QStringLiteral("ctx"), []() { return true; });
		QVERIFY(!ok);
		exec(db, "ROLLBACK");
		sqlite3_close(db);
	}
};

QTEST_APPLESS_MAIN(TstDbQuery)
#include "tst_db_query.moc"

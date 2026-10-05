// Trip groups (db_groups.cpp): create, rename, delete, reorder, assign.
#include "test_support.h"

#include "db.h"
#include "db_groups.h"
#include "logger.h"

#include <QTemporaryDir>
#include <QtTest>

using namespace TestSupport;

class TstGroups : public QObject {
	Q_OBJECT

private:
	sqlite3* db_ = nullptr;
	QTemporaryDir logDir_;
	QString logPath_;

	QVariant groupOfTrip(int id) {
		return queryValue(QStringLiteral("SELECT group_id FROM trips WHERE id=%1").arg(id));
	}

	QStringList groupNames() {
		QStringList out;
		for (const TripGroup& g : queryAllGroups(db_))
			out << g.name;
		return out;
	}

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		isolateFiles();
		logPath_ = logDir_.filePath(QStringLiteral("groups.log"));
		Logger::init(Logger::Warning, logPath_);
	}
	void init() {
		removeDatabase();
		migrate_db();
		db_ = connect_db_readwrite();
		QVERIFY(db_);
	}
	void cleanup() {
		sqlite3_close(db_);
		db_ = nullptr;
	}

	void emptyListWithoutGroups() {
		QVERIFY(queryAllGroups(db_).empty());
	}

	void insertTrimsAndAppendsInOrder() {
		const int a = insertGroup(db_, "  Training  ");
		const int b = insertGroup(db_, "Airline Ops");
		QVERIFY(a > 0);
		QVERIFY(b > 0);
		QCOMPARE(groupNames(), (QStringList{ "Training", "Airline Ops" }));
		QCOMPARE(queryValue(QStringLiteral("SELECT sort_order FROM trip_groups WHERE id=%1").arg(b)).toInt(), 1);
	}

	void insertRejectsBlankAndDuplicates() {
		QVERIFY(insertGroup(db_, "Training") > 0);
		QCOMPARE(insertGroup(db_, "   "), 0);
		QCOMPARE(insertGroup(db_, "training"), 0);
		QCOMPARE(insertGroup(db_, " TRAINING "), 0);
		QVERIFY(insertGroup(db_, QString::fromUtf8("München")) > 0);
		QCOMPARE(insertGroup(db_, QString::fromUtf8("MÜNCHEN")), 0); // non-ASCII case folding too
		QCOMPARE(groupNames().size(), 2);
	}

	// Each query insertGroup runs logs its own failure, including the one
	// that finds where the new group goes.
	void insertLogsWhenItCantFindWhereTheGroupGoes() {
		exec(db_, "DROP TABLE trip_groups");
		QCOMPARE(insertGroup(db_, "Training"), 0);
		QVERIFY(warningLogged(logPath_, { QStringLiteral("insertGroup"), QStringLiteral("MAX(sort_order)") }));
	}

	void nameExistsIgnoresCaseAndTheExcludedGroup() {
		const int a = insertGroup(db_, QString::fromUtf8("München"));
		const int b = insertGroup(db_, "Ops");
		QVERIFY(groupNameExists(db_, "ops", 0));
		QVERIFY(groupNameExists(db_, QString::fromUtf8("MÜNCHEN"), 0));
		QVERIFY(!groupNameExists(db_, "Training", 0));
		// A group's own name doesn't count when it is excluded (renaming it).
		QVERIFY(!groupNameExists(db_, "OPS", b));
		QVERIFY(groupNameExists(db_, "OPS", a));
	}

	void renameChecksBlankAndCollisions() {
		const int a = insertGroup(db_, "Training");
		const int b = insertGroup(db_, "Ops");
		QVERIFY(renameGroup(db_, a, "  Flight Training "));
		QVERIFY(!renameGroup(db_, a, ""));
		QVERIFY(!renameGroup(db_, a, "ops"));
		QVERIFY(renameGroup(db_, b, "OPS")); // its own name in a new case
		QCOMPARE(groupNames(), (QStringList{ "Flight Training", "OPS" }));
	}

	void tripCountsAndAssignment() {
		const int g = insertGroup(db_, "Training");
		addTrip(1);
		addTrip(2);
		QVERIFY(setTripGroup(db_, 1, g));
		QCOMPARE(groupOfTrip(1).toInt(), g);
		QCOMPARE(queryAllGroups(db_)[0].tripCount, 1);
		QVERIFY(setTripGroup(db_, 1, 0));
		QVERIFY(groupOfTrip(1).isNull());
		QCOMPARE(queryAllGroups(db_)[0].tripCount, 0);
	}

	void deleteUngroupsItsTrips() {
		const int g = insertGroup(db_, "Training");
		const int keep = insertGroup(db_, "Keep");
		addTrip(1);
		addTrip(2);
		setTripGroup(db_, 1, g);
		setTripGroup(db_, 2, keep);
		QVERIFY(deleteGroup(db_, g));
		QCOMPARE(groupNames(), (QStringList{ "Keep" }));
		QVERIFY(groupOfTrip(1).isNull());
		QCOMPARE(groupOfTrip(2).toInt(), keep);
	}

	void failedDeleteRollsBackUngrouping() {
		const int g = insertGroup(db_, "Training");
		addTrip(1);
		setTripGroup(db_, 1, g);
		// The ungrouping UPDATE succeeds, then the group's DELETE fails.
		QCOMPARE(sqlite3_exec(db_, "CREATE TRIGGER block_delete BEFORE DELETE ON trip_groups "
			"BEGIN SELECT RAISE(ABORT, 'blocked'); END;", nullptr, nullptr, nullptr), SQLITE_OK);
		QVERIFY(!deleteGroup(db_, g));
		QCOMPARE(groupNames(), (QStringList{ "Training" }));
		QCOMPARE(groupOfTrip(1).toInt(), g);
	}

	void reorderPersistsAndIgnoresMissingIds() {
		const int a = insertGroup(db_, "A");
		const int b = insertGroup(db_, "B");
		const int c = insertGroup(db_, "C");
		QVERIFY(reorderGroups(db_, { c, 999, a, b }));
		QCOMPARE(groupNames(), (QStringList{ "C", "A", "B" }));
	}

	// A failed update (logged) rolls back the ones before it.
	void failedReorderKeepsTheOldOrder() {
		const int a = insertGroup(db_, "A");
		const int b = insertGroup(db_, "B");
		const int c = insertGroup(db_, "C");
		QCOMPARE(sqlite3_exec(db_, QStringLiteral("CREATE TRIGGER block_c BEFORE UPDATE ON trip_groups WHEN OLD.id = %1 "
			"BEGIN SELECT RAISE(ABORT, 'blocked'); END;").arg(c).toUtf8().constData(), nullptr, nullptr, nullptr), SQLITE_OK);
		QVERIFY(!reorderGroups(db_, { b, a, c })); // b and a are updated, then c fails
		QCOMPARE(groupNames(), (QStringList{ "A", "B", "C" }));
		QVERIFY(warningLogged(logPath_, { QStringLiteral("reorderGroups"), QStringLiteral("blocked") }));
	}

	void equalSortOrderFallsBackToName() {
		QCOMPARE(sqlite3_exec(db_, "INSERT INTO trip_groups (name, sort_order) VALUES ('beta',0),('Alpha',0);", nullptr, nullptr, nullptr), SQLITE_OK);
		QCOMPARE(groupNames(), (QStringList{ "Alpha", "beta" }));
	}
};

QTEST_MAIN(TstGroups)
#include "tst_groups.moc"

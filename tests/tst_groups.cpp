// Trip groups (db_groups.cpp): create, rename, delete, reorder, assign.
#include "test_support.h"

#include "db.h"
#include "db_groups.h"

#include <QtTest>

using namespace TestSupport;

class TstGroups : public QObject {
	Q_OBJECT

private:
	sqlite3* db_ = nullptr;

	void addTrip(int id) {
		const QByteArray sql = "INSERT INTO trips (id,title,atc_airline,atc_flight_number,atc_id,atc_model,atc_type,departure_latitude,"
			"departure_longitude,departure_zulu_time,departure_local_time) VALUES (" + QByteArray::number(id) + ",'T','A','1','I','M','T',0,0,'z','l');";
		QCOMPARE(sqlite3_exec(db_, sql.constData(), nullptr, nullptr, nullptr), SQLITE_OK);
	}

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
	void initTestCase() { isolateFiles(); }
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

	void reorderPersistsAndIgnoresMissingIds() {
		const int a = insertGroup(db_, "A");
		const int b = insertGroup(db_, "B");
		const int c = insertGroup(db_, "C");
		QVERIFY(reorderGroups(db_, { c, 999, a, b }));
		QCOMPARE(groupNames(), (QStringList{ "C", "A", "B" }));
	}

	void equalSortOrderFallsBackToName() {
		QCOMPARE(sqlite3_exec(db_, "INSERT INTO trip_groups (name, sort_order) VALUES ('beta',0),('Alpha',0);", nullptr, nullptr, nullptr), SQLITE_OK);
		QCOMPARE(groupNames(), (QStringList{ "Alpha", "beta" }));
	}
};

QTEST_MAIN(TstGroups)
#include "tst_groups.moc"

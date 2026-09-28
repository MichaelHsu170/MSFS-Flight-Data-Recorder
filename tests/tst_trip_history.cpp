// Trip History (trip_history_panel.cpp): the table model's display rules, and
// the panel's loading, selection, group filter and row context menu.
#include "test_support.h"

#include "app_settings.h"
#include "db.h"
#include "db_groups.h"
#include "trip_history_panel.h"

#include <QComboBox>
#include <QLabel>
#include <QMenu>
#include <QTableView>
#include <QtTest>

using namespace TestSupport;

Q_DECLARE_METATYPE(std::shared_ptr<TripDataset>)
Q_DECLARE_METATYPE(std::vector<TripSummary>)

namespace {

TripSummary summary(int id, const char* dep, const char* dest, TripStatus status = TripStatus::Completed, int groupId = 0) {
	TripSummary t;
	t.id = id;
	t.departureZuluTime = QString::fromLatin1(dep);
	t.destinationZuluTime = QString::fromLatin1(dest);
	t.status = status;
	t.groupId = groupId;
	return t;
}

void insertTrip(int id, const char* dep, const char* dest, int groupId = 0) {
	sqlite3* db = connect_db_readwrite();
	QVERIFY(db);
	const QString sql = QStringLiteral("INSERT INTO trips (id,title,atc_airline,atc_flight_number,atc_id,atc_model,atc_type,"
		"departure_latitude,departure_longitude,departure_zulu_time,departure_local_time,destination_zulu_time,group_id) "
		"VALUES (%1,'Trip %1','AIR','%1','I','M','T',0,0,'%2','l',%3,%4);")
		.arg(id).arg(dep).arg(dest ? QStringLiteral("'%1'").arg(dest) : QStringLiteral("NULL"))
		.arg(groupId ? QString::number(groupId) : QStringLiteral("NULL"));
	QCOMPARE(sqlite3_exec(db, sql.toUtf8().constData(), nullptr, nullptr, nullptr), SQLITE_OK);
	sqlite3_close(db);
}

const char* kDep = "2026-01-01T10:00:00.000+00:00_4";
const char* kArr = "2026-01-01T11:00:00.000+00:00_4";

}

class TstTripHistory : public QObject {
	Q_OBJECT

private:
	static QTableView* view(TripHistoryPanel& p) { return p.findChild<QTableView*>(); }
	static QComboBox* groupCombo(TripHistoryPanel& p) { return p.findChild<QComboBox*>(); }
	static QString totalText(TripHistoryPanel& p) {
		for (QLabel* l : p.findChildren<QLabel*>())
			if (l->text().startsWith("Total:"))
				return l->text();
		return QString();
	}
	static QString cell(QTableView* v, int row, int column) {
		return v->model()->index(row, column).data().toString();
	}
	static void clickRow(QTableView* v, int row) {
		const QRect rect = v->visualRect(v->model()->index(row, 0));
		QTest::mouseClick(v->viewport(), Qt::LeftButton, Qt::NoModifier, rect.center());
	}
	static void openRowMenu(QTableView* v, int row) {
		emit v->customContextMenuRequested(v->visualRect(v->model()->index(row, 0)).center());
	}
	static int rowOfTrip(QTableView* v, int tripId) {
		for (int r = 0; r < v->model()->rowCount(); ++r)
			if (cell(v, r, TripHistoryModel::TitleColumn) == QStringLiteral("Trip %1").arg(tripId))
				return r;
		return -1;
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() {
		removeDatabase();
		removeSettings();
	}

	// --- Model display rules ---

	void durationsRoundUpToTheMinute_data() {
		QTest::addColumn<QString>("arrival");
		QTest::addColumn<QString>("expected");
		QTest::newRow("exact hour") << "2026-01-01T11:00:00.000+00:00_4" << "1h 00m";
		QTest::newRow("30 s past") << "2026-01-01T11:00:30.000+00:00_4" << "1h 01m";
		QTest::newRow("under a minute") << "2026-01-01T10:00:10.000+00:00_4" << "0h 01m";
		QTest::newRow("zero") << "2026-01-01T10:00:00.000+00:00_4" << "0h 00m";
		QTest::newRow("over a day") << "2026-01-02T11:00:00.000+00:00_5" << "1d 1h 00m";
		QTest::newRow("arrival before departure") << "2026-01-01T09:00:00.000+00:00_4" << "-";
		QTest::newRow("open trip") << "" << "-";
		QTest::newRow("unparseable") << "garbage" << "-";
	}

	void durationsRoundUpToTheMinute() {
		QFETCH(QString, arrival);
		QFETCH(QString, expected);
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, arrival.toLatin1().constData()) });
		QCOMPARE(model.index(0, TripHistoryModel::DurationColumn).data().toString(), expected);
	}

	void displayColumns() {
		TripSummary t = summary(1, kDep, kArr);
		t.title = "A320";
		t.atcAirline = "AIR";
		t.atcFlightNumber = "123";
		t.departureIcao = "AAAA";
		t.departureName = "Alpha";
		t.destinationIcao = "BBBB";
		t.departureRwy = "09";
		TripHistoryModel model;
		model.setTrips({ t });
		auto text = [&model](int c) { return model.index(0, c).data().toString(); };
		QCOMPARE(text(TripHistoryModel::TitleColumn), QStringLiteral("A320"));
		QCOMPARE(text(TripHistoryModel::FlightColumn), QStringLiteral("AIR 123"));
		QCOMPARE(text(TripHistoryModel::GroupColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DepartureColumn), QStringLiteral("AAAA [Alpha]"));
		QCOMPARE(text(TripHistoryModel::DestinationColumn), QStringLiteral("BBBB"));
		QCOMPARE(text(TripHistoryModel::DepartureRwyColumn), QStringLiteral("09"));
		QCOMPARE(text(TripHistoryModel::DestinationRwyColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DepartureRegionColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DepartureTimeColumn), QString::fromLatin1(kDep));
		QCOMPARE(model.index(0, TripHistoryModel::DepartureColumn).data(Qt::ToolTipRole).toString(), QStringLiteral("AAAA [Alpha]"));
		QCOMPARE(model.headerData(TripHistoryModel::TitleColumn, Qt::Horizontal, Qt::DisplayRole).toString(), QStringLiteral("Aircraft"));
		QCOMPARE(model.headerData(TripHistoryModel::DurationColumn, Qt::Horizontal, Qt::DisplayRole).toString(), QStringLiteral("Duration"));
		QCOMPARE(model.columnCount(), (int)TripHistoryModel::ColumnCount);
	}

	void groupNameShownOnlyForGroupedTrips() {
		TripSummary grouped = summary(1, kDep, kArr, TripStatus::Completed, 5);
		grouped.groupName = "Training";
		TripHistoryModel model;
		model.setTrips({ grouped });
		QCOMPARE(model.index(0, TripHistoryModel::GroupColumn).data().toString(), QStringLiteral("Training"));
	}

	void statusColorsAndSelectability() {
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, "", TripStatus::Live), summary(2, kDep, "", TripStatus::Open), summary(3, kDep, kArr) });
		QCOMPARE(model.index(0, 0).data(Qt::BackgroundRole).value<QBrush>().color(), QColor(200, 255, 200));
		QCOMPARE(model.index(1, 0).data(Qt::BackgroundRole).value<QBrush>().color(), QColor(255, 235, 180));
		QVERIFY(!model.index(2, 0).data(Qt::BackgroundRole).isValid());
		model.setHoveredRow(2);
		QCOMPARE(model.index(2, 0).data(Qt::BackgroundRole).value<QBrush>().color(), QColor(220, 230, 245));
		QVERIFY(!(model.flags(model.index(0, 0)) & Qt::ItemIsSelectable));
		QVERIFY(model.flags(model.index(1, 0)) & Qt::ItemIsSelectable);
	}

	void groupFilterAndTotals() {
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, kArr, TripStatus::Completed, 5), summary(2, kDep, "2026-01-01T10:30:00.000+00:00_4"),
			summary(3, kDep, "", TripStatus::Open) });
		QCOMPARE(model.rowCount(), 3);
		QCOMPARE(model.totalDurationText(), QStringLiteral("1h 30m"));
		model.setGroupFilter(5);
		QCOMPARE(model.rowCount(), 1);
		QCOMPARE(model.tripAt(0)->id, 1);
		model.setGroupFilter(0);
		QCOMPARE(model.rowCount(), 2);
		QCOMPARE(model.totalDurationText(), QStringLiteral("0h 30m"));
		model.setTrips({ summary(4, kDep, "", TripStatus::Open) });
		model.setGroupFilter(-1);
		QCOMPARE(model.totalDurationText(), QStringLiteral("-"));
		QVERIFY(model.tripAt(5) == nullptr);
		QVERIFY(model.tripAt(-1) == nullptr);
	}

	// --- Panel ---

	void listsTripsNewestFirstWithTotal() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		insertTrip(2, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QTableView* v = view(panel);
		QCOMPARE(v->model()->rowCount(), 2);
		QCOMPARE(cell(v, 0, TripHistoryModel::TitleColumn), QStringLiteral("Trip 2"));
		QCOMPARE(totalText(panel), QStringLiteral("Total: 2 trips | 2h 00m"));
	}

	void initialOverviewCarriesEveryTrip() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		panel.showInitialOverview();
		QCOMPARE(deselected.count(), 1);
		QCOMPARE(deselected.at(0).at(0).value<std::vector<TripSummary>>().size(), size_t(1));
	}

	void groupFilterNarrowsTableAndOverview() {
		FlightDriver sim;
		sqlite3* db = connect_db_readwrite();
		const int training = insertGroup(db, "Training");
		insertGroup(db, "Ops");
		sqlite3_close(db);
		insertTrip(1, kDep, kArr, training);
		insertTrip(2, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QComboBox* combo = groupCombo(panel);
		QStringList items;
		for (int i = 0; i < combo->count(); ++i)
			items << combo->itemText(i);
		QCOMPARE(items, (QStringList{ "All Trips", "Ungrouped", "Training", "Ops" }));
		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		combo->setCurrentIndex(2);
		QCOMPARE(view(panel)->model()->rowCount(), 1);
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("Training"));
		QCOMPARE(deselected.count(), 1);
		QCOMPARE(deselected.at(0).at(0).value<std::vector<TripSummary>>().size(), size_t(1));
		QCOMPARE(deselected.at(0).at(0).value<std::vector<TripSummary>>()[0].groupRank, 1);
		combo->setCurrentIndex(1);
		QCOMPARE(view(panel)->model()->rowCount(), 1);
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::TitleColumn), QStringLiteral("Trip 2"));
	}

	void clickingATripLoadsEverythingAboutIt() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.ticks(3);
		sim.simEvent(EVENT_GEAR_UP, 1);
		sim.setOnGround(false);
		sim.tick();
		QThread::msleep(600);
		sim.tick();
		sim.setOnGround(true);
		sim.tick();
		sim.endTrip();
		QVERIFY(waitFor([tripId] { return queryValue(QStringLiteral("SELECT COUNT(*) FROM trip_events WHERE trip=%1").arg(tripId)).toInt() == 1; }));

		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		clickRow(view(panel), 0);
		QVERIFY(!view(panel)->isEnabled()); // locked while loading
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		const std::shared_ptr<TripDataset> d = ready.at(0).at(0).value<std::shared_ptr<TripDataset>>();
		QCOMPARE(d->tripId, tripId);
		QCOMPARE(d->aircraftTitle, QStringLiteral("Test Aircraft"));
		QCOMPARE(d->departureZuluTime, QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		QCOMPARE(d->points.size(), size_t(7));
		QCOMPARE(d->liftoffPoints.size(), size_t(1));
		QCOMPARE(d->touchdowns.size(), size_t(1));
		QCOMPARE(d->events.size(), size_t(1));
		QVERIFY(d->events[0].sampleIndex >= 0);
		QVERIFY(!view(panel)->isEnabled()); // until the views finish rendering
		panel.setLoadingFinished();
		QVERIFY(view(panel)->isEnabled());
	}

	void selectTripByIdLoadsOnlySelectableNewTrips() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		insertTrip(2, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(99); // not listed
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		QCOMPARE(ready.at(0).at(0).value<std::shared_ptr<TripDataset>>()->tripId, 1);
		panel.setLoadingFinished();
		panel.selectTripById(1); // already shown
		QTest::qWait(200);
		QCOMPARE(ready.count(), 1);
		QCOMPARE(view(panel)->selectionModel()->selectedRows().value(0).row(), rowOfTrip(view(panel), 1));
	}

	void liveTripAppearsAndCannotBeLoaded() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		const int tripId = sim.startTrip(); // recordingStateChanged refreshes the table
		QTableView* v = view(panel);
		QCOMPARE(v->model()->rowCount(), 2);
		const int row = rowOfTrip(v, tripId) < 0 ? 0 : rowOfTrip(v, tripId);
		QCOMPARE(v->model()->index(row, 0).data(Qt::BackgroundRole).value<QBrush>().color(), QColor(200, 255, 200));
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		clickRow(v, row);
		panel.selectTripById(tripId);
		QTest::qWait(200);
		QCOMPARE(ready.count(), 0);
		sim.endTrip();
		QVERIFY(waitFor([v, row] { return !v->model()->index(row, 0).data(Qt::BackgroundRole).isValid(); }));
	}

	void deleteFromTheRowMenuAfterConfirming() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		insertTrip(2, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* confirm) { clickDialogButton(confirm, "Yes"); });
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), 1));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QCOMPARE(view(panel)->model()->rowCount(), 1);
		QCOMPARE(deselected.count(), 1);
	}

	void cancellingDeleteKeepsTheTrip() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* confirm) { clickDialogButton(confirm, "Cancel"); });
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
	}

	void setGroupFromTheRowMenu() {
		FlightDriver sim;
		sqlite3* db = connect_db_readwrite();
		const int training = insertGroup(db, "Training");
		sqlite3_close(db);
		insertTrip(1, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* submenu) { chooseMenuItem(submenu, "Training"); });
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(queryValue("SELECT group_id FROM trips WHERE id=1").toInt(), training);
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("Training"));

		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* submenu) { chooseMenuItem(submenu, "Ungrouped"); });
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QVERIFY(queryValue("SELECT group_id FROM trips WHERE id=1").isNull());
	}

	void deselectAndResetZoomOnlyForTheSelectedTrip() {
		FlightDriver sim;
		insertTrip(1, kDep, kArr);
		insertTrip(2, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();

		QStringList otherRowItems;
		onNextModal([&otherRowItems](QWidget* menu) {
			for (QAction* a : menu->actions())
				if (!a->isSeparator())
					otherRowItems << a->text();
			menu->close();
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), 2));
		QVERIFY(!otherRowItems.contains("Deselect"));
		QVERIFY(!otherRowItems.contains("Set Group")); // another trip is selected
		QVERIFY(otherRowItems.contains("Delete Trip"));

		QSignalSpy zoom(&panel, &TripHistoryPanel::zoomResetRequested);
		onNextModal([](QWidget* menu) { chooseMenuItem(menu, "Reset Zoom"); });
		openRowMenu(view(panel), rowOfTrip(view(panel), 1));
		QCOMPARE(zoom.count(), 1);

		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		onNextModal([](QWidget* menu) { chooseMenuItem(menu, "Deselect"); });
		openRowMenu(view(panel), rowOfTrip(view(panel), 1));
		QCOMPARE(deselected.count(), 1);
		QVERIFY(view(panel)->selectionModel()->selectedRows().isEmpty());
	}

	void resizedColumnWidthsArePersisted() {
		FlightDriver sim;
		{
			TripHistoryPanel panel(sim.bridge());
			view(panel)->setColumnWidth(TripHistoryModel::GroupColumn, 99);
		}
		QCOMPARE(AppSettings::instance().tripHistoryColumnWidths().value("GroupColumn"), 99);
		TripHistoryPanel again(sim.bridge());
		QCOMPARE(view(again)->columnWidth(TripHistoryModel::GroupColumn), 99);
	}
};

QTEST_MAIN(TstTripHistory)
#include "tst_trip_history.moc"

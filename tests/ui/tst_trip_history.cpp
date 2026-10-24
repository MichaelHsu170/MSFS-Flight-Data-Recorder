// Trip History (trip_history_panel.cpp): the table model's display rules, and
// the panel's loading, selection, group filter and row context menu.
#include "test_support.h"

#include "app_settings.h"
#include "trip_history_panel.h"

#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QStyleFactory>
#include <QTableView>
#include <QTextDocument>
#include <QTextLayout>
#include <QTextTable>
#include <QToolTip>
#include <QEnterEvent>
#include <QtTest>

#include <memory>
#include <mutex>

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

const char* kDep = "2026-01-01T10:00:00.000+00:00_4";
const char* kArr = "2026-01-01T11:00:00.000+00:00_4";

}

class TstTripHistory : public QObject {
	Q_OBJECT

private:
	static QTableView* view(TripHistoryPanel& p) { return p.findChild<QTableView*>(); }
	static QComboBox* groupCombo(TripHistoryPanel& p) { return p.findChild<QComboBox*>(); }
	static QLabel* totalLabel(TripHistoryPanel& p) {
		for (QLabel* l : p.findChildren<QLabel*>())
			if (l->text().startsWith("Total:"))
				return l;
		return nullptr;
	}
	static QString totalText(TripHistoryPanel& p) {
		QLabel* l = totalLabel(p);
		return l ? l->text() : QString();
	}
	// aircraftTotals() as "model|trips|duration|hours" lines.
	static QStringList rowsOf(const TripHistoryModel& model) {
		QStringList rows;
		for (const TripHistoryModel::AircraftTotals& r : model.aircraftTotals())
			rows << QStringLiteral("%1|%2|%3|%4").arg(r.model).arg(r.trips).arg(r.duration, r.hours);
		return rows;
	}
	// Calls f(row, column, cell) for each cell of the first table in html, as
	// Qt's rich text (the tooltip's renderer) lays it out, textWidth px wide
	// (-1: as wide as it needs).
	template <typename F>
	static void forEachTableCell(const QString& html, F f, qreal textWidth = -1) {
		QTextDocument doc;
		doc.setHtml(html);
		doc.setTextWidth(textWidth);
		doc.size(); // lays it out
		for (QTextFrame* frame : doc.rootFrame()->childFrames()) {
			if (auto* table = qobject_cast<QTextTable*>(frame)) {
				for (int r = 0; r < table->rows(); ++r)
					for (int c = 0; c < table->columns(); ++c)
						f(r, c, table->cellAt(r, c));
				return;
			}
		}
	}
	// The table's rows as "cell|cell|..." lines (none if html has no table),
	// the non-breaking spaces Qt turns a no-wrap cell's spaces into read as
	// the spaces they show as.
	static QStringList tableCells(const QString& html) {
		QStringList rows;
		forEachTableCell(html, [&rows](int r, int c, const QTextTableCell& cell) {
			if (c == 0)
				rows << QString();
			rows[r] += (c ? QStringLiteral("|") : QString()) + cell.firstCursorPosition().block().text().replace(QChar::Nbsp, u' ');
		});
		return rows;
	}
	// Each row's cell alignments, one letter per cell: L(eft) or R(ight).
	static QStringList tableAlignments(const QString& html) {
		QStringList rows;
		forEachTableCell(html, [&rows](int r, int c, const QTextTableCell& cell) {
			if (c == 0)
				rows << QString();
			rows[r] += (cell.firstCursorPosition().blockFormat().alignment() & Qt::AlignRight) ? 'R' : 'L';
		});
		return rows;
	}
	// Each row's line count per cell ("1111"), laid out only 1 px wide, so a
	// cell that can wrap does.
	static QStringList tableLineCounts(const QString& html) {
		QStringList rows;
		forEachTableCell(html, [&rows](int r, int c, const QTextTableCell& cell) {
			if (c == 0)
				rows << QString();
			rows[r] += QString::number(cell.firstCursorPosition().block().layout()->lineCount());
		}, 1);
		return rows;
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
	// Makes tripId look like a trip whose last samples are still being written.
	static void markSaving(FlightDriver& sim, int tripId) {
		std::lock_guard<std::mutex> lock(sim.status().flushing_trip_ids_mutex);
		sim.status().flushing_trip_ids.insert(tripId);
	}
	// Answers the next modal box, keeping its title. Closes whatever box it
	// is, so a missed refusal fails the test instead of hanging it.
	static void keepTitleOfNextModal(QString& title) {
		onNextModal([&title](QWidget* box) {
			title = box->windowTitle();
			box->close();
		});
	}

private slots:
	void initTestCase() {
		isolateFiles();
		// Save dialogs a test can answer (see saveFileDialogAs()).
		QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
	}
	void init() {
		removeDatabase();
		removeSettings();
	}
	void cleanup() { cancelPendingModals(); } // no dialog action outlives its test

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
		t.destinationRegion = "BB";
		t.departureRwy = "09";
		TripHistoryModel model;
		model.setTrips({ t });
		auto text = [&model](int c) { return model.index(0, c).data().toString(); };
		QCOMPARE(text(TripHistoryModel::TitleColumn), QStringLiteral("A320"));
		QCOMPARE(text(TripHistoryModel::FlightColumn), QStringLiteral("AIR 123"));
		QCOMPARE(text(TripHistoryModel::GroupColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DepartureColumn), QStringLiteral("AAAA (Alpha)"));
		QCOMPARE(text(TripHistoryModel::DestinationColumn), QStringLiteral("BBBB"));
		QCOMPARE(text(TripHistoryModel::DepartureRwyColumn), QStringLiteral("09"));
		QCOMPARE(text(TripHistoryModel::DestinationRwyColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DepartureRegionColumn), QStringLiteral("-"));
		QCOMPARE(text(TripHistoryModel::DestinationRegionColumn), QStringLiteral("BB"));
		QCOMPARE(text(TripHistoryModel::DepartureTimeColumn), QString::fromLatin1(kDep));
		QCOMPARE(text(TripHistoryModel::DestinationTimeColumn), QString::fromLatin1(kArr));
		QCOMPARE(model.index(0, TripHistoryModel::DepartureColumn).data(Qt::ToolTipRole).toString(), QStringLiteral("AAAA (Alpha)"));
		// The table's columns, left to right.
		QStringList headers;
		for (int c = 0; c < model.columnCount(); ++c)
			headers << model.headerData(c, Qt::Horizontal, Qt::DisplayRole).toString();
		QCOMPARE(headers, (QStringList{ "Aircraft", "Flight", "Group", "Region", "From", "Dep Rwy", "Region", "To", "Dest Rwy",
			"Departed (Z)", "Arrived (Z)", "Duration" }));
		// Out-of-enum section/column, wrong orientation: all fall through to the
		// shared "nothing to show" return rather than a matching case.
		QVERIFY(!model.headerData(TripHistoryModel::ColumnCount, Qt::Horizontal, Qt::DisplayRole).isValid());
		QVERIFY(!model.headerData(TripHistoryModel::TitleColumn, Qt::Vertical, Qt::DisplayRole).isValid());
	}

	void dataAndFlagsAreEmptyForInvalidOrOutOfRangeIndex() {
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, kArr) });
		QVERIFY(!model.data(QModelIndex(), Qt::DisplayRole).isValid());
		QVERIFY(!model.data(model.index(1, TripHistoryModel::TitleColumn), Qt::DisplayRole).isValid());
		QCOMPARE(model.flags(QModelIndex()), Qt::ItemFlags());
		QCOMPARE(model.flags(model.index(1, TripHistoryModel::TitleColumn)), Qt::ItemFlags());
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

		// Re-hovering the same row is a no-op: no extra dataChanged for either
		// the old or the new row (both are the same row here).
		QSignalSpy changed(&model, &QAbstractItemModel::dataChanged);
		model.setHoveredRow(2);
		QCOMPARE(changed.count(), 0);
	}

	void groupFilterAndTotals() {
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, kArr, TripStatus::Completed, 5), summary(2, kDep, "2026-01-01T10:30:00.000+00:00_4"),
			summary(3, kDep, "", TripStatus::Open) });
		QCOMPARE(model.rowCount(), 3);
		QCOMPARE(model.totalText(), QStringLiteral("3 trips | 1h 30m (1.5 h)"));
		model.setGroupFilter(5);
		QCOMPARE(model.rowCount(), 1);
		QCOMPARE(model.tripAt(0)->id, 1);
		model.setGroupFilter(0);
		QCOMPARE(model.rowCount(), 2);
		QCOMPARE(model.totalText(), QStringLiteral("2 trips | 0h 30m (0.5 h)"));
		model.setTrips({ summary(4, kDep, "", TripStatus::Open) });
		model.setGroupFilter(-1);
		QCOMPARE(model.totalText(), QStringLiteral("1 trip | -"));
		QCOMPARE(rowsOf(model), (QStringList{ "-|1|-|-" }));
		model.setTrips({});
		QCOMPARE(rowsOf(model), QStringList());
		QVERIFY(model.tripAt(5) == nullptr);
		QVERIFY(model.tripAt(-1) == nullptr);
	}

	// The decimal hours come from the same rounded-up minutes as the duration.
	void totalHoursMatchTheRoundedDuration_data() {
		QTest::addColumn<QString>("arrival");
		QTest::addColumn<QString>("expected");
		QTest::newRow("exact hour") << "2026-01-01T11:00:00.000+00:00_4" << "1 trip | 1h 00m (1.0 h)";
		QTest::newRow("30 s past") << "2026-01-01T11:00:30.000+00:00_4" << "1 trip | 1h 01m (1.0 h)";
		// 2m 31s is 0.04 h, but it shows as 3 minutes: 0.05 h, which rounds up.
		QTest::newRow("hours of the rounded minutes") << "2026-01-01T10:02:31.000+00:00_4" << "1 trip | 0h 03m (0.1 h)";
		QTest::newRow("rounds to a tenth") << "2026-01-01T11:03:00.000+00:00_4" << "1 trip | 1h 03m (1.1 h)";
		// 0.15 h and 0.35 h: halves whose binary value falls just below, still up.
		QTest::newRow("9 min rounds half up") << "2026-01-01T10:09:00.000+00:00_4" << "1 trip | 0h 09m (0.2 h)";
		QTest::newRow("21 min rounds half up") << "2026-01-01T10:21:00.000+00:00_4" << "1 trip | 0h 21m (0.4 h)";
		QTest::newRow("under half a tenth") << "2026-01-01T10:02:00.000+00:00_4" << "1 trip | 0h 02m (0.0 h)";
		QTest::newRow("over a day") << "2026-01-02T11:00:00.000+00:00_5" << "1 trip | 1d 1h 00m (25.0 h)";
	}

	void totalHoursMatchTheRoundedDuration() {
		QFETCH(QString, arrival);
		QFETCH(QString, expected);
		TripHistoryModel model;
		model.setTrips({ summary(1, kDep, arrival.toLatin1().constData()) });
		QCOMPARE(model.totalText(), expected);
	}

	// Fixtures are ATC models MSFS recorded for real aircraft.
	void aircraftModelFromAtcModel_data() {
		QTest::addColumn<QString>("atcModel");
		QTest::addColumn<QString>("title");
		QTest::addColumn<QString>("model");
		QTest::newRow("localization key") << "ATCCOM.AC_MODEL A320.0.text" << "FenixA320 IAE SL" << "A320";
		QTest::newRow("key with underscore") << "ATCCOM.AC_MODEL_A320.0.text" << "A320neo" << "A320";
		QTest::newRow("key with underscore 2") << "ATCCOM.AC_MODEL_FA18E.0.text" << "F/A-18E Super Hornet" << "FA18E";
		QTest::newRow("$$: prefix") << "$$:C182 RG" << "Carenado C182 RG II N2817E" << "C182 RG";
		QTest::newRow("plain") << "A350-900" << "A350-900 (Default Cabin)" << "A350-900";
		QTest::newRow("padded") << " DHC6 " << "DHC-6-300 Twin Otter Wheels" << "DHC6";
		QTest::newRow("no ATC model") << "" << " Some Glider " << "Some Glider";
		QTest::newRow("empty $$: prefix") << "$$:" << "Some Glider" << "Some Glider";
		QTest::newRow("no ATC model or title") << "" << "" << "-";
		// Keys cut at the recorded field's 31 characters, and one whose model
		// name has dots of its own.
		QTest::newRow("key cut in \"text\"") << "ATCCOM.AC_MODEL A350-1000.0.tex" << "A350-1000 (Default Cabin)" << "A350-1000";
		QTest::newRow("key cut after the dot") << "ATCCOM.AC_MODEL Albatros L39.0." << "L-39 Albatros" << "Albatros L39";
		QTest::newRow("key cut at the first dot") << "ATCCOM.AC_MODEL Beechcraft 350." << "King Air 350i" << "Beechcraft 350";
		QTest::newRow("key cut in the name") << "ATCCOM.AC_MODEL Boeing 787-10 D" << "Boeing 787-10" << "Boeing 787-10 D";
		QTest::newRow("dots in the name") << "ATCCOM.AC_MODEL F.1.0.text" << "F.1 Camel" << "F.1";
	}

	void aircraftModelFromAtcModel() {
		QFETCH(QString, atcModel);
		QFETCH(QString, title);
		QFETCH(QString, model);
		TripSummary t = summary(1, kDep, kArr);
		t.atcModel = atcModel;
		t.title = title;
		TripHistoryModel m;
		m.setTrips({ t });
		QCOMPARE(rowsOf(m), (QStringList{ model + QStringLiteral("|1|1h 00m|1.0") }));
	}

	void aircraftTotalsSumPerModelInNameOrder() {
		auto trip = [](int id, const char* atcModel, const char* arrival, int groupId = 0) {
			TripSummary t = summary(id, kDep, arrival, arrival[0] ? TripStatus::Completed : TripStatus::Open, groupId);
			t.atcModel = QString::fromLatin1(atcModel);
			return t;
		};
		TripHistoryModel model;
		model.setTrips({
			trip(1, "ATCCOM.AC_MODEL A320.0.text", "2026-01-01T11:00:00.000+00:00_4"),
			trip(2, "ATCCOM.AC_MODEL A320.0.text", "2026-01-01T10:30:00.000+00:00_4", 5),
			trip(3, "ATCCOM.AC_MODEL_A320.0.text", ""), // the key's other form, same model
			trip(4, "A350-900", "2026-01-01T12:00:00.000+00:00_4", 5),
			trip(5, "DHC6", "2026-01-01T10:12:00.000+00:00_4"),
			trip(6, "$$:C182 RG", "2026-01-01T10:12:00.000+00:00_4"),
		});
		// Ascending by name, not by hours: the A350-900's 2 h comes after the A320.
		QCOMPARE(rowsOf(model), (QStringList{ "A320|3|1h 30m|1.5", "A350-900|1|2h 00m|2.0", "C182 RG|1|0h 12m|0.2", "DHC6|1|0h 12m|0.2" }));
		model.setGroupFilter(5);
		QCOMPARE(rowsOf(model), (QStringList{ "A320|1|0h 30m|0.5", "A350-900|1|2h 00m|2.0" }));
		// A model whose only trip is still open has no duration or hours.
		model.setGroupFilter(-1);
		model.setTrips({ trip(7, "DHC6", "") });
		QCOMPARE(rowsOf(model), (QStringList{ "DHC6|1|-|-" }));
	}

	// --- Panel ---

	void listsTripsNewestFirstWithTotal() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QTableView* v = view(panel);
		QCOMPARE(v->model()->rowCount(), 2);
		QCOMPARE(cell(v, 0, TripHistoryModel::TitleColumn), QStringLiteral("Trip 2"));
		QCOMPARE(totalText(panel), QStringLiteral("Total: 2 trips | 2h 00m (2.0 h)"));
	}

	void initialOverviewCarriesEveryTrip() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		panel.showInitialOverview();
		QCOMPARE(deselected.count(), 1);
		QCOMPARE(deselected.at(0).at(0).value<std::vector<TripSummary>>().size(), size_t(1));
	}

	void groupFilterNarrowsTableAndOverview() {
		FlightDriver sim;
		const int training = addGroup("Training");
		addGroup("Ops");
		addTrip(1, training, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
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
		// The total and its per-aircraft tooltip follow the filter.
		QCOMPARE(totalText(panel), QStringLiteral("Total: 1 trip | 1h 00m (1.0 h)"));
		QCOMPARE(tableCells(totalLabel(panel)->toolTip()), (QStringList{ "Aircraft|Trips|Duration|Hours", "M|1|1h 00m|1.0" }));
		combo->setCurrentIndex(0);
		QCOMPARE(tableCells(totalLabel(panel)->toolTip()), (QStringList{ "Aircraft|Trips|Duration|Hours", "M|2|2h 00m|2.0" }));
	}

	// The per-aircraft tooltip is a table, numbers right-aligned so they line
	// up, shown the moment the pointer enters the Total label.
	void totalTooltipIsAnInstantTable() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		exec("UPDATE trips SET atc_model = 'A<B & %2 C>';"); // markup or an arg() marker in a name shows as text
		TripHistoryPanel panel(sim.bridge());
		panel.show();
		QVERIFY(QTest::qWaitForWindowExposed(&panel));
		QLabel* label = totalLabel(panel);
		QCOMPARE(tableCells(label->toolTip()), (QStringList{ "Aircraft|Trips|Duration|Hours", "A<B & %2 C>|1|1h 00m|1.0" }));
		QCOMPARE(tableAlignments(label->toolTip()), (QStringList{ "LRRR", "LRRR" }));
		// Never wrapped, however narrow Qt makes the tooltip.
		QCOMPARE(tableLineCounts(label->toolTip()), (QStringList{ "1111", "1111" }));
		QEnterEvent enter(QPointF(5, 5), label->mapToGlobal(QPointF(5, 5)), label->mapToGlobal(QPointF(5, 5)));
		QCoreApplication::sendEvent(label, &enter);
		QVERIFY(QToolTip::isVisible());
		QCOMPARE(QToolTip::text(), label->toolTip());
		QEvent leave(QEvent::Leave);
		QCoreApplication::sendEvent(label, &leave);
		QVERIFY(waitFor([] { return !QToolTip::isVisible(); }, 2000));
	}

	void noTripsMeansNoTotalTooltip() {
		FlightDriver sim;
		TripHistoryPanel panel(sim.bridge());
		QCOMPARE(totalText(panel), QStringLiteral("Total: 0 trips | -"));
		QCOMPARE(totalLabel(panel)->toolTip(), QString());
	}

	void clickingATripLoadsEverythingAboutIt() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.ticks(3);
		sim.simEvent(EVENT_GEAR_UP, 1);
		sim.setOnGround(false);
		sim.tick();
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
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
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
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		const int tripId = sim.startTrip(); // recordingStateChanged refreshes the table
		QTableView* v = view(panel);
		QCOMPARE(v->model()->rowCount(), 2);
		// The live trip is the row titled with the sim's aircraft (makeRecord()).
		const int row = cell(v, 0, TripHistoryModel::TitleColumn) == QStringLiteral("Test Aircraft") ? 0 : 1;
		QCOMPARE(cell(v, row, TripHistoryModel::TitleColumn), QStringLiteral("Test Aircraft"));
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
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
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

	// The confirmation names the airports the same way the table does; "-"
	// where none was found.
	void cancellingDeleteKeepsTheTrip() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		exec("UPDATE trips SET departure_icao='AAAA', departure_name='Alpha' WHERE id=1");
		TripHistoryPanel panel(sim.bridge());
		QString details;
		onNextModal([&details](QWidget* menu) {
			onNextModal([&details](QWidget* confirm) {
				details = static_cast<QMessageBox*>(confirm)->informativeText();
				clickDialogButton(confirm, "Cancel");
			});
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QVERIFY2(details.contains("From: AAAA (Alpha)\nTo: -\n"), qPrintable(details));
	}

	// A delete that fails (here: a child table is gone) says so and keeps the
	// trip listed.
	void failedDeleteShowsAnErrorAndKeepsTheTrip() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		exec("DROP TABLE trip_liftoffs");
		QString error;
		onNextModal([&error](QWidget* menu) {
			onNextModal([&error](QWidget* confirm) {
				onNextModal([&error](QWidget* box) {
					error = static_cast<QMessageBox*>(box)->text();
					clickDialogButton(box, "OK");
				});
				clickDialogButton(confirm, "Yes");
			});
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(error, QStringLiteral("Failed to delete trip data."));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QCOMPARE(view(panel)->model()->rowCount(), 1);
	}

	void setGroupFromTheRowMenu() {
		FlightDriver sim;
		const int training = addGroup("Training");
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* submenu) { chooseMenuItem(submenu, "Training"); });
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(tripRow(1)["group_id"].toInt(), training);
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("Training"));

		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* submenu) { chooseMenuItem(submenu, "Ungrouped"); });
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QVERIFY(tripRow(1)["group_id"].isNull());
	}

	// "Manage Groups…" under the row menu's Set Group opens the same dialog as
	// the panel's button: a group deleted there is gone from the trip's row.
	void manageGroupsFromTheRowMenu() {
		FlightDriver sim;
		addTrip(1, addGroup("Training"), kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("Training"));
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* submenu) {
				onNextModal([](QWidget* dialog) {
					dialog->findChild<QListWidget*>()->setCurrentRow(0);
					onNextModal([](QWidget* confirm) { clickDialogButton(confirm, "Yes"); });
					clickDialogButton(dialog, "Delete");
					static_cast<QDialog*>(dialog)->reject();
				});
				chooseMenuItem(submenu, "Manage Groups…");
			});
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_groups").toInt(), 0);
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("-"));
	}

	// A group change the database refuses says so and leaves the trip as it was.
	void failedSetGroupShowsAnErrorAndKeepsTheGroup() {
		FlightDriver sim;
		addGroup("Training");
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		exec("CREATE TRIGGER refuse BEFORE UPDATE ON trips BEGIN SELECT RAISE(ABORT, 'refused'); END");
		QString error;
		onNextModal([&error](QWidget* menu) {
			onNextModal([&error](QWidget* submenu) {
				onNextModal([&error](QWidget* box) {
					error = static_cast<QMessageBox*>(box)->text();
					clickDialogButton(box, "OK");
				});
				chooseMenuItem(submenu, "Training");
			});
			chooseMenuItem(menu, "Set Group");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(error, QStringLiteral("Failed to update the trip's group."));
		QVERIFY(tripRow(1)["group_id"].isNull());
		QCOMPARE(cell(view(panel), 0, TripHistoryModel::GroupColumn), QStringLiteral("-"));
	}

	void deselectAndResetZoomOnlyForTheSelectedTrip() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
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

	void contextMenuIsSuppressedWhileLoading() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1); // starts loading
		openRowMenu(view(panel), rowOfTrip(view(panel), 1)); // suppressed: a load is already in progress
		QVERIFY(!QApplication::activePopupWidget());
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();
	}

	void callingSetLoadingFinishedTwiceIsHarmless() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();
		QVERIFY(view(panel)->isEnabled());
		panel.setLoadingFinished(); // already finished: no-op
		QVERIFY(view(panel)->isEnabled());
	}

	void reentrantSelectTripByIdWhileLoadingIsIgnored() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1); // starts loading trip 1
		panel.selectTripById(2); // ignored: a load is already in progress
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		QCOMPARE(ready.at(0).at(0).value<std::shared_ptr<TripDataset>>()->tripId, 1);
		panel.setLoadingFinished();
		QTest::qWait(200);
		QCOMPARE(ready.count(), 1);
	}

	void groupRankFallsBackToZeroForAnUnknownGroupId() {
		FlightDriver sim;
		addTrip(1, 999, kDep, kArr); // references a group that was never created
		TripHistoryPanel panel(sim.bridge());
		auto* model = static_cast<TripHistoryModel*>(view(panel)->model());
		QCOMPARE(model->trips()[0].groupRank, 0);
	}

	void selectionSurvivesARefreshTriggeredWhileSelected() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();
		QCOMPARE(view(panel)->selectionModel()->selectedRows().value(0).row(), rowOfTrip(view(panel), 1));

		sim.startTrip(); // recordingStateChanged -> refreshTrips() while trip 1 is still selected
		QCOMPARE(view(panel)->selectionModel()->selectedRows().value(0).row(), rowOfTrip(view(panel), 1));
	}

	void deletingTheSelectedTripClearsItsSelection() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();

		QSignalSpy deselected(&panel, &TripHistoryPanel::tripDeselected);
		onNextModal([](QWidget* menu) {
			onNextModal([](QWidget* confirm) { clickDialogButton(confirm, "Yes"); });
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), 1));
		QCOMPARE(deselected.count(), 1);
		QVERIFY(view(panel)->selectionModel()->selectedRows().isEmpty());
	}

	// A trip whose last samples are still being written can't be deleted:
	// checked when Delete Trip is chosen and again once it's confirmed.
	void deletingATripStillSavingIsRefused_data() {
		QTest::addColumn<bool>("startsSavingWhileConfirming");
		QTest::newRow("saving when chosen") << false;
		QTest::newRow("saving once confirmed") << true;
	}

	void deletingATripStillSavingIsRefused() {
		QFETCH(bool, startsSavingWhileConfirming);
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		if (!startsSavingWhileConfirming)
			markSaving(sim, 1);
		QString refusal;
		onNextModal([&](QWidget* menu) {
			if (startsSavingWhileConfirming) {
				onNextModal([&](QWidget* confirm) {
					markSaving(sim, 1);
					keepTitleOfNextModal(refusal);
					clickDialogButton(confirm, "Yes");
				});
			} else {
				keepTitleOfNextModal(refusal);
			}
			chooseMenuItem(menu, "Delete Trip");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(refusal, QStringLiteral("Trip Still Saving"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QCOMPARE(view(panel)->model()->rowCount(), 1);
	}

	// Nor can it be opened, from the table or from the map: its track would
	// be cut off. The trip already shown stays shown and highlighted.
	void openingATripStillSavingIsRefused_data() {
		QTest::addColumn<bool>("fromTheMap");
		QTest::newRow("row click") << false;
		QTest::newRow("map click") << true;
	}

	void openingATripStillSavingIsRefused() {
		QFETCH(bool, fromTheMap);
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();

		markSaving(sim, 2);
		QString refusal;
		keepTitleOfNextModal(refusal);
		if (fromTheMap)
			panel.selectTripById(2);
		else
			clickRow(view(panel), rowOfTrip(view(panel), 2));
		QCOMPARE(refusal, QStringLiteral("Trip Still Saving"));
		QTest::qWait(200);
		QCOMPARE(ready.count(), 1);
		QVERIFY(view(panel)->isEnabled());
		QCOMPARE(view(panel)->selectionModel()->selectedRows().value(0).row(), rowOfTrip(view(panel), 1));
	}

	// Nor exported: the KML would be cut off.
	void exportingATripStillSavingIsRefused() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		markSaving(sim, 1);
		QString refusal;
		onNextModal([&refusal](QWidget* menu) {
			keepTitleOfNextModal(refusal); // the save dialog, if not refused
			chooseMenuItem(menu, "Export to KML");
		});
		openRowMenu(view(panel), 0);
		QCOMPARE(refusal, QStringLiteral("Trip Still Saving"));
	}

	// Export to KML suggests "<departure>-<destination>_<departure time>.kml"
	// and writes the trip's track to the chosen file.
	void exportToKmlFromTheRowMenu() {
		FlightDriver sim;
		const int tripId = sim.startTrip(); // departs 2026-01-02 10:00:00.5Z
		sim.ticks(3);
		sim.endTrip();
		exec("UPDATE trips SET departure_icao='AAAA', destination_icao='BBBB'");
		TripHistoryPanel panel(sim.bridge());
		const QString path = QDir::current().absoluteFilePath("export.kml");
		QString suggested;
		onNextModal([&](QWidget* menu) {
			onNextModal([&](QWidget* dialog) { suggested = saveFileDialogAs(dialog, path); });
			chooseMenuItem(menu, "Export to KML");
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), tripId));
		QCOMPARE(suggested, QStringLiteral("AAAA-BBBB_20260102100000.kml"));
		QFile file(path);
		QVERIFY(waitFor([&file] { return file.size() > 0; }));
		QVERIFY(waitFor([&file] { return file.open(QIODevice::ReadOnly); }));
		const QString kml = QString::fromUtf8(file.readAll());
		QVERIFY2(kml.trimmed().endsWith("</kml>"), qPrintable(kml.right(200)));
		QCOMPARE(kml.count("<when>"), 4); // the samples sent while the trip was recording
	}

	// An export that can't write its file says so, naming it.
	void failedExportFromTheRowMenuShowsAnError() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.endTrip();
		TripHistoryPanel panel(sim.bridge());
		const QString path = QDir::current().absoluteFilePath("readonly.kml");
		QFile existing(path);
		QVERIFY(existing.open(QIODevice::WriteOnly));
		existing.close();
		QVERIFY(existing.setPermissions(QFileDevice::ReadOwner));
		onNextModal([&](QWidget* menu) {
			onNextModal([&](QWidget* dialog) {
				onNextModal([](QWidget* replace) { clickDialogButton(replace, "Yes"); });
				saveFileDialogAs(dialog, path);
			});
			chooseMenuItem(menu, "Export to KML");
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), tripId));
		QString error;
		onNextModal([&error](QWidget* box) {
			error = static_cast<QMessageBox*>(box)->text();
			clickDialogButton(box, "OK");
		});
		const bool shown = waitFor([&error] { return !error.isEmpty(); });
		existing.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner);
		QVERIFY(shown);
		QVERIFY2(error.startsWith(QStringLiteral("Failed to export KML to %1.").arg(path)), qPrintable(error));
	}

	// A trip with no sample to read has no track: the export says so and
	// writes no file.
	void exportingATripWithNoSamplesSaysSo() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.endTrip();
		exec(qPrintable(QStringLiteral("DELETE FROM trip_data WHERE trip = %1").arg(tripId)));
		TripHistoryPanel panel(sim.bridge());
		const QString path = QDir::current().absoluteFilePath("empty.kml");
		onNextModal([&](QWidget* menu) {
			onNextModal([&](QWidget* dialog) { saveFileDialogAs(dialog, path); });
			chooseMenuItem(menu, "Export to KML");
		});
		openRowMenu(view(panel), rowOfTrip(view(panel), tripId));
		QString error;
		onNextModal([&error](QWidget* box) {
			error = static_cast<QMessageBox*>(box)->text();
			clickDialogButton(box, "OK");
		});
		QVERIFY(waitFor([&error] { return !error.isEmpty(); }));
		QCOMPARE(error, QStringLiteral("Failed to export KML to %1.\nNo recorded samples of this trip could be read.").arg(path));
		QVERIFY(!QFile::exists(path));
	}

	// Pointing at a row tints it (a completed trip only has a color while
	// hovered); leaving the table drops the tint.
	void hoveringARowTintsIt() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		panel.show();
		QTableView* v = view(panel);
		QWidget* viewport = v->viewport();
		auto moveTo = [viewport](const QPoint& pos) {
			QMouseEvent event(QEvent::MouseMove, QPointF(pos), viewport->mapToGlobal(QPointF(pos)), Qt::NoButton, Qt::NoButton, Qt::NoModifier);
			QCoreApplication::sendEvent(viewport, &event);
		};
		auto tint = [v](int row) { return v->model()->index(row, 0).data(Qt::BackgroundRole); };
		moveTo(v->visualRect(v->model()->index(1, 0)).center());
		QCOMPARE(tint(1).value<QBrush>().color(), QColor(220, 230, 245));
		QVERIFY(!tint(0).isValid());
		moveTo(v->visualRect(v->model()->index(0, 0)).center());
		QCOMPARE(tint(0).value<QBrush>().color(), QColor(220, 230, 245));
		QVERIFY(!tint(1).isValid());
		QEvent leave(QEvent::Leave);
		QCoreApplication::sendEvent(viewport, &leave);
		QVERIFY(!tint(0).isValid());
	}

	// A right-click only opens the row menu: the loaded trip stays selected.
	void rightClickingAnotherRowKeepsTheSelection() {
		FlightDriver sim;
		addTrip(1, 0, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		panel.show();
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();
		QTableView* v = view(panel);
		const QPoint other = v->visualRect(v->model()->index(rowOfTrip(v, 2), 0)).center();
		QMouseEvent press(QEvent::MouseButtonPress, QPointF(other), v->viewport()->mapToGlobal(QPointF(other)), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
		QCoreApplication::sendEvent(v->viewport(), &press);
		QCOMPARE(v->selectionModel()->selectedRows().size(), 1);
		QCOMPARE(v->selectionModel()->selectedRows().value(0).row(), rowOfTrip(v, 1));
	}

	// Deleting the group the table is filtered by, from Manage Groups opened
	// on the panel, puts the filter back on "All Trips" and drops the
	// selection.
	void deletingTheFilteredGroupShowsAllTrips() {
		FlightDriver sim;
		const int training = addGroup("Training");
		addTrip(1, training, kDep, kArr);
		addTrip(2, 0, kDep, kArr);
		TripHistoryPanel panel(sim.bridge());
		QComboBox* combo = groupCombo(panel);
		combo->setCurrentIndex(combo->findText("Training"));
		QSignalSpy ready(&panel, &TripHistoryPanel::tripDatasetReady);
		panel.selectTripById(1);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		panel.setLoadingFinished();
		onNextModal([](QWidget* dialog) {
			dialog->findChild<QListWidget*>()->setCurrentRow(0);
			onNextModal([](QWidget* confirm) { clickDialogButton(confirm, "Yes"); });
			clickDialogButton(dialog, "Delete");
			static_cast<QDialog*>(dialog)->reject();
		});
		clickDialogButton(&panel, "Manage Groups…");
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_groups").toInt(), 0);
		QCOMPARE(combo->currentText(), QStringLiteral("All Trips"));
		QCOMPARE(combo->count(), 2);
		QCOMPARE(view(panel)->model()->rowCount(), 2);
		QVERIFY(view(panel)->selectionModel()->selectedRows().isEmpty());
	}

	// The mouse-over state is dropped before a cell is painted, so a hovered
	// row keeps its own status color in every cell instead of the style's
	// per-cell hover highlight. Painted with the Windows 11 style the app uses
	// on Windows, which draws that highlight.
	void hoveredCellsPaintLikeUnhoveredOnes() {
		std::unique_ptr<QStyle> style(QStyleFactory::create(QStringLiteral("windows11")));
		QVERIFY(style);
		FlightDriver sim;
		TripHistoryPanel panel(sim.bridge());
		sim.startTrip(); // a live row, painted green
		QTableView* v = view(panel);
		v->setStyle(style.get());
		auto paintCell = [v](QStyle::State extra) {
			QImage image(120, 24, QImage::Format_ARGB32);
			image.fill(Qt::white);
			QPainter painter(&image);
			QStyleOptionViewItem option;
			option.rect = image.rect();
			option.state = QStyle::State_Enabled | extra;
			option.palette = v->palette();
			option.widget = v;
			v->itemDelegate()->paint(&painter, option, v->model()->index(0, TripHistoryModel::TitleColumn));
			return image;
		};
		QVERIFY(paintCell(QStyle::State_MouseOver) == paintCell(QStyle::State_None));
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

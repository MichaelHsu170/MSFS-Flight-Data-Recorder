// Data Table panel (data_table_panel.cpp): field rows, value formatting,
// which point is shown, and the persisted field-visibility filter.
#include "test_support.h"

#include "app_settings.h"
#include "data_table_panel.h"
#include "trip_data_fields.h"

#include <QCheckBox>
#include <QClipboard>
#include <QGuiApplication>
#include <QHeaderView>
#include <QTableWidget>
#include <QtTest>

using namespace TestSupport;

namespace {

int numIndex(const char* column) {
	int i = 0, found = -1;
#define FIND(dbColumn, memberExpr, sqlType) if (QLatin1String(#dbColumn) == QLatin1String(column)) found = i; ++i;
	TRIP_DATA_NUM_FIELDS(FIND)
#undef FIND
	return found;
}

TripSamplePoint makePoint(double base, const char* zulu) {
	TripSamplePoint p;
	p.zuluTime = QString::fromLatin1(zulu);
	p.localTime = QStringLiteral("local ") + QString::fromLatin1(zulu);
	int n = 0;
#define COUNT(dbColumn, memberExpr, sqlType) ++n;
	TRIP_DATA_NUM_FIELDS(COUNT)
#undef COUNT
	for (int i = 0; i < n; ++i)
		p.rawNums.push_back(base + i);
	p.rawNums[numIndex("gps_position_lat")] = 43.5;
	p.rawNums[numIndex("gps_position_lon")] = -1.25;
	p.boolGroup1 = 0x1;        // autopilot_airspeed_hold only
	p.boolGroup2 = 1u << 31;   // general_eng_master_alternator only
	p.boolGroup3 = 1u << 31;   // kohlsman_setting_std only
	return p;
}

}

class TstDataTablePanel : public QObject {
	Q_OBJECT

private:
	static QTableWidget* table(DataTablePanel& panel) { return panel.findChild<QTableWidget*>(); }

	static int row(QTableWidget* t, const QString& label) {
		for (int r = 0; r < t->rowCount(); ++r)
			if (t->item(r, 0)->text() == label)
				return r;
		return -1;
	}

	static QString value(QTableWidget* t, const QString& label) {
		const int r = row(t, label);
		return r < 0 ? QStringLiteral("<no row %1>").arg(label) : t->item(r, 1)->text();
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() { removeSettings(); }

	void rowsCoverEveryField() {
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		// Zulu, Local, combined GPS row, the other 134 numeric fields, speed
		// and load of 4 engines, 96 bools.
		QCOMPARE(t->rowCount(), 3 + 134 + 8 + 96);
		QCOMPARE(t->item(3 + 134, 0)->text(), QStringLiteral("Engine Speed 1"));
		QCOMPARE(t->item(3 + 134 + 7, 0)->text(), QStringLiteral("Engine Load 4"));
		QCOMPARE(t->item(0, 0)->text(), QStringLiteral("Time (Zulu)"));
		QCOMPARE(t->item(1, 0)->text(), QStringLiteral("Time (Local)"));
		QCOMPARE(t->item(2, 0)->text(), QStringLiteral("GPS Position"));
		QCOMPARE(t->item(3, 0)->text(), QStringLiteral("Eng Exhaust Gas Temperature 1"));
		QCOMPARE(t->item(t->rowCount() - 1, 0)->text(), QStringLiteral("Kohlsman Setting Std"));
		QCOMPARE(row(t, "Gps Position Lat"), -1);
	}

	void startsEmpty() {
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		for (int r = 0; r < t->rowCount(); ++r)
			QCOMPARE(t->item(r, 1)->text(), QString());
	}

	void datasetShowsItsLastPoint() {
		DataTablePanel panel;
		TripDataset d;
		d.points = { makePoint(100, "first"), makePoint(200, "last") };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		QCOMPARE(value(t, "Time (Zulu)"), QStringLiteral("last"));
		QCOMPARE(value(t, "Time (Local)"), QStringLiteral("local last"));
		QCOMPARE(value(t, "Eng Exhaust Gas Temperature 1"), QStringLiteral("200"));
		QCOMPARE(t->item(row(t, "Time (Zulu)"), 1)->toolTip(), QStringLiteral("last"));
	}

	void valuesAreFormatted() {
		DataTablePanel panel;
		TripDataset d;
		TripSamplePoint p = makePoint(0, "t");
		p.rawNums[numIndex("g_force")] = 1.23456789;
		p.rawNums[numIndex("plane_altitude")] = 1234567.0;
		d.points = { p };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		QCOMPARE(value(t, "G Force"), QStringLiteral("1.23457"));
		QCOMPARE(value(t, "Plane Altitude"), QStringLiteral("1.23457e+06"));
		QCOMPARE(value(t, "GPS Position"), QString::fromUtf8("43°30'00.0\"N 1°15'00.0\"W"));
		QCOMPARE(value(t, "Autopilot Airspeed Hold"), QStringLiteral("Yes"));
		QCOMPARE(value(t, "Autopilot Master"), QStringLiteral("No"));
		QCOMPARE(value(t, "General Eng Master Alternator"), QStringLiteral("Yes"));
		QCOMPARE(value(t, "Flap Damage By Speed"), QStringLiteral("No"));
		QCOMPARE(value(t, "Kohlsman Setting Std"), QStringLiteral("Yes"));
		QCOMPARE(value(t, "Sim On Ground"), QStringLiteral("No"));
	}

	void engineRowsShowTheRecordedEnginesByType() {
		DataTablePanel panel;
		TripDataset d;
		TripSamplePoint p = makePoint(0, "t");
		p.engine = { 5, 2, { 1700, 1712.6f }, { 81.24f, 80 } };
		d.points = { p };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		QCOMPARE(value(t, "Engine Speed 1"), QStringLiteral("Prop RPM: 1700 rpm"));
		QCOMPARE(value(t, "Engine Speed 2"), QStringLiteral("Prop RPM: 1713 rpm"));
		QCOMPARE(value(t, "Engine Load 1"), QStringLiteral("Torque: 81.2 %"));
		QCOMPARE(value(t, "Engine Load 2"), QStringLiteral("Torque: 80.0 %"));
		QCOMPARE(value(t, "Engine Speed 3"), QString());
		QCOMPARE(value(t, "Engine Load 4"), QString());
		// The rows after them stay aligned with their values.
		QCOMPARE(value(t, "Autopilot Airspeed Hold"), QStringLiteral("Yes"));

		// An engine type that isn't recorded shows nothing.
		p.engine = { 2, 2, { 1700, 1700 }, { 80, 80 } };
		d.points = { p };
		panel.setDataset(&d);
		QCOMPARE(value(t, "Engine Speed 1"), QString());
		QCOMPARE(value(t, "Engine Load 1"), QString());
	}

	void dmsCarriesInsteadOfShowingSixtySeconds() {
		DataTablePanel panel;
		TripDataset d;
		TripSamplePoint p = makePoint(0, "t");
		p.rawNums[numIndex("gps_position_lat")] = 10.99999999;
		p.rawNums[numIndex("gps_position_lon")] = 0;
		d.points = { p };
		panel.setDataset(&d);
		QCOMPARE(value(table(panel), "GPS Position"), QString::fromUtf8("11°00'00.0\"N 0°00'00.0\"E"));
	}

	void cursorPicksAPointAndClearingFallsBackToLast() {
		DataTablePanel panel;
		TripDataset d;
		d.points = { makePoint(1, "p0"), makePoint(2, "p1"), makePoint(3, "p2") };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		panel.setCursorIndex(0);
		QCOMPARE(value(t, "Time (Zulu)"), QStringLiteral("p0"));
		panel.setCursorIndex(99);
		QCOMPARE(value(t, "Time (Zulu)"), QStringLiteral("p2"));
		panel.setCursorIndex(1);
		panel.setCursorIndex(-1);
		QCOMPARE(value(t, "Time (Zulu)"), QStringLiteral("p2"));
	}

	void clearingTheDatasetEmptiesTheTable() {
		DataTablePanel panel;
		TripDataset d;
		d.points = { makePoint(1, "p0") };
		panel.setDataset(&d);
		panel.setDataset(nullptr);
		QCOMPARE(value(table(panel), "Time (Zulu)"), QString());
		TripDataset empty;
		panel.setDataset(&empty);
		QCOMPARE(value(table(panel), "Time (Zulu)"), QString());
	}

	void hiddenFieldsFromSettingsAreHidden() {
		AppSettings::instance().setDataTableHiddenFields({ "G Force", "Time (Local)" });
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		QVERIFY(t->isRowHidden(row(t, "G Force")));
		QVERIFY(t->isRowHidden(row(t, "Time (Local)")));
		QVERIFY(!t->isRowHidden(row(t, "Time (Zulu)")));
	}

	void fieldsDialogHidesAndPersists() {
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		onNextModal([](QWidget* dialog) {
			for (QCheckBox* box : dialog->findChildren<QCheckBox*>())
				if (box->text() == QLatin1String("G Force"))
					box->setChecked(false);
			clickDialogButton(dialog, "OK");
		});
		emit t->horizontalHeader()->sectionClicked(0);
		QCOMPARE(AppSettings::instance().dataTableHiddenFields(), QStringList{ "G Force" });
		QVERIFY(t->isRowHidden(row(t, "G Force")));
	}

	void fieldsDialogCancelChangesNothing() {
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		onNextModal([](QWidget* dialog) {
			for (QCheckBox* box : dialog->findChildren<QCheckBox*>())
				box->setChecked(false);
			clickDialogButton(dialog, "Cancel");
		});
		emit t->horizontalHeader()->sectionClicked(0);
		QCOMPARE(AppSettings::instance().dataTableHiddenFields(), QStringList());
		QVERIFY(!t->isRowHidden(row(t, "G Force")));
	}

	// Only a non-empty value cell offers Copy. A menu opened anywhere else
	// would take the one answer below, and the clipboard would differ.
	void rightClickOnAValueCopiesIt() {
		DataTablePanel panel;
		panel.resize(400, 600);
		panel.show();
		QVERIFY(QTest::qWaitForWindowExposed(&panel));
		QTableWidget* t = table(panel);
		QGuiApplication::clipboard()->setText("untouched");
		int menus = 0;
		onNextModal([&menus](QWidget* menu) {
			++menus;
			chooseMenuItem(menu, "Copy");
		});
		const QPoint label = t->visualItemRect(t->item(0, 0)).center();
		const QPoint value = t->visualItemRect(t->item(0, 1)).center();
		emit t->customContextMenuRequested(label);
		emit t->customContextMenuRequested(value); // still empty
		emit t->customContextMenuRequested(QPoint(-5, -5)); // no cell
		QCOMPARE(menus, 0);
		TripDataset d;
		d.points = { makePoint(1, "2026-01-02T10:00:00Z") };
		panel.setDataset(&d);
		emit t->customContextMenuRequested(t->visualItemRect(t->item(0, 1)).center());
		QCOMPARE(menus, 1);
		QCOMPARE(QGuiApplication::clipboard()->text(), QStringLiteral("2026-01-02T10:00:00Z"));
	}

	void fieldColumnWidthIsPersisted() {
		{
			DataTablePanel panel;
			table(panel)->setColumnWidth(0, 210);
		}
		QCOMPARE(AppSettings::instance().dataTableFieldColumnWidth(), 210);
		DataTablePanel again;
		QCOMPARE(table(again)->columnWidth(0), 210);
	}
};

QTEST_MAIN(TstDataTablePanel)
#include "tst_data_table_panel.moc"

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

#include <cmath>

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
	p.boolGroups[1] = 0x1;        // autopilot_airspeed_hold only
	p.boolGroups[2] = 1u << 31;   // general_eng_master_alternator only
	p.boolGroups[3] = 1u << 31;   // kohlsman_setting_std only
	return p;
}

// p with engines 1..engines, every engine value not recorded (NaN).
TripSamplePoint withEngines(TripSamplePoint p, int engines) {
	p.engineValues.assign((size_t)engines * TRIP_ENGINE_FIELD_COUNT, std::nan(""));
	return p;
}

void setEngineValue(TripSamplePoint& p, int engine, TripEngineField field, double value) {
	p.engineValues[(size_t)(engine - 1) * TRIP_ENGINE_FIELD_COUNT + field] = value;
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
	void cleanup() { cancelPendingModals(); } // no dialog action outlives its test

	void rowsCoverEveryField() {
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		// Zulu, Local, combined GPS row, the other 108 numeric fields, 70
		// bools; no engine rows before a trip with engines loads.
		QCOMPARE(t->rowCount(), 3 + 108 + 70);
		QCOMPARE(t->item(0, 0)->text(), QStringLiteral("Time (Zulu)"));
		QCOMPARE(t->item(1, 0)->text(), QStringLiteral("Time (Local)"));
		QCOMPARE(t->item(2, 0)->text(), QStringLiteral("GPS Position"));
		QCOMPARE(t->item(3, 0)->text(), QStringLiteral("Ambient Temperature"));
		QCOMPARE(t->item(t->rowCount() - 1, 0)->text(), QStringLiteral("Kohlsman Setting Std"));
		QCOMPARE(row(t, "Gps Position Lat"), -1);

		// Then the 32 engine values, each for engines 1..2.
		TripDataset d;
		d.points = { withEngines(makePoint(0, "t"), 2) };
		panel.setDataset(&d);
		QCOMPARE(t->rowCount(), 3 + 108 + 70 + 32 * 2);
		QCOMPARE(t->item(3 + 108 + 70, 0)->text(), QStringLiteral("Turb Eng N1 1"));
		QCOMPARE(t->item(3 + 108 + 70 + 1, 0)->text(), QStringLiteral("Turb Eng N1 2"));
		QCOMPARE(t->item(3 + 108 + 70 + 2, 0)->text(), QStringLiteral("Turb Eng N2 1"));
		QCOMPARE(t->item(t->rowCount() - 1, 0)->text(), QStringLiteral("Turb Eng Is Igniting 2"));
		QCOMPARE(t->item(3 + 108 + 70 - 1, 0)->text(), QStringLiteral("Kohlsman Setting Std"));
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
		QCOMPARE(value(t, "Ambient Temperature"), QStringLiteral("200"));
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

	// One row per engine value of each engine the trip has: a number, Yes/No
	// for an on/off value, blank for what wasn't recorded.
	void engineRowsShowEachEnginesValues() {
		DataTablePanel panel;
		TripDataset d;
		TripSamplePoint twin = withEngines(makePoint(0, "twin"), 2);
		setEngineValue(twin, 1, TRIP_ENGINE_turb_eng_n1, 85.5);
		setEngineValue(twin, 2, TRIP_ENGINE_turb_eng_n1, 1234567.0);
		setEngineValue(twin, 1, TRIP_ENGINE_eng_combustion, 1);
		setEngineValue(twin, 2, TRIP_ENGINE_eng_combustion, 0);
		TripSamplePoint triple = withEngines(makePoint(0, "triple"), 3);
		setEngineValue(triple, 3, TRIP_ENGINE_prop_rpm, 2102);
		d.points = { twin, triple };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		// Rows for the trip's most engines, its last point shown.
		QCOMPARE(value(t, "Prop Rpm 3"), QStringLiteral("2102"));
		QCOMPARE(value(t, "Eng Oil Pressure 3"), QString());
		QCOMPARE(value(t, "Turb Eng N1 1"), QString());
		QCOMPARE(value(t, "Eng Combustion 1"), QString());

		panel.setCursorIndex(0);
		QCOMPARE(value(t, "Turb Eng N1 1"), QStringLiteral("85.5"));
		QCOMPARE(value(t, "Turb Eng N1 2"), QStringLiteral("1.23457e+06"));
		QCOMPARE(value(t, "Eng Combustion 1"), QStringLiteral("Yes"));
		QCOMPARE(value(t, "Eng Combustion 2"), QStringLiteral("No"));
		// An engine past the point's own engines is blank.
		QCOMPARE(value(t, "Prop Rpm 3"), QString());
		QCOMPARE(value(t, "Eng Combustion 3"), QString());
		// The rows before them stay aligned with their values.
		QCOMPARE(value(t, "Kohlsman Setting Std"), QStringLiteral("Yes"));

		// A single's trip has no rows for engine 2.
		TripDataset single;
		single.points = { withEngines(makePoint(0, "single"), 1) };
		panel.setDataset(&single);
		QCOMPARE(row(t, "Turb Eng N1 2"), -1);
		QCOMPARE(row(t, "Turb Eng Is Igniting 1"), t->rowCount() - 1);
		QCOMPARE(value(t, "Time (Zulu)"), QStringLiteral("single"));
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
		AppSettings::instance().setDataTableHiddenFields({ "G Force", "Time (Local)", "Eng Oil Pressure 3" });
		DataTablePanel panel;
		QTableWidget* t = table(panel);
		QVERIFY(t->isRowHidden(row(t, "G Force")));
		QVERIFY(t->isRowHidden(row(t, "Time (Local)")));
		QVERIFY(!t->isRowHidden(row(t, "Time (Zulu)")));
		// An engine row is hidden once a trip with that engine loads.
		TripDataset d;
		d.points = { withEngines(makePoint(0, "t"), 3) };
		panel.setDataset(&d);
		QVERIFY(t->isRowHidden(row(t, "Eng Oil Pressure 3")));
		QVERIFY(!t->isRowHidden(row(t, "Eng Oil Pressure 2")));
		QVERIFY(t->isRowHidden(row(t, "G Force")));
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

	// A hidden engine row the loaded trip doesn't have (no engine 3) isn't in
	// the dialog, and stays hidden.
	void fieldsDialogKeepsHiddenRowsTheTripLacks() {
		AppSettings::instance().setDataTableHiddenFields({ "Eng Oil Pressure 3" });
		DataTablePanel panel;
		TripDataset d;
		d.points = { withEngines(makePoint(0, "t"), 2) };
		panel.setDataset(&d);
		QTableWidget* t = table(panel);
		bool listed = false;
		onNextModal([&listed](QWidget* dialog) {
			for (QCheckBox* box : dialog->findChildren<QCheckBox*>()) {
				listed |= box->text() == QLatin1String("Eng Oil Pressure 3");
				if (box->text() == QLatin1String("Eng Oil Pressure 2"))
					box->setChecked(false);
			}
			clickDialogButton(dialog, "OK");
		});
		emit t->horizontalHeader()->sectionClicked(0);
		QVERIFY(!listed);
		QCOMPARE(AppSettings::instance().dataTableHiddenFields(), (QStringList{ "Eng Oil Pressure 3", "Eng Oil Pressure 2" }));
		QVERIFY(t->isRowHidden(row(t, "Eng Oil Pressure 2")));
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
		const ClipboardGuard keepUsersClipboard;
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

// Live Status panel (live_status_panel.cpp): indicators, the recording
// toggle, the message history (events, retractions, cap) and the snapshot line.
#include "test_support.h"

#include "live_status_panel.h"
#include "version.h"

#include <QLabel>
#include <QListWidget>
#include <QMouseEvent>
#include <QRegularExpression>
#include <QToolTip>
#include <QtTest>

using namespace TestSupport;

class TstLiveStatusPanel : public QObject {
	Q_OBJECT

private:
	static QListWidget* history(LiveStatusPanel& p) { return p.findChild<QListWidget*>(); }
	static QWidget* recordingToggle(LiveStatusPanel& p) { return p.findChild<QWidget*>("recordingToggle"); }
	static QLabel* labelWithText(LiveStatusPanel& p, const QString& prefix) {
		for (QLabel* l : p.findChildren<QLabel*>())
			if (l->text().startsWith(prefix))
				return l;
		return nullptr;
	}
	static QLabel* connectionDot(LiveStatusPanel& p) {
		// The dot right after the "Connection:" label in the title row.
		for (QLabel* l : p.findChildren<QLabel*>())
			if (!l->toolTip().isEmpty() && l->parentWidget() == &p)
				return l;
		return nullptr;
	}
	static QStringList lines(LiveStatusPanel& p) {
		QStringList out;
		for (int i = 0; i < history(p)->count(); ++i)
			out << history(p)->item(i)->text().mid(22); // strip "[yyyy-MM-dd hh:mm:ss] "
		return out;
	}
	static void releaseOn(QWidget* w, const QPoint& pos) {
		QMouseEvent release(QEvent::MouseButtonRelease, QPointF(pos), w->mapToGlobal(QPointF(pos)), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
		QCoreApplication::sendEvent(w, &release);
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() {
		removeDatabase();
		removeSettings();
	}

	void showsTheAppVersion() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QVERIFY(labelWithText(panel, QStringLiteral("ver. " APP_VERSION)) != nullptr);
	}

	void connectionIndicatorFollowsTheBridge() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QLabel* dot = connectionDot(panel);
		QVERIFY(dot);
		QCOMPARE(dot->toolTip(), QString::fromUtf8("Waiting for simulator…"));
		sim.send(recvPacket(SIMCONNECT_RECV_ID_OPEN, sizeof(SIMCONNECT_RECV)));
		QCOMPARE(dot->toolTip(), QStringLiteral("Connected to Microsoft Flight Simulator"));
		sim.send(recvPacket(SIMCONNECT_RECV_ID_QUIT, sizeof(SIMCONNECT_RECV)));
		QCOMPARE(dot->toolTip(), QString::fromUtf8("Waiting for simulator…"));
	}

	void logMessagesAreTimestampedAndTrimmed() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		emit sim.bridge().logMessage("  hello  ");
		emit sim.bridge().logMessage("   ");
		QCOMPARE(history(panel)->count(), 1);
		const QString text = history(panel)->item(0)->text();
		QVERIFY(QRegularExpression("^\\[\\d{4}-\\d{2}-\\d{2} \\d{2}:\\d{2}:\\d{2}\\] hello$").match(text).hasMatch());
	}

	void recordingMessagesAppear() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		sim.startTrip();
		sim.endTrip();
		QVERIFY(waitFor([&panel] { return lines(panel).contains("Recording stopped"); }));
		QVERIFY(lines(panel).contains("Recording started"));
	}

	void historyIsCappedAt500() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		for (int i = 0; i < 510; ++i)
			emit sim.bridge().logMessage(QString::number(i));
		QCOMPARE(history(panel)->count(), 500);
		QCOMPARE(lines(panel).first(), QStringLiteral("10"));
		QCOMPARE(lines(panel).last(), QStringLiteral("509"));
	}

	void recordingIndicatorStates() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		QVERIFY(toggle);
		QCOMPARE(toggle->toolTip(), QStringLiteral("Click to disable automatic recording."));
		const int tripId = sim.startTrip();
		QCOMPARE(toggle->toolTip(), QStringLiteral("Recording trip #%1 (can't be toggled while a trip is recording)").arg(tripId));
		sim.endTrip();
		QVERIFY(waitFor([toggle] { return toggle->toolTip() == QStringLiteral("Click to disable automatic recording."); }));
	}

	void clickingTheToggleFlipsAutomaticRecording() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		toggle->resize(100, 20);
		releaseOn(toggle, QPoint(5, 5));
		QVERIFY(!sim.bridge().isRecordingEnabled());
		QCOMPARE(toggle->toolTip(), QStringLiteral("Click to enable automatic recording."));
		releaseOn(toggle, QPoint(500, 500)); // dragged off before releasing: no toggle
		QVERIFY(!sim.bridge().isRecordingEnabled());
		releaseOn(toggle, QPoint(5, 5));
		QVERIFY(sim.bridge().isRecordingEnabled());
	}

	void toggleDoesNothingWhileRecording() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		toggle->resize(100, 20);
		sim.startTrip();
		releaseOn(toggle, QPoint(5, 5));
		QVERIFY(sim.bridge().isRecordingEnabled());
	}

	void committedEventsShowAndRetractionsRemoveThem() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		const int tripId = sim.startTrip();
		emit sim.bridge().eventCommitted(tripId, 41, "Event: GEAR_UP");
		emit sim.bridge().eventCommitted(tripId, 42, "Event: FLAPS_UP");
		emit sim.bridge().eventCommitted(tripId + 100, 43, "Event: STALE_TRIP");
		QVERIFY(lines(panel).contains("Event: GEAR_UP"));
		QVERIFY(lines(panel).contains("Event: FLAPS_UP"));
		QVERIFY(!lines(panel).contains("Event: STALE_TRIP"));
		emit sim.bridge().eventsRetracted({ 41, 999 });
		QVERIFY(!lines(panel).contains("Event: GEAR_UP"));
		QVERIFY(lines(panel).contains("Event: FLAPS_UP"));
	}

	void blankEventTextAddsNoLine() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		const int tripId = sim.startTrip();
		const int before = history(panel)->count();
		emit sim.bridge().eventCommitted(tripId, 5, "   ");
		QCOMPARE(history(panel)->count(), before);
	}

	// Seq 1's line scrolls out of the capped history; retracting it later
	// must not remove (or touch) anything still shown.
	void retractingAnEventPrunedByTheCapRemovesNothing() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		const int tripId = sim.startTrip();
		emit sim.bridge().eventCommitted(tripId, 1, "Event: OLDEST");
		for (int i = 0; i < 300; ++i)
			emit sim.bridge().logMessage(QString::number(i));
		emit sim.bridge().eventCommitted(tripId, 2, "Event: KEPT");
		for (int i = 300; i < 600; ++i)
			emit sim.bridge().logMessage(QString::number(i));
		QVERIFY(!lines(panel).contains("Event: OLDEST"));
		QVERIFY(lines(panel).contains("Event: KEPT"));
		const QStringList shown = lines(panel);
		emit sim.bridge().eventsRetracted({ 1 });
		QCOMPARE(lines(panel), shown);
		emit sim.bridge().eventsRetracted({ 2 });
		QVERIFY(!lines(panel).contains("Event: KEPT"));
		QCOMPARE(history(panel)->count(), 499);
	}

	void hoveringTheToggleShowsItsTooltipAtOnce() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		panel.show();
		QVERIFY(QTest::qWaitForWindowExposed(&panel));
		QWidget* toggle = recordingToggle(panel);
		QEnterEvent enter(QPointF(5, 5), toggle->mapToGlobal(QPointF(5, 5)), toggle->mapToGlobal(QPointF(5, 5)));
		QCoreApplication::sendEvent(toggle, &enter);
		QVERIFY(QToolTip::isVisible());
		QCOMPARE(QToolTip::text(), QStringLiteral("Click to disable automatic recording."));
		QEvent leave(QEvent::Leave);
		QCoreApplication::sendEvent(toggle, &leave);
		QVERIFY(waitFor([] { return !QToolTip::isVisible(); }, 2000));
	}

	void staleTripEndedIsIgnored() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		const int tripId = sim.startTrip();
		const QString recordingTip = recordingToggle(panel)->toolTip();
		emit sim.bridge().tripEnded(tripId + 1);
		QCOMPARE(recordingToggle(panel)->toolTip(), recordingTip);
	}

	void snapshotShowsTheLatestSample() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QVERIFY(labelWithText(panel, "Alt - ft | Hdg -° | Spd - kt | V/S - ft/min"));
		sim.record.plane_altitude = 1234;
		sim.record.airspeed_indicated = 140;
		sim.record.vertical_speed = -700;
		sim.startTrip();
		QVERIFY(labelWithText(panel, QString::fromUtf8("Alt 1234 ft | Hdg 90° | Spd 140 kt | V/S -700 ft/min")));
	}
};

QTEST_MAIN(TstLiveStatusPanel)
#include "tst_live_status_panel.moc"

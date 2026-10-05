// Live Status panel (live_status_panel.cpp): indicators, the recording
// toggle, the message history (events, retractions, cap, scrolling) and the snapshot line.
#include "test_support.h"

#include "live_status_panel.h"
#include "version.h"

#include <QLabel>
#include <QListWidget>
#include <QEnterEvent>
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
	static QLabel* recordingDot(LiveStatusPanel& p) {
		for (QLabel* l : recordingToggle(p)->findChildren<QLabel*>())
			if (!l->pixmap().isNull())
				return l;
		return nullptr;
	}
	// The color a dot is painted in, read from its middle.
	static QColor dotColor(const QLabel* dot) {
		const QImage image = dot->pixmap().toImage();
		return image.pixelColor(image.width() / 2, image.height() / 2);
	}
	static QStringList lines(LiveStatusPanel& p) {
		QStringList out;
		for (int i = 0; i < history(p)->count(); ++i)
			out << history(p)->item(i)->text().mid(22); // strip "[yyyy-MM-dd hh:mm:ss] "
		return out;
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
		QCOMPARE(dotColor(dot), QColor(0xd9, 0x3a, 0x3a)); // red
		sim.send(recvPacket(SIMCONNECT_RECV_ID_OPEN, sizeof(SIMCONNECT_RECV)));
		QCOMPARE(dot->toolTip(), QStringLiteral("Connected to Microsoft Flight Simulator"));
		QCOMPARE(dotColor(dot), QColor(0x2e, 0xa8, 0x4f)); // green
		sim.send(recvPacket(SIMCONNECT_RECV_ID_QUIT, sizeof(SIMCONNECT_RECV)));
		QCOMPARE(dot->toolTip(), QString::fromUtf8("Waiting for simulator…"));
		QCOMPARE(dotColor(dot), QColor(0xd9, 0x3a, 0x3a));
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

	// More lines than fit: the newest one is scrolled into view.
	void historyScrollsToTheNewestLine() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		panel.resize(400, 200);
		panel.show();
		QVERIFY(QTest::qWaitForWindowExposed(&panel));
		for (int i = 0; i < 50; ++i)
			emit sim.bridge().logMessage(QString::number(i));
		QListWidget* list = history(panel);
		const QRect shown = list->viewport()->rect();
		QVERIFY(!shown.intersects(list->visualItemRect(list->item(0))));
		QVERIFY(shown.contains(list->visualItemRect(list->item(list->count() - 1))));
	}

	// Red and clickable (a pointing hand) when it would record, green and not
	// clickable (an arrow) while recording.
	void recordingIndicatorStates() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		QVERIFY(toggle);
		QCOMPARE(toggle->toolTip(), QStringLiteral("Click to disable automatic recording."));
		QCOMPARE(dotColor(recordingDot(panel)), QColor(0xd9, 0x3a, 0x3a));
		QCOMPARE(toggle->cursor().shape(), Qt::PointingHandCursor);
		const int tripId = sim.startTrip();
		QCOMPARE(toggle->toolTip(), QStringLiteral("Recording trip #%1 (can't be toggled while a trip is recording)").arg(tripId));
		QCOMPARE(dotColor(recordingDot(panel)), QColor(0x2e, 0xa8, 0x4f));
		QCOMPARE(toggle->cursor().shape(), Qt::ArrowCursor);
		sim.endTrip();
		QVERIFY(waitFor([toggle] { return toggle->toolTip() == QStringLiteral("Click to disable automatic recording."); }));
		QCOMPARE(dotColor(recordingDot(panel)), QColor(0xd9, 0x3a, 0x3a));
		QCOMPARE(toggle->cursor().shape(), Qt::PointingHandCursor);
	}

	void clickingTheToggleFlipsAutomaticRecording() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		toggle->resize(100, 20);
		sendLeftButton(toggle, QEvent::MouseButtonRelease, QPoint(5, 5));
		QVERIFY(!sim.bridge().isRecordingEnabled());
		QCOMPARE(toggle->toolTip(), QStringLiteral("Click to enable automatic recording."));
		QCOMPARE(dotColor(recordingDot(panel)), QColor(0x9a, 0x9a, 0x9a)); // grey
		QCOMPARE(toggle->cursor().shape(), Qt::PointingHandCursor);
		sendLeftButton(toggle, QEvent::MouseButtonRelease, QPoint(500, 500)); // dragged off before releasing: no toggle
		QVERIFY(!sim.bridge().isRecordingEnabled());
		sendLeftButton(toggle, QEvent::MouseButtonRelease, QPoint(5, 5));
		QVERIFY(sim.bridge().isRecordingEnabled());
	}

	void toggleDoesNothingWhileRecording() {
		FlightDriver sim;
		LiveStatusPanel panel(sim.bridge());
		QWidget* toggle = recordingToggle(panel);
		toggle->resize(100, 20);
		sim.startTrip();
		sendLeftButton(toggle, QEvent::MouseButtonRelease, QPoint(5, 5));
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

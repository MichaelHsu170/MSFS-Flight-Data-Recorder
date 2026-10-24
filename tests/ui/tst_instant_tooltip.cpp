// Instant tooltip (instant_tooltip.h): the widget's tooltip shows the moment
// the pointer enters it and (by Qt itself) hides when it leaves, the Enter
// still reaches the widget's other filters, and the filter goes away with the
// widget.
#include "instant_tooltip.h"
#include "test_support.h"

#include <QEnterEvent>
#include <QLabel>
#include <QPointer>
#include <QtTest>

using namespace TestSupport;

namespace {

// Installed on the widget *before* showToolTipInstantly() runs, so in the
// eventFilter chain (most-recently-installed filter runs first) it sits right
// after the one under test: it only sees the Enter if that one passed it on.
class SpyFilter : public QObject {
public:
	bool received = false;
	bool eventFilter(QObject*, QEvent* event) override {
		if (event->type() == QEvent::Enter)
			received = true;
		return false;
	}
};

void sendEnter(QWidget* widget) {
	QEnterEvent enter(QPointF(5, 5), widget->mapToGlobal(QPointF(5, 5)), widget->mapToGlobal(QPointF(5, 5)));
	QCoreApplication::sendEvent(widget, &enter);
}

}

class TstInstantTooltip : public QObject {
	Q_OBJECT

private slots:
	void enterShowsTheTooltipAtOnceAndLeaveHidesIt() {
		QLabel label(QStringLiteral("x"));
		label.setToolTip(QStringLiteral("Tip text"));
		showToolTipInstantly(&label);
		// Away from the offscreen platform's fixed cursor: when the tooltip
		// closes, the widget under the cursor gets a fresh Enter, which would
		// show it again.
		label.move(300, 300);
		label.show();
		QVERIFY(QTest::qWaitForWindowExposed(&label));
		sendEnter(&label);
		QVERIFY(QToolTip::isVisible()); // no wake-up delay to wait out
		QCOMPARE(QToolTip::text(), QStringLiteral("Tip text"));
		QEvent leave(QEvent::Leave);
		QCoreApplication::sendEvent(&label, &leave);
		QVERIFY(waitFor([] { return !QToolTip::isVisible(); }, 2000));
	}

	void filterDoesNotConsumeTheEnter() {
		QLabel label(QStringLiteral("x"));
		label.setToolTip(QStringLiteral("Tip text"));
		SpyFilter spy;
		label.installEventFilter(&spy);
		showToolTipInstantly(&label);
		sendEnter(&label);
		QVERIFY(spy.received); // reached the filter installed "underneath" ours
	}

	void filterIsDestroyedWithTheWidget() {
		auto* label = new QLabel;
		const QList<QObject*> before = label->children();
		showToolTipInstantly(label);
		QList<QObject*> added = label->children();
		for (QObject* child : before)
			added.removeOne(child);
		QCOMPARE(added.size(), 1); // the filter, parented to the widget
		const QPointer<QObject> filter = added.first();
		delete label;
		QVERIFY(filter.isNull());
	}
};

QTEST_MAIN(TstInstantTooltip)
#include "tst_instant_tooltip.moc"

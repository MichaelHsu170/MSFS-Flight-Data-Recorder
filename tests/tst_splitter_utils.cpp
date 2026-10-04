// Splitter handle-release helper (splitter_utils.h): the callback fires once
// per release on the given handle and is unaffected by other event types.
#include "splitter_utils.h"

#include <QCoreApplication>
#include <QMouseEvent>
#include <QSplitter>
#include <QtTest>

#include <memory>

namespace {

// Installed on the handle *before* connectSplitterHandleReleased() runs, so
// in the eventFilter chain (most-recently-installed filter runs first) it
// sits right after the one under test: it only sees the release if that one
// passed it on. Checking the real sendEvent() return value doesn't work for
// this -- QSplitterHandle's own mouseReleaseEvent() accepts the event
// regardless, so the final return is true either way.
class SpyFilter : public QObject {
public:
	bool received = false;
	bool eventFilter(QObject*, QEvent* event) override {
		if (event->type() == QEvent::MouseButtonRelease)
			received = true;
		return false;
	}
};

}

class TstSplitterUtils : public QObject {
	Q_OBJECT

private slots:
	void releaseOnHandleFiresCallback() {
		QSplitter splitter(Qt::Horizontal);
		splitter.addWidget(new QWidget(&splitter));
		splitter.addWidget(new QWidget(&splitter));
		splitter.resize(200, 100);

		int callCount = 0;
		connectSplitterHandleReleased(&splitter, 1, [&callCount]() { ++callCount; });

		QWidget* handle = splitter.handle(1);
		QMouseEvent press(QEvent::MouseButtonPress, QPointF(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
		QCoreApplication::sendEvent(handle, &press);
		QCOMPARE(callCount, 0);  // press alone doesn't fire it

		QMouseEvent release(QEvent::MouseButtonRelease, QPointF(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
		QCoreApplication::sendEvent(handle, &release);
		QCOMPARE(callCount, 1);

		QCoreApplication::sendEvent(handle, &release);
		QCOMPARE(callCount, 2);  // fires again on a second release
	}

	void filterDoesNotConsumeTheEvent() {
		QSplitter splitter(Qt::Horizontal);
		splitter.addWidget(new QWidget(&splitter));
		splitter.addWidget(new QWidget(&splitter));

		SpyFilter spy;
		splitter.handle(1)->installEventFilter(&spy);
		connectSplitterHandleReleased(&splitter, 1, []() {});

		QMouseEvent release(QEvent::MouseButtonRelease, QPointF(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
		QCoreApplication::sendEvent(splitter.handle(1), &release);
		QVERIFY(spy.received);  // reached the filter installed "underneath" ours -- confirms ours passed the event on
	}

	void filterIsDestroyedWithTheHandle() {
		// Parented to the handle, so deleting the splitter frees the filter
		// and the callback it holds: the callback's captured state goes too.
		auto* splitter = new QSplitter(Qt::Horizontal);
		splitter->addWidget(new QWidget(splitter));
		splitter->addWidget(new QWidget(splitter));
		auto captured = std::make_shared<int>(0);
		const std::weak_ptr<int> watch = captured;
		connectSplitterHandleReleased(splitter, 1, [captured]() {});
		captured.reset();
		QVERIFY(!watch.expired()); // held by the filter
		delete splitter;
		QVERIFY(watch.expired());
	}
};

QTEST_MAIN(TstSplitterUtils)
#include "tst_splitter_utils.moc"

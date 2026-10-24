#pragma once

#include <QCursor>
#include <QEvent>
#include <QObject>
#include <QToolTip>
#include <QWidget>

// Shows widget's toolTip() the instant the pointer enters it, bypassing Qt's
// default ~700ms tooltip wake-up delay (QStyle::SH_ToolTip_WakeUpDelay). Qt
// itself hides a tooltip shown for a widget when the pointer leaves it.
// Doesn't consume the Enter, so the widget and other filters still see it.
// The filter object is parented to widget, so it is destroyed automatically
// along with it; the caller doesn't need to keep a pointer.
inline void showToolTipInstantly(QWidget* widget) {
	class InstantToolTipFilter : public QObject {
	public:
		using QObject::QObject;
	protected:
		bool eventFilter(QObject* obj, QEvent* event) override {
			if (event->type() == QEvent::Enter) {
				auto* w = static_cast<QWidget*>(obj);
				QToolTip::showText(QCursor::pos(), w->toolTip(), w);
			}
			return false;
		}
	};
	widget->installEventFilter(new InstantToolTipFilter(widget));
}

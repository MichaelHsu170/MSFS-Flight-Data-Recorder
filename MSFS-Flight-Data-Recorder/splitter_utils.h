#pragma once

#include <QEvent>
#include <QObject>
#include <QSplitter>

#include <functional>
#include <utility>

// Installs an event filter on one of splitter's handles so onRelease fires
// once when the user releases the mouse after dragging it, instead of once
// per pixel of movement the way QSplitter::splitterMoved does -- needed for
// handlers that persist to disk (e.g. rewriting settings.ini), where firing
// on every intermediate move causes visible stutter and disk churn
// proportional to drag distance.
// The filter object is parented to the handle, so it is destroyed
// automatically along with it; the caller doesn't need to keep a pointer.
inline void connectSplitterHandleReleased(QSplitter* splitter, int handleIndex, std::function<void()> onRelease) {
	class ReleaseFilter : public QObject {
	public:
		ReleaseFilter(QObject* parent, std::function<void()> cb) : QObject(parent), cb_(std::move(cb)) {}
	protected:
		bool eventFilter(QObject*, QEvent* event) override {
			if (event->type() == QEvent::MouseButtonRelease)
				cb_();
			return false;
		}
	private:
		std::function<void()> cb_;
	};
	QWidget* handle = splitter->handle(handleIndex);
	handle->installEventFilter(new ReleaseFilter(handle, std::move(onRelease)));
}

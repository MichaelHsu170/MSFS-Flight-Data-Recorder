#include "live_status_panel.h"
#include "recorder_bridge.h"
#include "types.h"
#include "logger.h"

#include <QDateTime>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QToolTip>
#include <QVBoxLayout>

namespace {

constexpr int kDotDiameter = 16;

// On/Off match Connection's two-state meaning; Disabled is only used by the
// Recording dot, for the user-facing "won't auto-start" state (see
// LiveStatusPanel::updateRecordingIndicator).
enum class DotState { Off, On, Disabled };

// Painted solid-color dot rather than a Unicode glyph (power/record symbols
// etc.): even "text presentation" code points fall back to Windows' color
// emoji font when the UI font lacks the glyph, and that font ignores
// QLabel's stylesheet/text color entirely -- painting the pixmap ourselves
// guarantees a strict result on every platform.
QPixmap dotPixmap(DotState state) {
	QPixmap pixmap(kDotDiameter, kDotDiameter);
	pixmap.fill(Qt::transparent);
	QPainter painter(&pixmap);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setPen(Qt::NoPen);
	QColor color;
	switch (state) {
	case DotState::On:       color = QColor(0x2e, 0xa8, 0x4f); break;
	case DotState::Disabled: color = QColor(0x9a, 0x9a, 0x9a); break;
	default:                 color = QColor(0xd9, 0x3a, 0x3a); break;
	}
	painter.setBrush(color);
	painter.drawEllipse(0, 0, kDotDiameter - 1, kDotDiameter - 1);
	return pixmap;
}

QLabel* makeIcon(QWidget* parent) {
	auto* icon = new QLabel(parent);
	icon->setFixedSize(kDotDiameter, kDotDiameter);
	icon->setPixmap(dotPixmap(DotState::Off));
	return icon;
}

// Keeps the same label keys on screen whether or not a value is available
// yet, so the row's height (and the gap above the history list) never
// changes -- only the numbers themselves appear/disappear.
QString snapshotText(const QString& altitude, const QString& heading, const QString& speed, const QString& verticalSpeed) {
	return QString("Alt %1 ft | Hdg %2° | Spd %3 kt | V/S %4 ft/min")
		.arg(altitude, heading, speed, verticalSpeed);
}

}

LiveStatusPanel::LiveStatusPanel(RecorderBridge& bridge, QWidget* parent)
	: QWidget(parent)
	, bridge_(bridge)
{
	// Create version label
	versionLabel_ = new QLabel(QString("ver. %1").arg(APP_VERSION), this);
	QFont versionFont = versionLabel_->font();
	versionFont.setPointSize(versionFont.pointSize() - 1);
	versionLabel_->setFont(versionFont);
	versionLabel_->setAlignment(Qt::AlignRight);

	// Create separator line
	auto* separator = new QFrame(this);
	separator->setFrameShape(QFrame::HLine);
	separator->setFrameShadow(QFrame::Sunken);
	separator->setFixedHeight(1);

	auto* titleLabel = new QLabel("<b>Live Status</b>", this);

	connectionIcon_ = makeIcon(this);
	recordingIcon_ = makeIcon(this);

	snapshotLabel_ = new QLabel(this);
	// The panel can be narrower than the full "Alt ... | Hdg ... | Spd ... |
	// V/S ..." line with real values filled in -- without this, Qt would wrap
	// it onto a second line and grow the row height instead of just clipping.
	snapshotLabel_->setWordWrap(false);
	snapshotLabel_->setText(snapshotText("-", "-", "-", "-"));

	historyList_ = new QListWidget(this);
	historyList_->setAlternatingRowColors(true);
	historyList_->setSelectionMode(QAbstractItemView::NoSelection);
	historyList_->setFocusPolicy(Qt::NoFocus);
	historyList_->setStyleSheet(
		"QListWidget { outline: none; }"
		"QListWidget::item:hover { background: transparent; }"
		"QListWidget::item:selected { background: transparent; color: inherit; }"
	);

	setIndicator(connectionIcon_, false, "Waiting for simulator…");

	recordingLabel_ = new QLabel("Recording:", this);

	// Wrap the label + dot in one small container so a hover highlight and
	// click both cover the pair as a single target, rather than the label and
	// dot behaving as two disconnected hit areas.
	recordingToggle_ = new QWidget(this);
	recordingToggle_->setObjectName("recordingToggle");
	// A static outline (rather than only a hover highlight) is the discoverability
	// cue: it reads as a small button/chip at rest, distinguishing this control from
	// the plain, non-interactive Connection indicator beside it.
	recordingToggle_->setStyleSheet(
		"QWidget#recordingToggle { border: 1px solid rgba(128, 128, 128, 90); border-radius: 4px; }"
		"QWidget#recordingToggle:hover { background-color: rgba(128, 128, 128, 60); }"
	);
	auto* recordingToggleLayout = new QHBoxLayout(recordingToggle_);
	recordingToggleLayout->setContentsMargins(2, 1, 2, 1);
	recordingToggleLayout->setSpacing(4);
	recordingToggleLayout->addWidget(recordingLabel_);
	recordingToggleLayout->addWidget(recordingIcon_);
	recordingToggle_->installEventFilter(this);

	auto* titleRow = new QHBoxLayout();
	titleRow->addWidget(titleLabel);
	titleRow->addStretch(1);
	titleRow->addWidget(new QLabel("Connection:", this));
	titleRow->addSpacing(4);
	titleRow->addWidget(connectionIcon_);
	titleRow->addSpacing(8);
	titleRow->addWidget(recordingToggle_);

	updateRecordingIndicator();

	auto* layout = new QVBoxLayout(this);
	// Default QVBoxLayout spacing (~9-11px per style) compounds across four
	// stacked rows into a visibly large gap above the history list -- pull it
	// in tight since these rows are all one cohesive status block. Left margin
	// is also thinned to match tripHistoryPanel_'s right margin, since the two
	// panels sit side by side in MainWindow's splitter.
	layout->setContentsMargins(0, 4, 4, 4);
	layout->setSpacing(0);
	layout->addWidget(versionLabel_);
	layout->addSpacing(4);
	layout->addWidget(separator);
	layout->addSpacing(4);
	layout->addLayout(titleRow);
	layout->addWidget(snapshotLabel_);
	layout->addWidget(historyList_, 1);

	connect(&bridge_, &RecorderBridge::logMessage, this, &LiveStatusPanel::onLogMessage);
	connect(&bridge_, &RecorderBridge::connectionChanged, this, &LiveStatusPanel::onConnectionChanged);
	connect(&bridge_, &RecorderBridge::recordingStateChanged, this, &LiveStatusPanel::onRecordingStateChanged);
	connect(&bridge_, &RecorderBridge::tripEnded, this, &LiveStatusPanel::onTripEnded);
	connect(&bridge_, &RecorderBridge::recordingEnabledChanged, this, &LiveStatusPanel::onRecordingEnabledChanged);
	connect(&bridge_, &RecorderBridge::sampleUpdated, this, &LiveStatusPanel::onSampleUpdated);
	connect(&bridge_, &RecorderBridge::eventCommitted, this, &LiveStatusPanel::onEventCommitted);
	connect(&bridge_, &RecorderBridge::eventsRetracted, this, &LiveStatusPanel::onEventsRetracted);
}

void LiveStatusPanel::setIndicator(QLabel* icon, bool active, const QString& tooltip) {
	icon->setPixmap(dotPixmap(active ? DotState::On : DotState::Off));
	icon->setToolTip(tooltip);
}

QListWidgetItem* LiveStatusPanel::appendHistoryItem(const QString& text) {
	QDateTime now = QDateTime::currentDateTime();
	historyList_->addItem(QString("[%1] %2").arg(now.toString("yyyy-MM-dd hh:mm:ss"), text));
	QListWidgetItem* item = historyList_->item(historyList_->count() - 1);
	// Cap so an extremely long recording session doesn't grow this list (and its
	// underlying widget items) without bound.
	constexpr int kMaxHistoryItems = 500;
	while (historyList_->count() > kMaxHistoryItems) {
		QListWidgetItem* pruned = historyList_->takeItem(0);
		// Keep eventItems_ from ever holding a dangling pointer once its line
		// scrolls out of the capped history -- a late retraction for it then
		// becomes a harmless no-op lookup miss in onEventsRetracted instead of
		// touching freed memory. eventItems_ is expected to stay small (only
		// recently-committed, not-yet-retracted/not-yet-pruned events), so
		// this linear scan is cheap and only runs when the cap is actually hit.
		for (auto it = eventItems_.begin(); it != eventItems_.end(); ++it) {
			if (it.value() == pruned) {
				eventItems_.erase(it);
				break;
			}
		}
		delete pruned;
	}
	historyList_->scrollToBottom();
	return item;
}

void LiveStatusPanel::onLogMessage(const QString& text) {
	const QString trimmed = text.trimmed();
	if (trimmed.isEmpty())
		return;
	appendHistoryItem(trimmed);
}

void LiveStatusPanel::onEventCommitted(int tripId, quint64 seq, const QString& text) {
	if (tripId != recordingTripId_) {
		Logger::logf(Logger::Trace, "LiveStat", "eventCommitted(seq=%llu, trip=%d) ignored: current recording trip is #%d", seq, tripId, recordingTripId_);
		return;
	}
	const QString trimmed = text.trimmed();
	if (trimmed.isEmpty())
		return;
	eventItems_.insert(seq, appendHistoryItem(trimmed));
}

void LiveStatusPanel::onEventsRetracted(const QList<quint64>& seqs) {
	for (quint64 seq : seqs) {
		auto it = eventItems_.find(seq);
		if (it == eventItems_.end())
			continue;
		QListWidgetItem* item = it.value();
		eventItems_.erase(it);
		int row = historyList_->row(item);
		if (row >= 0)
			delete historyList_->takeItem(row);
	}
}

void LiveStatusPanel::onConnectionChanged(bool connected) {
	Logger::logf(Logger::Trace, "LiveStat", "Connection indicator -> %s", connected ? "connected" : "waiting");
	setIndicator(connectionIcon_, connected,
		connected ? "Connected to Microsoft Flight Simulator" : "Waiting for simulator…");
}

void LiveStatusPanel::onRecordingStateChanged(int tripId) {
	Logger::logf(Logger::Trace, "LiveStat", "Recording indicator -> recording trip #%d", tripId);
	recordingTripId_ = tripId;
	updateRecordingIndicator();
}

void LiveStatusPanel::onTripEnded(int tripId) {
	// The DB writer's queue can deliver this after a new trip has already
	// started recording (it's only flushed once that trip's samples finish
	// draining); ignore it then so the indicator doesn't go stale.
	if (tripId != recordingTripId_) {
		Logger::logf(Logger::Trace, "LiveStat", "tripEnded(%d) ignored: current recording trip is #%d", tripId, recordingTripId_);
		return;
	}
	recordingTripId_ = -1;
	Logger::logf(Logger::Trace, "LiveStat", "Recording indicator -> not recording (trip #%d ended)", tripId);
	updateRecordingIndicator();
}

void LiveStatusPanel::onRecordingEnabledChanged(bool enabled) {
	Q_UNUSED(enabled);
	updateRecordingIndicator();
}

void LiveStatusPanel::updateRecordingIndicator() {
	const bool recording = bridge_.isRecording();
	const bool enabled = bridge_.isRecordingEnabled();

	DotState state;
	QString tooltip;
	if (recording) {
		state = DotState::On;
		tooltip = QString("Recording trip #%1 (can't be toggled while a trip is recording)").arg(recordingTripId_);
	} else if (!enabled) {
		state = DotState::Disabled;
		tooltip = "Click to enable automatic recording.";
	} else {
		state = DotState::Off;
		tooltip = "Click to disable automatic recording.";
	}

	recordingIcon_->setPixmap(dotPixmap(state));
	// Tooltip lives on recordingToggle_ only (not the child label/icon) so
	// eventFilter's Enter handler below is the sole thing that shows it --
	// setting it on the children too would leave Qt's own delayed tooltip
	// timer racing (and duplicating) the instant one shown for the container.
	recordingToggle_->setToolTip(tooltip);
	recordingToggle_->setCursor(recording ? Qt::ArrowCursor : Qt::PointingHandCursor);
}

void LiveStatusPanel::toggleRecordingEnabled() {
	if (bridge_.isRecording())
		return;
	bridge_.setRecordingEnabled(!bridge_.isRecordingEnabled());
}

bool LiveStatusPanel::eventFilter(QObject* obj, QEvent* event) {
	if (obj != recordingToggle_)
		return QWidget::eventFilter(obj, event);

	switch (event->type()) {
	case QEvent::MouseButtonRelease: {
		// Qt implicitly grabs the mouse to whichever widget received the
		// matching press, so a release delivered here can still land outside
		// recordingToggle_'s bounds if the user dragged off before releasing
		// -- require the release position to still be inside so that gesture
		// (a conventional way to abort a click) doesn't fire the toggle.
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton && recordingToggle_->rect().contains(mouseEvent->pos()))
			toggleRecordingEnabled();
		return true;
	}
	case QEvent::Enter:
		// Bypasses Qt's default ~700ms tooltip wake-up delay (QStyle::
		// SH_ToolTip_WakeUpDelay), which reads as sluggish for a control this
		// small -- show it the instant the pointer arrives instead.
		QToolTip::showText(QCursor::pos(), recordingToggle_->toolTip(), recordingToggle_);
		return false;
	case QEvent::Leave:
		QToolTip::hideText();
		return false;
	default:
		return QWidget::eventFilter(obj, event);
	}
}

void LiveStatusPanel::onSampleUpdated() {
	const FLIGHT_DATA& data = bridge_.currentData();
	snapshotLabel_->setText(snapshotText(
		QString::number(data.altitude),
		QString::number(data.heading),
		QString::number(data.speed),
		QString::number(data.vertical_speed)));
}


#include "live_status_panel.h"
#include "recorder_bridge.h"
#include "types.h"

#include <QDateTime>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>

namespace {

constexpr int kDotDiameter = 16;

// Painted solid-color dot rather than a Unicode glyph (power/record symbols
// etc.): even "text presentation" code points fall back to Windows' color
// emoji font when the UI font lacks the glyph, and that font ignores
// QLabel's stylesheet/text color entirely -- painting the pixmap ourselves
// guarantees a strict green/red result on every platform.
QPixmap dotPixmap(bool active) {
	QPixmap pixmap(kDotDiameter, kDotDiameter);
	pixmap.fill(Qt::transparent);
	QPainter painter(&pixmap);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setPen(Qt::NoPen);
	painter.setBrush(active ? QColor(0x2e, 0xa8, 0x4f) : QColor(0xd9, 0x3a, 0x3a));
	painter.drawEllipse(0, 0, kDotDiameter - 1, kDotDiameter - 1);
	return pixmap;
}

QLabel* makeIcon(QWidget* parent) {
	auto* icon = new QLabel(parent);
	icon->setFixedSize(kDotDiameter, kDotDiameter);
	icon->setPixmap(dotPixmap(false));
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
	snapshotLabel_->setText(snapshotText("—", "—", "—", "—"));

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
	setIndicator(recordingIcon_, false, "Not recording");

	auto* titleRow = new QHBoxLayout();
	titleRow->addWidget(titleLabel);
	titleRow->addStretch(1);
	titleRow->addWidget(new QLabel("Connection:", this));
	titleRow->addWidget(connectionIcon_);
	titleRow->addSpacing(12);
	titleRow->addWidget(new QLabel("Recording:", this));
	titleRow->addWidget(recordingIcon_);

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
	connect(&bridge_, &RecorderBridge::sampleUpdated, this, &LiveStatusPanel::onSampleUpdated);
}

void LiveStatusPanel::setIndicator(QLabel* icon, bool active, const QString& tooltip) {
	icon->setPixmap(dotPixmap(active));
	icon->setToolTip(tooltip);
}

void LiveStatusPanel::onLogMessage(const QString& text) {
	const QString trimmed = text.trimmed();
	if (trimmed.isEmpty())
		return;
	QDateTime now = QDateTime::currentDateTime();
	historyList_->addItem(QString("[%1] %2").arg(now.toString("hh:mm:ss"), trimmed));
	historyList_->scrollToBottom();
}

void LiveStatusPanel::onConnectionChanged(bool connected) {
	setIndicator(connectionIcon_, connected,
		connected ? "Connected to Microsoft Flight Simulator" : "Waiting for simulator…");
}

void LiveStatusPanel::onRecordingStateChanged(int tripId) {
	setIndicator(recordingIcon_, true, QString("Recording (trip #%1)").arg(tripId));
}

void LiveStatusPanel::onTripEnded(int /*tripId*/) {
	setIndicator(recordingIcon_, false, "Not recording");
}

void LiveStatusPanel::onSampleUpdated() {
	const FLIGHT_DATA& data = bridge_.currentData();
	snapshotLabel_->setText(snapshotText(
		QString::number(data.altitude),
		QString::number(data.heading),
		QString::number(data.speed),
		QString::number(data.vertical_speed)));
}


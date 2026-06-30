#pragma once

#include <QString>

// Durable log file next to the executable (flight_data_recorder.log),
// written alongside the existing LiveStatusPanel UI feed so log history
// survives even when nothing is listening to RecorderBridge::logMessage.
// Hooked into gui_notify_log() (recorder_bridge.cpp), the single funnel
// point all recorder log text already passes through.
namespace Logger {

void log(const QString& text);

}

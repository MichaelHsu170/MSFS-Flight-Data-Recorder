#pragma once

#include <QtGlobal>

#include <time.h>

// Kept out of test_support.h, whose windows.h min/max macros break the Qt
// Graphs headers the chart tests include.
namespace TestSupport {

// Makes US Pacific time (PST8PDT, with its DST changes) the process's local
// time zone for the rest of the run. Call it from initTestCase(): Qt keeps
// the zone it first computes a local time in and ignores later changes.
inline void usePacificLocalTime() {
	qputenv("TZ", "PST8PDT");
	_tzset();
}

}

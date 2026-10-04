#pragma once

#include "types.h"

// The trip's life, driven by the simulator's samples: a trip starts when
// any of the aircraft's engines runs on the ground (and recording is enabled),
// stops when all are off on the ground; while it records, the first liftoff is its departure,
// later liftoffs (touch-and-goes) and every touchdown are markers, each
// written to the database the moment it happens and resolved to an airport/
// runway afterwards by the airport lookup (airport_lookup.h), one at a time
// and in order. Samples are queued for trip_data every sample_interval_ms.
// State lives in STATUS::flight.

// One decoded simulator sample (every sim frame).
void flight_on_sample(struct STATUS* status, const FLIGHT_DATA_RECORD& sample);

// Ends the recording trip: stores its destination time, frees its liftoff/
// touchdown records and queues the end-of-trip barrier behind its samples.
void stop_recording(struct STATUS* status);

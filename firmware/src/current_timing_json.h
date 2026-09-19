#pragma once
#include <Arduino.h>
#include "current_timing_audit.h"

// Serialization is performed only for a stopped-run download, never at a read
// or violation. The arrays and counters are bounded at acquisition time.
void appendCurrentTimingJson(String& json, const CurrentTimingAudit& audit);

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "../native_test/Arduino.h"
using std::max;
using std::min;
extern uint32_t fake_us;
// Let nested safety checks cross a deadline, as they can on hardware.
inline uint32_t micros() { return fake_us++; }
inline uint32_t millis() { return fake_us / 1000; }
inline void* ps_malloc(size_t n) { return malloc(n); }

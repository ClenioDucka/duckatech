/*
 * powerLatch.cpp
 * -------------------------------------------------------------------------
 * See powerLatch.h for the full design description.
 * -------------------------------------------------------------------------
 */

#include "powerLatch.h"
#include <Arduino.h>
#include "globals.h"
#include "adaptation.h"
#include "src/controllers/fuelPump/fuelPumpController.h"

enum PowerLatchState : uint8_t {
  PL_RUNNING = 0,       // pump off (engine not yet run long enough) or pump on -- normal tracking
  PL_DEBOUNCE = 1,      // pump turned off after a qualifying run -- confirming it's a real shutdown
  PL_SHUTDOWN_DONE = 2  // EEPROM write completed, just waiting for the relay to drop power
};

static PowerLatchState g_state = PL_RUNNING;
static bool g_wasPumpOn = false;
static uint32_t g_pumpOnStartMs = 0;
static uint32_t g_pumpOffStartMs = 0;
static uint32_t g_lastUpdateMs = 0;

void powerLatch_init(void) {
  g_state = PL_RUNNING;
  g_wasPumpOn = isFuelPumpOn();
  g_pumpOnStartMs = g_wasPumpOn ? millis() : 0;
  g_pumpOffStartMs = 0;
}

void powerLatch_update(void) {
  uint32_t now = millis();
  if ((uint32_t)(now - g_lastUpdateMs) < 100U) { return; } // ~10Hz
  g_lastUpdateMs = now;

  bool pumpOn = isFuelPumpOn();

  if (pumpOn && !g_wasPumpOn) {
    // Pump just turned ON -- start tracking a fresh run, and cancel any
    // in-progress shutdown sequence (this covers both "pump came back on
    // during debounce" and "engine restarted after a completed write").
    g_pumpOnStartMs = now;
    g_state = PL_RUNNING;
  }
  else if (!pumpOn && g_wasPumpOn) {
    // Pump just turned OFF -- only a shutdown candidate if the prior ON
    // stretch met the minimum run time (excludes key-on priming pulses
    // and short aborted starts).
    uint32_t onDurationMs = now - g_pumpOnStartMs;
    uint32_t minRunMs = (uint32_t)configPage15.adaptMinRunTimeSec * 1000UL;
    if (minRunMs == 0UL) { minRunMs = 600000UL; } // safe fallback if config unset/zero (10 min)

    if (onDurationMs >= minRunMs) {
      g_pumpOffStartMs = now;
      g_state = PL_DEBOUNCE;
    }
    // else: too short to be a real run -- ignore, stay in PL_RUNNING
  }
  g_wasPumpOn = pumpOn;

  if (g_state == PL_DEBOUNCE && !pumpOn) {
    uint32_t debounceMs = (uint32_t)configPage15.adaptShutdownDebounceSec * 1000UL;
    if (debounceMs == 0UL) { debounceMs = 4000UL; } // safe fallback if config unset/zero (4s)
    if ((now - g_pumpOffStartMs) >= debounceMs) {
      adaptation_onIgnitionOff(); // commits the learned-data EEPROM write if dirty
      g_state = PL_SHUTDOWN_DONE;
    }
  }
}

bool powerLatch_isShuttingDown(void) {
  return g_state != PL_RUNNING;
}

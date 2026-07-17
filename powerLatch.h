/*
 * powerLatch.h
 * -------------------------------------------------------------------------
 * Shutdown detection for a timed self-latch relay setup (the classic Ford
 * EEC-IV / VW "power hold relay" topology): a dumb, fixed-duration relay
 * (~10s, adjustable on the relay itself) keeps the ECU powered after the
 * driver turns the key off, giving the firmware a window to do its
 * shutdown work before power drops on its own.
 *
 * ZERO new hardware inputs. This uses ONLY the fuel pump's existing,
 * already-tracked on/off state (via fuelPumpController's isFuelPumpOn(),
 * one query function added to that module) -- no ignition-sense pin, no
 * separate confirmation sensor.
 *
 * LOGIC:
 *   1. Track how long the fuel pump has been continuously ON.
 *   2. When the pump turns OFF, check whether that ON stretch met
 *      configPage15.adaptMinRunTimeSec (default 600s / 10 min). If it
 *      didn't, this was just a key-on priming pulse or an aborted/short
 *      start -- ignore it entirely, no shutdown sequence starts.
 *   3. If the ON stretch DID meet the threshold, this OFF transition is a
 *      shutdown candidate: start the debounce timer.
 *   4. If the pump stays OFF continuously for
 *      configPage15.adaptShutdownDebounceSec seconds (default 4s, your
 *      3-5s range), treat this as a genuine shutdown and call
 *      adaptation_onIgnitionOff() to commit the learned-data EEPROM write.
 *   5. If the pump turns back ON at any point (debounce or after the
 *      write), reset to normal tracking -- no write on a false trigger,
 *      and if it happens after a write already landed, nothing bad
 *      happens either (adaptation_onIgnitionOff() no-ops when nothing is
 *      dirty).
 *
 * TIMING BUDGET: candidate-detection is instant once the pump goes off,
 * debounce is 3-5s (tunable), the EEPROM write itself is under 100ms --
 * all comfortably inside your relay's ~10s hardware timeout.
 *
 * KNOWN EDGE CASE, worth knowing rather than a blocker: if the engine
 * stalls and is restarted mid-drive (after already running past the
 * min-run-time threshold), the brief pump-off during the stall becomes a
 * shutdown candidate exactly like a real key-off would. If the restart
 * completes within your debounce window (typically true -- cranking
 * re-engages the pump almost immediately), it aborts cleanly. If a
 * stall-to-restart genuinely takes longer than your debounce setting, this
 * would fire a write mid-drive. Harmless (just an earlier-than-necessary
 * save of whatever's been learned so far), but worth knowing if you're
 * chasing an unexpected EEPROM write during bench testing.
 * -------------------------------------------------------------------------
 */

#ifndef POWERLATCH_H
#define POWERLATCH_H

#include <stdint.h>

void powerLatch_init(void);

// Call every main loop iteration (internally rate-limited to ~10Hz --
// plenty for tracking on/off transitions and second-scale timers).
void powerLatch_update(void);

// True from the moment a shutdown candidate is detected through to the
// completed write -- useful as a TunerStudio output-channel flag for
// watching the state machine live while bench-testing the relay timing.
bool powerLatch_isShuttingDown(void);

#endif // POWERLATCH_H

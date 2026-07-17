/*
 * adaptation.h
 * -------------------------------------------------------------------------
 * Self-adaptation module (Bosch-style LTFT/STFT for fuel + idle) for the
 * ClenioDucka/speeducka fork of Speeduino.
 *
 * This version is wired directly to speeducka's real types and globals --
 * no placeholder TODOs for the core hooks. Verified against a clone of
 * https://github.com/ClenioDucka/speeducka (master, as of this writing):
 *   - currentStatus.RPM / MAP / TPS / coolant / egoCorrection / isDFCOActive
 *     / AEamount / idleOn / idleLoad / CLIdleTarget  (statuses.h)
 *   - configPage6.egoAlgorithm, EGO_ALGORITHM_* constants (config_pages.h)
 *   - configPage15.adapt* fields -- added directly into config15's reserved
 *     block, see the config_pages.h patch shipped alongside this file
 *   - getLoad(LoadSource, statuses) from load_source.h
 *   - getStorageAPI() from storage.h / storage_api.h (portable across your
 *     AVR2560/STM32/Teensy board targets, not just raw EEPROM.h)
 *
 * PERSISTENCE STRATEGY (unchanged from the original design):
 *   - STFT: not persisted. speeduino's own currentStatus.egoCorrection IS
 *     your fuel STFT (100 = neutral) -- this module does not duplicate that
 *     closed-loop controller, it just reads the result.
 *   - Fuel LTFT / idle LTFT: learned in RAM during the drive, written to
 *     EEPROM once during the ignition-off power-latch window via
 *     adaptation_onIgnitionOff(). Double-buffered with CRC16 so a corrupted
 *     write can't brick the learned table.
 *   - Config (enable switches + learning window): lives inside configPage15
 *     (a real TS page), so it's loaded/saved automatically by speeduino's
 *     existing loadAllPages()/savePage() -- no separate load/save code
 *     needed here at all.
 *
 * EEPROM MAP (see ADDRESSES.md for full derivation):
 *   - Config fields: inside configPage15, EEPROM abs. address 3307-3320
 *     (officially-reserved space, durable across upstream merges)
 *   - Learned-data double buffer: EEPROM abs. address 3460-3630
 *     (unclaimed gap between page15's end and the calibration tables at the
 *     time this was written -- RE-VERIFY if you merge upstream Speeduino
 *     changes; see ADDRESSES.md)
 * -------------------------------------------------------------------------
 */

#ifndef ADAPTATION_H
#define ADAPTATION_H

#include <stdint.h>
#include "globals.h"
#include "load_source.h"

// ============================================================================
// Bin definitions
// ============================================================================

#define LTFT_RPM_BINS   8
#define LTFT_LOAD_BINS  8
#define IDLE_CLT_BINS   6

extern const uint16_t ltftRpmBins[LTFT_RPM_BINS];
// Units depend on configPage15.adaptLoadSource: MAP mode = kPa, TPS mode = %
// (matches getLoad()'s LOAD_SOURCE_TPS convention of TPS*2, so this can go
// to 200 -- see adaptation_getSelectedLoad()). Widened to uint16_t for boost.
extern const uint16_t ltftLoadBins[LTFT_LOAD_BINS];
extern const int16_t  idleCltBins[IDLE_CLT_BINS]; // deg C, matches currentStatus.coolant directly (no +40 offset -- that offset is only applied at the wire/table-lookup layer via temperatureAddOffset())

// ============================================================================
// Tunable constants (compile-time; the learning WINDOW itself is runtime-
// tunable via configPage15, these are algorithm internals)
// ============================================================================

#define LTFT_LSB_PCT_X10        5      // 1 LSB = 0.5% (tenths-of-percent units)
#define LTFT_MAX_TRIM_COUNTS    50     // 50 * 0.5% = +-25%

#define IDLE_IGN_LSB_DEG_X10    1
#define IDLE_IGN_MAX_COUNTS     60     // +-6.0 degrees

#define IDLE_AIR_MAX_DELTA      15     // currentStatus.idleLoad is 0-100%, so
                                        // +-15% is a generous learned-drift range

#define LEARN_IIR_SHIFT          4     // ~1/16th of error folded in per commit

#define STEADY_WINDOW             8
#define STEADY_RPM_DELTA_MAX     150   // RPM
#define STEADY_LOAD_DELTA_MAX      6   // kPa or %, matches ltftLoadBins units

#define ADAPT_UPDATE_INTERVAL_MS 100

// ============================================================================
// Persisted learned-data block (double-buffered, CRC-protected, written only
// on ignition-off)
// ============================================================================

#pragma pack(push, 1)
struct AdaptationBlock {
  uint8_t  version;
  int8_t   fuelLtft[LTFT_RPM_BINS][LTFT_LOAD_BINS];    // 0.5%/count
  int16_t  idleAirBase[IDLE_CLT_BINS];                 // delta vs design center, native 0-100% units
  int8_t   idleIgnTrim[IDLE_CLT_BINS];                 // 0.1 deg/count
  uint16_t crc;
};
#pragma pack(pop)

#define ADAPT_BLOCK_VERSION  1

// ============================================================================
// Public API
// ============================================================================

// Call once from setup(), after loadAllPages() / initialiseAll() -- config15
// (and its embedded adaptation config fields) is already loaded by that
// point via speeduino's normal page-load mechanism. This just loads the
// separate learned-data double buffer.
void adaptation_init(void);

// Call from loop() periodically (internally rate-limited to ADAPT_UPDATE_INTERVAL_MS).
// Reads currentStatus directly -- no arguments needed.
void adaptation_update(void);

// --- Fuel ---
// Returns the combined LTFT+egoCorrection-derived trim as a percentage in
// the SAME convention as speeduino's own corrections pipeline (100 = neutral,
// suitable for combineCorrections()). Call this from corrections.cpp right
// after currentStatus.egoCorrection is combined into sumCorrections -- see
// corrections.cpp patch.
uint16_t adaptation_getFuelTrimPercent100(void);

// --- Idle ---
// Returns the value to ADD to currentStatus.idleLoad (native 0-100% units)
// just before the "if (idleLoad > 100) idleLoad = 100;" safety clamp in
// idle.cpp. Returns 0 until idle learning is enabled and has learned
// something for the current coolant temp bin.
int16_t adaptation_getIdleAirDelta(void);

// Returns the learned+live idle ignition trim in tenths of a degree, for
// wherever your idle ignition advance table lookup happens (add this on top
// of the base idle-advance-table value).
int16_t adaptation_getIdleIgnTrim_x10(void);

// Call from the shutdown sequence while self-latched (ignition off, ECU
// still powered), BEFORE dropping the latch. Writes the learned-data double
// buffer to EEPROM only if something changed. Returns true if a write occurred.
bool adaptation_onIgnitionOff(void);

// Resets all learned trims to zero and commits immediately (does not wait
// for ignition-off). Wire this to the "Clear Adaptation" command button --
// see TS_CommandButtonHandler patch.
void adaptation_clearAll(void);

// --- TunerStudio output channels ---
// Fills a 9-byte block for the live-telemetry output channels. See
// logger.cpp patch for where these bytes get appended (cases 139-147).
#pragma pack(push, 1)
struct AdaptationOutputChannels {
  int16_t fuelLtft_x10;
  int16_t fuelStft_x10;   // mirrors (currentStatus.egoCorrection - 100) * 10
  int16_t idleAirTrim;
  int16_t idleIgn_x10;
  uint8_t adaptFlags;     // bit0 dirty, bit1 fuelWindowActive, bit2 idleWindowActive,
                          // bit3 fuelLearnEnabled, bit4 idleLearnEnabled, bit5 loadSource(TPS),
                          // bit6 powerLatchShuttingDown (see powerLatch.h)
};
#pragma pack(pop)
void adaptation_fillOutputChannels(AdaptationOutputChannels &out);

// Single shared instance, filled once per output-channel packet by
// generateLiveValues() in comms.cpp, read byte-by-byte by getTSLogEntry()
// in logger.cpp (cases 139-147). Avoids recomputing per byte.
extern AdaptationOutputChannels adaptationOutputChannelsCache;

#endif // ADAPTATION_H

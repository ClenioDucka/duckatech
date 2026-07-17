/*
 * adaptation.cpp
 * -------------------------------------------------------------------------
 * See adaptation.h for design notes and the exact EEPROM address derivation
 * in ADDRESSES.md. This implementation reads real speeducka globals
 * directly -- no TODO(fork) placeholders remain for the core hooks.
 * -------------------------------------------------------------------------
 */

#include "adaptation.h"
#include "storage.h"
#include "storage_api.h"
#include "powerLatch.h"
#include <string.h>

AdaptationOutputChannels adaptationOutputChannelsCache;

// ============================================================================
// EEPROM addresses for the learned-data double buffer
// -----------------------------------------------------------------------
// See ADDRESSES.md for the full derivation from speeducka's real storage.cpp
// constants (EEPROM_CONFIG15_START=3281, sizeof(config15)=176 verified via
// offsetof, EEPROM_LAST_BARO=3806 derived from calibration table sizes).
// This range (3457-3806) was UNCLAIMED at the time of writing but is not an
// officially reserved block the way configPage15's Unused15_106_255 is --
// re-verify against your storage.cpp if you merge upstream Speeduino changes.
// ============================================================================

#define ADAPT_POINTER_ADDR   3460U
#define ADAPT_BLOCK_A_ADDR   3461U
#define ADAPT_BLOCK_B_ADDR   (ADAPT_BLOCK_A_ADDR + sizeof(AdaptationBlock))

// ============================================================================
// Bin tables
// ============================================================================

const uint16_t ltftRpmBins[LTFT_RPM_BINS]   = {600, 1000, 1500, 2000, 2500, 3500, 4500, 6000};
const uint16_t ltftLoadBins[LTFT_LOAD_BINS] = {20, 40, 60, 80, 100, 130, 160, 200};
const int16_t  idleCltBins[IDLE_CLT_BINS]   = {-10, 10, 30, 50, 70, 90};

// ============================================================================
// RAM state
// ============================================================================

static AdaptationBlock g_active;
static bool            g_dirty = false;

static uint8_t g_fuelHitCounter[LTFT_RPM_BINS][LTFT_LOAD_BINS];
static int8_t  g_fuelHitSign[LTFT_RPM_BINS][LTFT_LOAD_BINS];
static uint8_t g_idleHitCounter[IDLE_CLT_BINS];
static int8_t  g_idleHitSign[IDLE_CLT_BINS];

static bool g_fuelWindowActive = false;
static bool g_idleWindowActive = false;

static int16_t g_liveFuelLtft_x10 = 0;
static int16_t g_liveIdleAirTrim  = 0;
static int16_t g_liveIdleIgn_x10  = 0;

static uint16_t g_rpmHist[STEADY_WINDOW];
static uint16_t g_loadHist[STEADY_WINDOW];
static uint8_t  g_histIdx = 0;
static uint8_t  g_histFilled = 0;

static uint32_t g_lastUpdateMs = 0;

// ============================================================================
// CRC16 (CCITT)
// ============================================================================

static uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}

static uint16_t crc16_block(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) crc = crc16_update(crc, buf[i]);
  return crc;
}

// ============================================================================
// Bin lookup + bilinear interpolation
// ============================================================================

static void findBinU16(uint16_t val, const uint16_t *bins, uint8_t n, uint8_t &loIdx, uint8_t &frac256) {
  if (val <= bins[0]) { loIdx = 0; frac256 = 0; return; }
  if (val >= bins[n - 1]) { loIdx = n - 2; frac256 = 255; return; }
  for (uint8_t i = 0; i < n - 1; i++) {
    if (val >= bins[i] && val <= bins[i + 1]) {
      uint16_t span = bins[i + 1] - bins[i];
      uint16_t off  = val - bins[i];
      loIdx = i;
      frac256 = (uint8_t)(((uint32_t)off * 255) / span);
      return;
    }
  }
  loIdx = n - 2; frac256 = 255;
}

static void findBinI16(int16_t val, const int16_t *bins, uint8_t n, uint8_t &loIdx, uint8_t &frac256) {
  if (val <= bins[0]) { loIdx = 0; frac256 = 0; return; }
  if (val >= bins[n - 1]) { loIdx = n - 2; frac256 = 255; return; }
  for (uint8_t i = 0; i < n - 1; i++) {
    if (val >= bins[i] && val <= bins[i + 1]) {
      int16_t span = bins[i + 1] - bins[i];
      int16_t off  = val - bins[i];
      loIdx = i;
      frac256 = (uint8_t)(((int32_t)off * 255) / span);
      return;
    }
  }
  loIdx = n - 2; frac256 = 255;
}

// ============================================================================
// EEPROM load / save (learned-data double buffer, via getStorageAPI())
// ============================================================================

static bool loadAndValidate(uint16_t address, AdaptationBlock &out) {
  uint8_t *p = (uint8_t *)&out;
  const storage_api_t &api = getStorageAPI();
  for (size_t i = 0; i < sizeof(AdaptationBlock); i++) {
    p[i] = api.read(address + i);
  }
  if (out.version != ADAPT_BLOCK_VERSION) return false;
  uint16_t calc = crc16_block(p, sizeof(AdaptationBlock) - sizeof(uint16_t));
  return calc == out.crc;
}

static void zeroBlock(AdaptationBlock &b) {
  memset(&b, 0, sizeof(AdaptationBlock));
  b.version = ADAPT_BLOCK_VERSION;
}

void adaptation_init(void) {
  const storage_api_t &api = getStorageAPI();
  uint8_t pointer = api.read(ADAPT_POINTER_ADDR);
  AdaptationBlock candidate;
  bool loaded = false;

  if (pointer == 0 && loadAndValidate(ADAPT_BLOCK_A_ADDR, candidate)) {
    loaded = true;
  } else if (pointer == 1 && loadAndValidate(ADAPT_BLOCK_B_ADDR, candidate)) {
    loaded = true;
  } else {
    if (loadAndValidate(ADAPT_BLOCK_A_ADDR, candidate)) loaded = true;
    else if (loadAndValidate(ADAPT_BLOCK_B_ADDR, candidate)) loaded = true;
  }

  if (loaded) { g_active = candidate; }
  else { zeroBlock(g_active); }

  memset(g_fuelHitCounter, 0, sizeof(g_fuelHitCounter));
  memset(g_fuelHitSign, 0, sizeof(g_fuelHitSign));
  memset(g_idleHitCounter, 0, sizeof(g_idleHitCounter));
  memset(g_idleHitSign, 0, sizeof(g_idleHitSign));
  memset(g_rpmHist, 0, sizeof(g_rpmHist));
  memset(g_loadHist, 0, sizeof(g_loadHist));
  g_histIdx = 0;
  g_histFilled = 0;
  g_dirty = false;
}

bool adaptation_onIgnitionOff(void) {
  if (!g_dirty) return false;

  const storage_api_t &api = getStorageAPI();
  uint8_t currentPointer = api.read(ADAPT_POINTER_ADDR);
  uint16_t targetAddr = (currentPointer == 0) ? ADAPT_BLOCK_B_ADDR : ADAPT_BLOCK_A_ADDR;
  uint8_t newPointerVal = (currentPointer == 0) ? 1 : 0;

  g_active.version = ADAPT_BLOCK_VERSION;
  uint8_t *p = (uint8_t *)&g_active;
  g_active.crc = crc16_block(p, sizeof(AdaptationBlock) - sizeof(uint16_t));

  updateBlock(api, targetAddr, p, p + sizeof(AdaptationBlock)); // wear-reducing conditional write, from storage_api.h

  AdaptationBlock verify;
  bool ok = loadAndValidate(targetAddr, verify);
  if (ok) {
    update(api, ADAPT_POINTER_ADDR, newPointerVal);
    g_dirty = false;
  }
  return ok;
}

void adaptation_clearAll(void) {
  zeroBlock(g_active);
  memset(g_fuelHitCounter, 0, sizeof(g_fuelHitCounter));
  memset(g_fuelHitSign, 0, sizeof(g_fuelHitSign));
  memset(g_idleHitCounter, 0, sizeof(g_idleHitCounter));
  memset(g_idleHitSign, 0, sizeof(g_idleHitSign));
  g_dirty = true;
  adaptation_onIgnitionOff();
}

// ============================================================================
// Steady-state detection
// ============================================================================

static bool isSteadyState(uint16_t rpm, uint16_t load) {
  g_rpmHist[g_histIdx] = rpm;
  g_loadHist[g_histIdx] = load;
  g_histIdx = (g_histIdx + 1) % STEADY_WINDOW;
  if (g_histFilled < STEADY_WINDOW) { g_histFilled++; return false; }

  uint16_t rpmMin = 0xFFFF, rpmMax = 0;
  uint16_t loadMin = 0xFFFF, loadMax = 0;
  for (uint8_t i = 0; i < STEADY_WINDOW; i++) {
    if (g_rpmHist[i] < rpmMin) rpmMin = g_rpmHist[i];
    if (g_rpmHist[i] > rpmMax) rpmMax = g_rpmHist[i];
    if (g_loadHist[i] < loadMin) loadMin = g_loadHist[i];
    if (g_loadHist[i] > loadMax) loadMax = g_loadHist[i];
  }
  return ((rpmMax - rpmMin) <= STEADY_RPM_DELTA_MAX) &&
         ((loadMax - loadMin) <= STEADY_LOAD_DELTA_MAX);
}

// ============================================================================
// Real speeducka accessor helpers
// ============================================================================

// currentStatus.RPM < currentStatus.crankRPM approximates decoders.cpp's
// static IsCranking() (which also checks startRevolutions==0, but that's
// not externally visible -- this is a very close approximation and errs
// safe, i.e. slightly wider "cranking" window, not narrower).
static inline bool isCrankingApprox(void) {
  return currentStatus.RPM < currentStatus.crankRPM;
}

static inline bool isClosedLoopEgoEnabled(void) {
  return configPage6.egoAlgorithm != EGO_ALGORITHM_NONE;
}

static inline uint16_t getSelectedLoad(void) {
  LoadSource src = (configPage15.adaptLoadSource != 0U) ? LOAD_SOURCE_TPS : LOAD_SOURCE_MAP;
  return getLoad(src, currentStatus);
}

// ============================================================================
// Fuel: gating + learning + trim lookup
// ============================================================================

static void feedFuelError(uint16_t rpm, uint16_t load) {
  bool steady = isSteadyState(rpm, load);

  bool fuelLearnEnabled = configPage15.adaptLinkFuelToLambdaEn
                              ? isClosedLoopEgoEnabled()
                              : (bool)configPage15.adaptEnableFuelLearn;

  bool inWindow = (currentStatus.coolant >= (int16_t)configPage15.adaptFuelMinCltC) &&
                  (rpm >= configPage15.adaptFuelMinRpm) && (rpm <= configPage15.adaptFuelMaxRpm) &&
                  (load >= configPage15.adaptFuelMinLoad) && (load <= configPage15.adaptFuelMaxLoad);

  bool canLearn = fuelLearnEnabled && inWindow && isClosedLoopEgoEnabled() &&
                   !currentStatus.isDFCOActive && (currentStatus.AEamount <= 100U) &&
                   !isCrankingApprox() && steady;

  g_fuelWindowActive = canLearn;
  if (!canLearn) return;

  // currentStatus.egoCorrection: 100 = neutral, same convention as the rest
  // of speeduino's corrections pipeline. Convert to a signed tenths-of-percent
  // error relative to that baseline.
  int16_t err_x10 = ((int16_t)currentStatus.egoCorrection - 100) * 10;

  uint8_t rpmIdx, rpmFrac, loadIdx, loadFrac;
  findBinU16(rpm, ltftRpmBins, LTFT_RPM_BINS, rpmIdx, rpmFrac);
  findBinU16(load, ltftLoadBins, LTFT_LOAD_BINS, loadIdx, loadFrac);
  uint8_t rIdx = (rpmFrac > 127) ? (rpmIdx + 1) : rpmIdx;
  uint8_t lIdx = (loadFrac > 127) ? (loadIdx + 1) : loadIdx;
  if (rIdx >= LTFT_RPM_BINS) rIdx = LTFT_RPM_BINS - 1;
  if (lIdx >= LTFT_LOAD_BINS) lIdx = LTFT_LOAD_BINS - 1;

  int8_t sign = (err_x10 > 20) ? 1 : (err_x10 < -20) ? -1 : 0; // 2.0% deadband
  if (sign == 0) return;

  if (g_fuelHitSign[rIdx][lIdx] == sign) {
    if (g_fuelHitCounter[rIdx][lIdx] < 255) g_fuelHitCounter[rIdx][lIdx]++;
  } else {
    g_fuelHitSign[rIdx][lIdx] = sign;
    g_fuelHitCounter[rIdx][lIdx] = 1;
  }

  if (g_fuelHitCounter[rIdx][lIdx] >= configPage15.adaptLearnHitThreshold) {
    int16_t cellCounts = g_active.fuelLtft[rIdx][lIdx];
    int16_t errCounts = err_x10 / LTFT_LSB_PCT_X10;
    int16_t updated = cellCounts + ((errCounts - cellCounts) >> LEARN_IIR_SHIFT);
    updated = constrain(updated, -LTFT_MAX_TRIM_COUNTS, LTFT_MAX_TRIM_COUNTS);
    if (updated != cellCounts) {
      g_active.fuelLtft[rIdx][lIdx] = (int8_t)updated;
      g_dirty = true;
    }
    g_fuelHitCounter[rIdx][lIdx] = 0;
  }
}

uint16_t adaptation_getFuelTrimPercent100(void) {
  uint16_t rpm = currentStatus.RPM;
  uint16_t load = getSelectedLoad();

  uint8_t rIdx, rFrac, lIdx, lFrac;
  findBinU16(rpm, ltftRpmBins, LTFT_RPM_BINS, rIdx, rFrac);
  findBinU16(load, ltftLoadBins, LTFT_LOAD_BINS, lIdx, lFrac);

  int16_t c00 = g_active.fuelLtft[rIdx][lIdx];
  int16_t c10 = g_active.fuelLtft[min((uint8_t)(rIdx + 1), (uint8_t)(LTFT_RPM_BINS - 1))][lIdx];
  int16_t c01 = g_active.fuelLtft[rIdx][min((uint8_t)(lIdx + 1), (uint8_t)(LTFT_LOAD_BINS - 1))];
  int16_t c11 = g_active.fuelLtft[min((uint8_t)(rIdx + 1), (uint8_t)(LTFT_RPM_BINS - 1))][min((uint8_t)(lIdx + 1), (uint8_t)(LTFT_LOAD_BINS - 1))];

  int16_t top = c00 + (((int32_t)(c10 - c00) * rFrac) >> 8);
  int16_t bot = c01 + (((int32_t)(c11 - c01) * rFrac) >> 8);
  int16_t ltftCounts = top + (((int32_t)(bot - top) * lFrac) >> 8);

  int16_t ltft_x10 = ltftCounts * LTFT_LSB_PCT_X10;
  g_liveFuelLtft_x10 = ltft_x10;

  // Convert tenths-of-percent offset to speeduino's 100=neutral percent convention
  int32_t pct100 = 100 + (ltft_x10 / 10);
  return (uint16_t)constrain(pct100, 50, 150);
}

// ============================================================================
// Idle: gating + learning + trim lookup
// ============================================================================

static void feedIdleError(void) {
  if (!currentStatus.idleOn || isCrankingApprox()) { g_idleWindowActive = false; return; }
  if (!configPage15.adaptEnableIdleLearn) { g_idleWindowActive = false; return; }
  if (currentStatus.coolant < (int16_t)configPage15.adaptIdleMinCltC) { g_idleWindowActive = false; return; }
  g_idleWindowActive = true;

  // Target RPM in closed-loop idle modes; CLIdleTarget is stored /10.
  int16_t targetRpm = (int16_t)currentStatus.CLIdleTarget * 10;
  int16_t rpmError = (int16_t)currentStatus.RPM - targetRpm;

  // egoCorrection-style STFT doesn't exist for idle in speeduino's own code --
  // the idle PID (idlePID in idle.cpp) already closes this loop. We treat
  // sustained RPM error itself (not a separate fast trim) as the learning
  // signal, since there's no equivalent to egoCorrection to read here.
  bool sparkOnly = (g_liveIdleIgn_x10 > -IDLE_IGN_MAX_COUNTS && g_liveIdleIgn_x10 < IDLE_IGN_MAX_COUNTS);

  uint8_t idx, frac;
  findBinI16((int16_t)currentStatus.coolant, idleCltBins, IDLE_CLT_BINS, idx, frac);
  uint8_t cellIdx = (frac > 127) ? min((uint8_t)(idx + 1), (uint8_t)(IDLE_CLT_BINS - 1)) : idx;

  int16_t deadband = (int16_t)configPage15.adaptIdleRpmDeadband;
  int8_t sign = (rpmError > deadband) ? 1 : (rpmError < -deadband) ? -1 : 0;
  if (sign == 0) return;

  if (g_idleHitSign[cellIdx] == sign) {
    if (g_idleHitCounter[cellIdx] < 255) g_idleHitCounter[cellIdx]++;
  } else {
    g_idleHitSign[cellIdx] = sign;
    g_idleHitCounter[cellIdx] = 1;
  }

  if (g_idleHitCounter[cellIdx] >= configPage15.adaptLearnHitThreshold) {
    if (sparkOnly) {
      int16_t cellCounts = g_active.idleIgnTrim[cellIdx];
      int16_t nudge = (sign > 0) ? 2 : -2;
      int16_t updated = constrain(cellCounts + nudge, -IDLE_IGN_MAX_COUNTS, IDLE_IGN_MAX_COUNTS);
      if (updated != cellCounts) { g_active.idleIgnTrim[cellIdx] = (int8_t)updated; g_dirty = true; }
    } else {
      int16_t cellCounts = g_active.idleAirBase[cellIdx];
      int16_t nudge = (sign > 0) ? 1 : -1;
      int16_t updated = constrain(cellCounts + nudge, -IDLE_AIR_MAX_DELTA, IDLE_AIR_MAX_DELTA);
      if (updated != cellCounts) { g_active.idleAirBase[cellIdx] = updated; g_dirty = true; }
    }
    g_idleHitCounter[cellIdx] = 0;
  }
}

int16_t adaptation_getIdleAirDelta(void) {
  uint8_t idx, frac;
  findBinI16((int16_t)currentStatus.coolant, idleCltBins, IDLE_CLT_BINS, idx, frac);
  int16_t lo = g_active.idleAirBase[idx];
  int16_t hi = g_active.idleAirBase[min((uint8_t)(idx + 1), (uint8_t)(IDLE_CLT_BINS - 1))];
  int16_t interp = lo + (((int32_t)(hi - lo) * frac) >> 8);
  g_liveIdleAirTrim = interp;
  return interp;
}

int16_t adaptation_getIdleIgnTrim_x10(void) {
  uint8_t idx, frac;
  findBinI16((int16_t)currentStatus.coolant, idleCltBins, IDLE_CLT_BINS, idx, frac);
  int16_t lo = g_active.idleIgnTrim[idx];
  int16_t hi = g_active.idleIgnTrim[min((uint8_t)(idx + 1), (uint8_t)(IDLE_CLT_BINS - 1))];
  int16_t interp = lo + (((int32_t)(hi - lo) * frac) >> 8);
  int16_t result = interp * IDLE_IGN_LSB_DEG_X10;
  g_liveIdleIgn_x10 = result;
  return result;
}

// ============================================================================
// Periodic update
// ============================================================================

void adaptation_update(void) {
  uint32_t now = millis();
  if ((uint32_t)(now - g_lastUpdateMs) < ADAPT_UPDATE_INTERVAL_MS) return;
  g_lastUpdateMs = now;

  feedFuelError(currentStatus.RPM, getSelectedLoad());
  feedIdleError();
}

// ============================================================================
// TunerStudio output channels
// ============================================================================

void adaptation_fillOutputChannels(AdaptationOutputChannels &out) {
  out.fuelLtft_x10 = g_liveFuelLtft_x10;
  out.fuelStft_x10 = ((int16_t)currentStatus.egoCorrection - 100) * 10;
  out.idleAirTrim  = g_liveIdleAirTrim;
  out.idleIgn_x10  = g_liveIdleIgn_x10;
  out.adaptFlags   = (g_dirty ? 0x01 : 0x00)
                    | (g_fuelWindowActive ? 0x02 : 0x00)
                    | (g_idleWindowActive ? 0x04 : 0x00)
                    | (configPage15.adaptEnableFuelLearn ? 0x08 : 0x00)
                    | (configPage15.adaptEnableIdleLearn ? 0x10 : 0x00)
                    | (configPage15.adaptLoadSource ? 0x20 : 0x00)
                    | (powerLatch_isShuttingDown() ? 0x40 : 0x00);
}

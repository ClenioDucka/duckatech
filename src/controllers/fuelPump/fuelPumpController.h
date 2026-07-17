#pragma once

#include "../../../config_pages.h"
#include "../../../statuses.h"

void initialiseFuelPump(const statuses &current, const config2 &page2, uint8_t pumpPin);

void startPumpPriming(const statuses &current, const config2 &page2);
void stopPumpPriming(const statuses &current, const config2 &page2);

void fuelPumpOn(void);
void fuelPumpOff(void);
bool isFuelPumpOn(void); ///< Query the pump's currently commanded state -- added for the self-adaptation power-latch shutdown sequence, see powerLatch.h

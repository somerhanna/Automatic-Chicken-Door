#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <Arduino.h>

// Time sync
extern bool timeInitialized;
extern int currentDay;

bool syncTimeWithNTP();
void disconnectWiFi();
void printTime();

#endif // TIME_SYNC_H

#ifndef SCHEDULE_H
#define SCHEDULE_H
#include <freertos/FreeRTOS.h>
#include <Arduino.h>

// =====================================================
// Schedule Times
// Stored internally as 24-hour values
// =====================================================
extern int openHour;
extern int openMinute;
extern int closeHour;
extern int closeMinute;
extern bool scheduleEnabled;
extern portMUX_TYPE scheduleMux;

// Track last executed day-of-year
extern int lastExecutedOpenDay;
extern int lastExecutedCloseDay;

bool isValidTimeValue(int hour, int minute);

void loadScheduleFromPreferences();
void saveScheduleToPreferences();
void resetScheduleToDefaults();

void checkSchedule();

#endif // SCHEDULE_H

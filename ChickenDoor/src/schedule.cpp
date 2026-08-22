#include "schedule.h"
#include <Preferences.h>
#include <time.h>
#include "motor.h"
#include "display.h"
#include "time_sync.h"
#include "utils.h"
#include <freertos/FreeRTOS.h>

// =====================================================
// Persistent Storage
// =====================================================
Preferences preferences;
const char* PREF_NAMESPACE = "chicken_door";

// Mutex for schedule variables to ensure atomic updates
portMUX_TYPE scheduleMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// Schedule Times
// Stored internally as 24-hour values
// =====================================================
int openHour = 10;
int openMinute = 27;
int closeHour = 18;
int closeMinute = 0;
bool scheduleEnabled = true;

// Track last executed day-of-year
int lastExecutedOpenDay = -1;
int lastExecutedCloseDay = -1;

// =====================================================
// Utility
// =====================================================
bool isValidTimeValue(int hour, int minute) {
  return (hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59);
}

// =====================================================
// Persistent Storage Functions
// =====================================================
void loadScheduleFromPreferences() {
  preferences.begin(PREF_NAMESPACE, false);

  openHour = preferences.getInt("openHour", 10);
  openMinute = preferences.getInt("openMinute", 27);
  closeHour = preferences.getInt("closeHour", 18);
  closeMinute = preferences.getInt("closeMinute", 0);
  scheduleEnabled = preferences.getBool("scheduleEnabled", true);

  // Validate loaded values
  if (!isValidTimeValue(openHour, openMinute)) {
    openHour = 10;
    openMinute = 27;
  }
  if (!isValidTimeValue(closeHour, closeMinute)) {
    closeHour = 18;
    closeMinute = 0;
  }

  preferences.end();

  Serial.println("Schedule loaded from flash memory:");
  Serial.printf("  Open: %s\n", formatTime12Hour(openHour, openMinute).c_str());
  Serial.printf("  Close: %s\n", formatTime12Hour(closeHour, closeMinute).c_str());
  Serial.printf("  Enabled: %s\n", scheduleEnabled ? "Yes" : "No");
}

void saveScheduleToPreferences() {
  preferences.begin(PREF_NAMESPACE, false);

  preferences.putInt("openHour", openHour);
  preferences.putInt("openMinute", openMinute);
  preferences.putInt("closeHour", closeHour);
  preferences.putInt("closeMinute", closeMinute);
  preferences.putBool("scheduleEnabled", scheduleEnabled);

  preferences.end();

  Serial.println("Schedule saved to flash memory");
}

void resetScheduleToDefaults() {
  openHour = 10;
  openMinute = 27;
  closeHour = 18;
  closeMinute = 0;
  scheduleEnabled = true;
  saveScheduleToPreferences();
  Serial.println("Schedule reset to defaults");
}

// =====================================================
// Time / Schedule
// =====================================================
void checkSchedule() {
  int localOpenHour, localOpenMinute, localCloseHour, localCloseMinute;
  bool localEnabled;
  
  portENTER_CRITICAL(&scheduleMux);
  localEnabled = scheduleEnabled;
  localOpenHour = openHour;
  localOpenMinute = openMinute;
  localCloseHour = closeHour;
  localCloseMinute = closeMinute;
  portEXIT_CRITICAL(&scheduleMux);
  
  // Now use ONLY the local copies for the rest of the function
  if (!localEnabled) return;
  if (!timeInitialized) return;
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  int currentHour = timeinfo.tm_hour;
  int currentMinute = timeinfo.tm_min;
  int currentDayOfYear = timeinfo.tm_yday;

  if (currentDayOfYear != currentDay) {
    if (currentState != MOTOR_IDLE) {
      return;   // re-checked every loop iteration; will fire once motor goes idle; // Don't resync mid-move: the resync blocks the loop, and stopping BLE here
    }
    currentDay = currentDayOfYear;
    Serial.println("New day detected - resyncing time..."); 
    timeInitialized = syncTimeWithNTP();
    if (!timeInitialized) {
      setDisplayMode(DISPLAY_NO_TIME, 2500);
    }
    updateDisplay(true);
    return;
  }
  
  if (currentDayOfYear != currentDay) {
    if (currentState != MOTOR_IDLE) {
    return;}   // re-checked every loop iteration; will fire once motor goes idle; // Don't resync mid-move: the resync blocks the loop, and stopping BLE here
    currentDay = currentDayOfYear;
    Serial.println("New day detected - resyncing time..."); 
    timeInitialized = syncTimeWithNTP();
    if (!timeInitialized) {
      setDisplayMode(DISPLAY_NO_TIME, 2500);
    }
    updateDisplay(true);
    return;
  }

  int currentTotalMinutes = currentHour * 60 + currentMinute;
  int openTotalMinutes = localOpenHour * 60 + localOpenMinute;
  int closeTotalMinutes = localCloseHour * 60 + localCloseMinute;

  if (currentTotalMinutes == openTotalMinutes) {
    bool alreadyExecutedToday = (lastExecutedOpenDay == currentDayOfYear);

    if (!alreadyExecutedToday) {
      if (!limitTopActive) {
        Serial.printf("SCHEDULE: Opening door at %s\n",
                      formatTime12Hour(currentHour, currentMinute).c_str());
        setMotorState(MOTOR_FORWARD);
      } else {
        Serial.println("Door already open - marking as executed");
        setDisplayMode(DISPLAY_LIMIT_TOP, 1200);
      }
      lastExecutedOpenDay = currentDayOfYear;
    }
  }

  if (currentTotalMinutes == closeTotalMinutes) {
    bool alreadyExecutedToday = (lastExecutedCloseDay == currentDayOfYear);

    if (!alreadyExecutedToday) {
      if (!limitBottomActive) {
        Serial.printf("SCHEDULE: Closing door at %s\n",
                      formatTime12Hour(currentHour, currentMinute).c_str());
        setMotorState(MOTOR_REVERSE);
      } else {
        Serial.println("Door already closed - marking as executed");
        setDisplayMode(DISPLAY_LIMIT_BOTTOM, 1200);
      }
      lastExecutedCloseDay = currentDayOfYear;
    }
  }
}

#include "utils.h"
#include <time.h>
#include "motor.h"

// =====================================================
// Utility
// =====================================================
String twoDigit(int value) {
  if (value < 10) return "0" + String(value);
  return String(value);
}

String formatTime12Hour(int hour24, int minute) {
  String suffix = (hour24 >= 12) ? "PM" : "AM";
  int hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;
  return String(hour12) + ":" + twoDigit(minute) + " " + suffix;
}

String getMotorStateText() {
  switch (currentState) {
    case MOTOR_IDLE: return "IDLE";
    case MOTOR_FORWARD: return "OPENING";
    case MOTOR_REVERSE: return "CLOSING";
  }
  return "UNKNOWN";
}

String getCurrentTimeMainString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--";

  int hour24 = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;

  int hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;

  return String(hour12) + ":" + twoDigit(minute);
}

String getCurrentTimeSuffix() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--";

  return (timeinfo.tm_hour >= 12) ? "PM" : "AM";
}

String getCurrentDateString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--/--/--";

  int month = timeinfo.tm_mon + 1;
  int day = timeinfo.tm_mday;
  int year2 = (timeinfo.tm_year + 1900) % 100;

  return twoDigit(month) + "/" + twoDigit(day) + "/" + twoDigit(year2);
}

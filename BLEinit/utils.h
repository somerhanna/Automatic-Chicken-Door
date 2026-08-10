#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

String getMotorStateText();
String twoDigit(int value);
String formatTime12Hour(int hour24, int minute);
String getCurrentTimeMainString();
String getCurrentTimeSuffix();
String getCurrentDateString();

#endif // UTILS_H

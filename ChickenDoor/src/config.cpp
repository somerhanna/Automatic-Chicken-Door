#include "config.h"

// =====================================================
// WiFi Credentials
// =====================================================
const char* ssid = "Fairuz";
const char* password = "ILoveMyKids1!";

// =====================================================
// NTP Server Settings
// NOTE: Pacific Time with DST
// =====================================================
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -28800;
const int daylightOffset_sec = 3600;

// =====================================================
// Motor Timing
// =====================================================
const unsigned long MOTOR_RUN_TIME_MS = 4000;

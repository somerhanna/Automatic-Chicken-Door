#include "time_sync.h"
#include <WiFi.h>
#include <time.h>
#include "config.h"
#include "display.h"

// Time sync
bool timeInitialized = false;
int currentDay = -1;

// =====================================================
// WiFi / Time
// =====================================================
void disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disconnected - running on BLE only");
}

bool syncTimeWithNTP() {
  Serial.println("Connecting to WiFi for time sync...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
    updateDisplay(true);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed - time sync skipped");
    disconnectWiFi();
    return false;
  }

  Serial.println("\nWiFi connected!");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Syncing time with NTP...");

  struct tm timeinfo;
  int retry = 0;
  bool timeSynced = false;

  while (!getLocalTime(&timeinfo) && retry < 8) {
    delay(1000);
    Serial.print(".");
    retry++;
    updateDisplay(true);
  }

  if (getLocalTime(&timeinfo)) {
    Serial.println("\nTime synchronized!");
    printTime();
    timeSynced = true;
    currentDay = timeinfo.tm_yday;
  } else {
    Serial.println("\nTime sync failed");
    timeSynced = false;
  }

  disconnectWiFi();
  return timeSynced;
}

void printTime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    int hour24 = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;
    int second = timeinfo.tm_sec;

    String suffix = (hour24 >= 12) ? "PM" : "AM";
    int hour12 = hour24 % 12;
    if (hour12 == 0) hour12 = 12;

    Serial.printf("Current time: %d:%02d:%02d %s\n",
                  hour12, minute, second, suffix.c_str());
  } else {
    Serial.println("Failed to obtain time");
  }
}

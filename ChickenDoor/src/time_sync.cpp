#include "time_sync.h"
#include <WiFi.h>
#include <time.h>
#include "config.h"
#include "display.h"
#include <esp_wifi.h>
#include "ble_comm.h"

// Time sync
bool timeInitialized = false;
int currentDay = -1;
bool timeSyncInProgress = false;

// =====================================================
// WiFi / Time
// =====================================================
void disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  Serial.println("WiFi disconnected - running on BLE only");
}

bool syncTimeWithNTP() {
  Serial.println("Connecting to WiFi for time sync from NTP...");

  delay(400);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 12) {
    delay(300);
    Serial.print(".");
    attempts++;
  }

  updateDisplay(true);

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
    delay(5000);
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
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
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

bool performSafeTimeSync() {
  timeSyncInProgress = true;

  stopBLE();      // radio must be free of BLE before WiFi comes up
  bool result = syncTimeWithNTP();
  resumeBLE();

  timeSyncInProgress = false;
  return result;
}
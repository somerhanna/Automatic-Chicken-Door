#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>

#include "config.h"
#include "utils.h"
#include "motor.h"
#include "display.h"
#include "schedule.h"
#include "time_sync.h"
#include "ble_comm.h"
#include "serial_commands.h"

void setup() {

  init_Pins(); //Serial begin & Pin initialization

  loadScheduleFromPreferences(); // Load saved opening and closing time schedule from flash
  
  timeInitialized = syncTimeWithNTP();  // Sync time with NTP server and set timeInitialized flag

  initOLED(); // Initialize the SSD1309 OLED display

  if (!timeInitialized) {
  handleTimeInitFailure();
  }

  printInitStatus();  // Serial print info about the limit switches and motor run time

  printScheduleStatus();  // Serial print the current schedule status

  delay(500); // Wait a moment before starting BLE to ensure we do not spike current on power-up

  initBLE(); // Initialize BLE server and characteristics

  setDisplayMode(DISPLAY_NORMAL);
  
  updateDisplay(true);
}

// =====================================================
// Loop
// =====================================================
void loop() {
  checkLimitSwitches();
  updateMotor();
  handleSerialCommands();
  checkSchedule();

  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Disconnect edge detected");
    restartAdvertisingRequested = true;
    oldDeviceConnected = deviceConnected;
    updateDisplay(true);
  }
  else if (deviceConnected && !oldDeviceConnected) {
    Serial.println("Connect edge detected");
    oldDeviceConnected = deviceConnected;
    updateDisplay(true);
  }

  if (!deviceConnected && restartAdvertisingRequested) {
    delay(200);
    startAdvertising(true);
    restartAdvertisingRequested = false;
    updateDisplay(true);
  }

  if (!deviceConnected &&
      !advertisingActive &&
      (millis() - lastAdvertisingStart > ADVERTISING_RECOVERY_INTERVAL_MS)) {
    startAdvertising(false);
    updateDisplay(true);
  }

  updateDisplay();
  delay(10);
}

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
  timeInitialized = syncTimeWithNTP();  // Connect to WiFi. Sync time with NTP server and set timeInitialized flag

  initOLED(timeInitialized); // Initialize the SSD1309 OLED display

  printInitStatus();  // Serial print info about the limit switches and motor run time
  printScheduleStatus();  // Serial print the current schedule status

  delay(500); // Wait a moment before starting BLE to ensure we do not spike current on power-up
  initBLE(); // Initialize BLE server and characteristics

  setDisplayMode(DISPLAY_NORMAL); //
  updateDisplay(true);
}

void loop() {
  checkLimitSwitches(); // polling to see if the limit switches are active and update the state accordingly
  updateMotor();  // move the motor if the requested state is different from the current state and handle timing out if the motor runs too long
  handleSerialCommands(); // allow user input via serial console to control the motor and check status
  checkSchedule(); // check if the current time matches the scheduled open or close time and execute the action if it does
  delay(10);

  handleBLEConnection(); // handle BLE connection and disconnection events
  handleBLEAdvertising(); // handle BLE advertising and restart advertising if needed
  delay(10);

  updateDisplay(); // update the OLED display based on the current display mode and any changes that have occurred
  delay(10);
}

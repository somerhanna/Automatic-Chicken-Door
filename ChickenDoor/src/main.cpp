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

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(LIMIT_SWITCH_TOP, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_BOTTOM, INPUT_PULLUP);

  Serial.println("\n==========================================");
  Serial.println("Chicken Door Motor Controller v5.4 Persistent Storage");
  Serial.println("==========================================");

  // Load saved schedule from flash
  loadScheduleFromPreferences();

  Serial.println("Getting initial time from NTP...");
  timeInitialized = syncTimeWithNTP();

    initOLED();

  if (!timeInitialized) {
    Serial.println("Could not get initial time - schedule disabled");
    scheduleEnabled = false;
    setDisplayMode(DISPLAY_NO_TIME, 2500);
  }

  Serial.println("\nLimit switches (NC type):");
  Serial.println("  - Top switch: GPIO19 (stops OPEN)");
  Serial.println("  - Bottom switch: GPIO18 (stops CLOSE)");
  Serial.printf("\nMotor will run for maximum %lu seconds\n", MOTOR_RUN_TIME_MS / 1000);

  Serial.printf("Schedule: %s\n", scheduleEnabled ? "ENABLED" : "DISABLED");
  if (scheduleEnabled) {
    Serial.printf("  Open at: %s\n", formatTime12Hour(openHour, openMinute).c_str());
    Serial.printf("  Close at: %s\n", formatTime12Hour(closeHour, closeMinute).c_str());
    Serial.println("  Time will resync automatically once per day");
  }

  delay(500);

  BLEDevice::init("Chicken-Door");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pMotorCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pMotorCharacteristic->setCallbacks(new MotorCallbacks());
  pMotorCharacteristic->addDescriptor(new BLE2902());

  pScheduleCharacteristic = pService->createCharacteristic(
    SCHEDULE_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pScheduleCharacteristic->setCallbacks(new ScheduleCallbacks());
  pScheduleCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  startAdvertising(true);

  Serial.println("\nBLE Ready - Device: Chicken-Door");
  Serial.println("App can now connect via Bluetooth");
  Serial.println("SSD1309 OLED active on SDA=21, SCL=22");
  Serial.println("Schedule is saved to flash memory (persists after reboot)");
  Serial.println("==========================================\n");

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

#include "ble_comm.h"
#include <string>
#include "config.h"
#include "motor.h"
#include "display.h"
#include "schedule.h"
#include "utils.h"

//commit from VSCode

// BLE connection state
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool restartAdvertisingRequested = false;

// Advertising state/logging control
bool advertisingActive = false;
bool advertisingLoggedOnce = false;
unsigned long lastAdvertisingStart = 0;
const unsigned long ADVERTISING_RECOVERY_INTERVAL_MS = 3000;

// Status debounce
unsigned long lastStatusSend = 0;
const unsigned long STATUS_DEBOUNCE_MS = 100;

// BLE objects
BLEServer *pServer = nullptr;
BLECharacteristic *pMotorCharacteristic = nullptr;
BLECharacteristic *pScheduleCharacteristic = nullptr;

// =====================================================
// BLE Callbacks
// =====================================================
void MotorCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  String value = String(pCharacteristic->getValue().c_str());
  value.trim();
  value.toLowerCase();

  if (value.length() == 0) return;

  Serial.print("Command received: ");
  Serial.println(value);

  if (value == "forward") {
    setMotorState(MOTOR_FORWARD);
  }
  else if (value == "reverse") {
    setMotorState(MOTOR_REVERSE);
  }
  else if (value == "stop") {
    setMotorState(MOTOR_IDLE);
  }
  else if (value == "status") {
    sendStatus(true);
    updateDisplay(true);
  }
}

void ScheduleCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  String value = String(pCharacteristic->getValue().c_str());
  value.trim();

  if (value.length() == 0) return;

  Serial.print("Schedule received: ");
  Serial.println(value);

  // Format: "openHour,openMinute,closeHour,closeMinute,enabled"
  int comma1 = value.indexOf(',');
  int comma2 = value.indexOf(',', comma1 + 1);
  int comma3 = value.indexOf(',', comma2 + 1);
  int comma4 = value.indexOf(',', comma3 + 1);

  if (comma1 <= 0 || comma2 <= 0 || comma3 <= 0 || comma4 <= 0) {
    Serial.println("Invalid schedule format");
    return;
  }

  int newOpenHour = value.substring(0, comma1).toInt();
  int newOpenMinute = value.substring(comma1 + 1, comma2).toInt();
  int newCloseHour = value.substring(comma2 + 1, comma3).toInt();
  int newCloseMinute = value.substring(comma3 + 1, comma4).toInt();
  bool newEnabled = value.substring(comma4 + 1).toInt() == 1;

  if (!isValidTimeValue(newOpenHour, newOpenMinute) ||
      !isValidTimeValue(newCloseHour, newCloseMinute)) {
    Serial.println("Invalid schedule time values");
    return;
  }

  bool openTimeChanged = (newOpenHour != openHour || newOpenMinute != openMinute);
  bool closeTimeChanged = (newCloseHour != closeHour || newCloseMinute != closeMinute);

  openHour = newOpenHour;
  openMinute = newOpenMinute;
  closeHour = newCloseHour;
  closeMinute = newCloseMinute;
  scheduleEnabled = newEnabled;

  // Save to persistent storage
  saveScheduleToPreferences();

  if (openTimeChanged) {
    lastExecutedOpenDay = -1;
    Serial.println("Open time changed - resetting execution flag");
  }

  if (closeTimeChanged) {
    lastExecutedCloseDay = -1;
    Serial.println("Close time changed - resetting execution flag");
  }

  Serial.println("Schedule updated and saved to flash:");
  Serial.printf("  Open: %s\n", formatTime12Hour(openHour, openMinute).c_str());
  Serial.printf("  Close: %s\n", formatTime12Hour(closeHour, closeMinute).c_str());
  Serial.printf("  Enabled: %s\n", scheduleEnabled ? "Yes" : "No");

  sendSchedule();
  sendStatus(true);
  updateDisplay(true);
}

void ScheduleCallbacks::onRead(BLECharacteristic *pCharacteristic) {
  sendSchedule();
}

void MyServerCallbacks::onConnect(BLEServer* pServer) {
  deviceConnected = true;
  advertisingActive = false;
  Serial.println("Client connected!");
  sendStatus(true);
  sendSchedule();
  updateDisplay(true);
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  advertisingActive = false;
  restartAdvertisingRequested = true;
  Serial.println("Client disconnected");
  updateDisplay(true);
}

// =====================================================
// BLE Advertising zzzz
// =====================================================
void startAdvertising(bool logMessage) {
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->stop();
  delay(50);

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  advertisingActive = true;
  lastAdvertisingStart = millis();

  if (logMessage) {
    Serial.println("BLE advertising started");
    advertisingLoggedOnce = true;
  }
}

// =====================================================
// BLE Notify
// =====================================================
void sendStatus(bool force) {
  if (!deviceConnected || pMotorCharacteristic == nullptr) return;

  unsigned long now = millis();
  if (!force && (now - lastStatusSend < STATUS_DEBOUNCE_MS)) return;
  lastStatusSend = now;

  String status = "STATUS:";
  switch (currentState) {
    case MOTOR_IDLE: status += "IDLE"; break;
    case MOTOR_FORWARD: status += "FORWARD"; break;
    case MOTOR_REVERSE: status += "REVERSE"; break;
  }

  status += ",TOP:";
  status += limitTopActive ? "ACTIVE" : "OK";
  status += ",BOTTOM:";
  status += limitBottomActive ? "ACTIVE" : "OK";

  pMotorCharacteristic->setValue(std::string(status.c_str()));
  pMotorCharacteristic->notify();

  Serial.println("Status sent: " + status);
}

void sendSchedule() {
  if (!deviceConnected || pScheduleCharacteristic == nullptr) return;

  String scheduleStr = String(openHour) + "," +
                       String(openMinute) + "," +
                       String(closeHour) + "," +
                       String(closeMinute) + "," +
                       String(scheduleEnabled ? 1 : 0);

  pScheduleCharacteristic->setValue(std::string(scheduleStr.c_str()));
  pScheduleCharacteristic->notify();

  Serial.println("Schedule sent: " + scheduleStr);
}
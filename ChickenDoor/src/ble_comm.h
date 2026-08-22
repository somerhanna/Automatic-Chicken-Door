#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>

// BLE connection state

extern bool deviceConnected;
extern bool oldDeviceConnected;
extern bool restartAdvertisingRequested;

// Advertising state/logging control
extern bool advertisingActive;
extern bool advertisingLoggedOnce;
extern unsigned long lastAdvertisingStart;
extern const unsigned long ADVERTISING_RECOVERY_INTERVAL_MS;

// Status debounce
extern unsigned long lastStatusSend;
extern const unsigned long STATUS_DEBOUNCE_MS;

// BLE objects
extern BLEServer *pServer;
extern BLECharacteristic *pMotorCharacteristic;
extern BLECharacteristic *pScheduleCharacteristic;

//BLE pause/resume to prevent WiFi coexisting with it during daily nightly WiFi time sync
extern bool bleTemporarilyStopped;
void stopBLE();
void resumeBLE();

// BLE service and characteristic declarations
void sendStatus(bool force = false);
void sendSchedule();
void startAdvertising(bool logMessage);
void handleBLEConnection();
void handleBLEAdvertising();
void initBLE();

// BLE callback classes
class MotorCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
};

class ScheduleCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override;
  void onRead(BLECharacteristic *pCharacteristic) override;
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;
};

#endif // BLE_COMM_H

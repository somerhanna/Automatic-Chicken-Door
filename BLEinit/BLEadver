#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// -------- BLE UUIDs --------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// -------- Limit switch --------
constexpr int LIMIT_PIN = 18;
constexpr bool PRESSED_LEVEL = LOW;

// -------- Shared state --------
static int32_t g_timeOffsetSeconds = 0;
static bool deviceConnected = false;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// ========== BLE CALLBACKS ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✅ iPhone Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("❌ iPhone Disconnected");
      pServer->startAdvertising();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      
      if (value.length() > 0) {
        Serial.print("📱 Command: ");
        Serial.println(value);
        
        if (value == "add_hour") {
          g_timeOffsetSeconds += 3600;
          Serial.print("  New offset: ");
          Serial.print(g_timeOffsetSeconds / 3600);
          Serial.println(" hours");
        }
        else if (value == "add_30min") {
          g_timeOffsetSeconds += 1800;
        }
        else if (value == "add_15min") {
          g_timeOffsetSeconds += 900;
        }
      }
    }
};

// ========== LIMIT SWITCH TASK ==========
void limitSwitchTask(void* pv) {
  bool lastPressed = false;
  uint32_t lastChangeMs = 0;
  uint32_t bootTime = millis();

  for (;;) {
    bool pressedNow = (digitalRead(LIMIT_PIN) == PRESSED_LEVEL);
    uint32_t nowMs = millis();
    
    if (pressedNow != lastPressed && (nowMs - lastChangeMs) > 50) {
      lastChangeMs = nowMs;
      lastPressed = pressedNow;
      
      if (pressedNow && (nowMs - bootTime > 3000)) {
        g_timeOffsetSeconds += 3600;
        Serial.println("🔘 Switch: +1 hour");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n🚀 MINIMAL CHICKEN DOOR");
  Serial.println("========================");
  
  // Limit switch
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  
  // Initialize BLE (minimal)
  Serial.println("Starting BLE...");
  BLEDevice::init("ChickenDoor");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  
  pService->start();
  
  // Simple advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ Advertising as: ChickenDoor");
  Serial.println("📱 Look in LightBlue app now!");
  
  // Create limit switch task
  xTaskCreatePinnedToCore(limitSwitchTask, "Limit", 2048, NULL, 1, NULL, 0);
}

void loop() {
  delay(1000);
}
/*
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Use NimBLE instead of default BLE library
#include <NimBLEDevice.h>

// -------- OLED --------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// -------- WiFi / NTP --------
const char* ssid     = "Fairuz";
const char* password = "ILoveMyKids1!";
const char* TZ_INFO = "PST8PDT,M3.2.0/2,M11.1.0/2";

// -------- Limit switch --------
constexpr int LIMIT_PIN = 18;
constexpr bool PRESSED_LEVEL = LOW;

// -------- BLE UUIDs --------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// -------- Shared state --------
static int32_t g_timeOffsetSeconds = 0;
static SemaphoreHandle_t g_timeMutex;
static bool deviceConnected = false;

// Characteristic pointer for sending data back
static NimBLECharacteristic* pCharacteristic = NULL;

// ========== FUNCTION DEFINITIONS ==========
static int32_t getOffsetSeconds() {
  int32_t v;
  xSemaphoreTake(g_timeMutex, portMAX_DELAY);
  v = g_timeOffsetSeconds;
  xSemaphoreGive(g_timeMutex);
  return v;
}

static void addOffsetSeconds(int32_t delta) {
  xSemaphoreTake(g_timeMutex, portMAX_DELAY);
  g_timeOffsetSeconds += delta;
  xSemaphoreGive(g_timeMutex);
}

static void showMessage(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 && line2[0]) display.println(line2);
  display.display();
}

// ========== NIMBLE CALLBACKS ==========
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
      deviceConnected = true;
      Serial.println("\n✅✅✅ IPHONE CONNECTED! ✅✅✅");
      Serial.print("  Connection time (ms): ");
      Serial.println(millis());
      showMessage("iPhone", "Connected!");
    };

    void onDisconnect(NimBLEServer* pServer) {
      deviceConnected = false;
      Serial.println("\n❌❌❌ IPHONE DISCONNECTED! ❌❌❌");
      showMessage("iPhone", "Disconnected");
      pServer->startAdvertising();
      Serial.println("📡 Advertising restarted");
    }
};

class CharCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        Serial.print("\n📱 RECEIVED COMMAND: [");
        Serial.print(value.c_str());
        Serial.println("]");
        
        if (value == "add_hour") {
          addOffsetSeconds(3600);
          Serial.println("  ➡️ Added 1 hour");
        }
        else if (value == "add_30min") {
          addOffsetSeconds(1800);
          Serial.println("  ➡️ Added 30 minutes");
        }
        else if (value == "add_15min") {
          addOffsetSeconds(900);
          Serial.println("  ➡️ Added 15 minutes");
        }
      }
    }
};

// ========== TASKS ==========
void limitSwitchTask(void* pv) {
  // Wait for hardware to stabilize
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  
  bool lastPressed = (digitalRead(LIMIT_PIN) == PRESSED_LEVEL);
  uint32_t lastChangeMs = millis();
  
  // Record when we started
  uint32_t bootTime = millis();

  for (;;) {
    bool pressedNow = (digitalRead(LIMIT_PIN) == PRESSED_LEVEL);
    uint32_t nowMs = millis();
    
    if (pressedNow != lastPressed && (nowMs - lastChangeMs) > 50) {
      lastChangeMs = nowMs;
      lastPressed = pressedNow;
      
      if (pressedNow) {
        if (nowMs - bootTime > 3000) {
          addOffsetSeconds(3600);
          Serial.println("✅ Switch pressed: +1 hour");
        } else {
          Serial.println("⏳ Ignoring press during startup");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void displayTask(void* pv) {
  for (;;) {
    time_t rawNow = time(nullptr);
    int32_t offset = getOffsetSeconds();
    time_t displayTime = rawNow + offset;
    
    struct tm timeinfo;
    localtime_r(&displayTime, &timeinfo);

    display.clearDisplay();

    // Check if time is valid (year >= 2020)
    if (timeinfo.tm_year >= 120) {
      char timeStr[9];
      strftime(timeStr, 9, "%H:%M:%S", &timeinfo);
      
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(timeStr);

      display.setTextSize(1);
      display.setCursor(0, 35);
      if (offset > 0) {
        display.print("+");
        display.print(offset / 3600);
        display.print("h");
      }
      
      if (deviceConnected) {
        display.setCursor(90, 56);
        display.print("BLE");
      }
    } else {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Waiting for");
      display.setCursor(0, 20);
      display.println("time sync...");
      
      if (offset > 0) {
        display.setCursor(0, 40);
        display.print("Offset: +");
        display.print(offset / 3600);
        display.print("h");
      }
    }
    display.display();
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  // Give serial time to initialize
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("🚀 CHICKEN DOOR ESP32 STARTING");
  Serial.println("=================================");
  
  Serial.print("ESP32 Chip Model: ");
  Serial.println(ESP.getChipModel());
  Serial.print("Flash Size: ");
  Serial.println(ESP.getFlashChipSize());
  
  Serial.println("\n📡 Initializing BLE...");
  
  // Initialize BLE with error checking
  try {
    NimBLEDevice::init("ChickenDoor");
    Serial.println("  ✅ BLE Device initialized");
    
    NimBLEServer* pServer = NimBLEDevice::createServer();
    Serial.println("  ✅ BLE Server created");
    
    pServer->setCallbacks(new ServerCallbacks());
    Serial.println("  ✅ Server callbacks set");
    
    NimBLEService* pService = pServer->createService(SERVICE_UUID);
    Serial.println("  ✅ Service created");
    Serial.print("     UUID: ");
    Serial.println(SERVICE_UUID);
    
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        NIMBLE_PROPERTY::WRITE
                      );
    Serial.println("  ✅ Characteristic created");
    Serial.print("     UUID: ");
    Serial.println(CHARACTERISTIC_UUID);
    
    pCharacteristic->setCallbacks(new CharCallbacks());
    Serial.println("  ✅ Characteristic callbacks set");
    
    pService->start();
    Serial.println("  ✅ Service started");
    
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    Serial.println("  ✅ Advertising started");
    
    Serial.println("\n📡 BLE is now advertising as: ChickenDoor");
    Serial.println("   Look for this name in BLE scanner apps");
    
  } catch (const std::exception& e) {
    Serial.print("❌ BLE initialization failed: ");
    Serial.println(e.what());
  } catch (...) {
    Serial.println("❌ Unknown BLE initialization error");
  }

  g_timeMutex = xSemaphoreCreateMutex();

  // OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ Display failed");
    while (true) delay(1000);
  }
  Serial.println("✅ OLED initialized");

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  showMessage("Chicken Door", "BLE Ready");

  // === WIFI (QUICK) ===
  Serial.println("\n📶 Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi connected, getting time...");
    configTzTime(TZ_INFO, "pool.ntp.org");
    struct tm tmp;
    start = millis();
    while (!getLocalTime(&tmp) && millis() - start < 5000) {
      delay(200);
      Serial.print(".");
    }
    Serial.println();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("✅ Time synced, WiFi off");
  } else {
    Serial.println("⚠️ WiFi failed - time not set");
  }

  // Create tasks
  xTaskCreatePinnedToCore(limitSwitchTask, "Limit", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "Display", 2048, NULL, 1, NULL, 1);
  
  Serial.println("\n✅ All tasks started!");
  Serial.println("=================================\n");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
*/

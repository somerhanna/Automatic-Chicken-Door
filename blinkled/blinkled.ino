#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// <-- NEW: Add BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// -------- OLED --------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// -------- WiFi / NTP (only at boot) --------
const char* ssid     = "Fairuz";
const char* password = "ILoveMyKids1!";

// Change timezone string if needed:
const char* TZ_INFO = "PST8PDT,M3.2.0/2,M11.1.0/2";
const char* NTP1 = "pool.ntp.org";
const char* NTP2 = "time.nist.gov";

// -------- Limit switch --------
constexpr int LIMIT_PIN = 18;          // pick any safe GPIO (18 is usually fine)
constexpr bool PRESSED_LEVEL = LOW;    // with INPUT_PULLUP + switch to GND

// <-- NEW: BLE UUIDs - these must match your app!
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// <-- NEW: BLE Server objects
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// <-- NEW: BLE callback classes
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("📱 iPhone Connected!");
      showMessage("iPhone", "Connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("📱 iPhone Disconnected");
      showMessage("iPhone", "Disconnected");
      
      // Restart advertising so iPhone can reconnect
      pServer->startAdvertising();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        Serial.print("📱 Received from iPhone: ");
        Serial.println(value.c_str());
        
        // Parse the command
        if (value == "add_hour") {
          addOffsetSeconds(3600); // +1 hour
          Serial.println("➕ Added 1 hour from iPhone");
        }
        else if (value == "add_30min") {
          addOffsetSeconds(1800); // +30 minutes
          Serial.println("➕ Added 30 minutes from iPhone");
        }
        else if (value == "add_15min") {
          addOffsetSeconds(900); // +15 minutes
          Serial.println("➕ Added 15 minutes from iPhone");
        }
        else if (value == "get_time") {
          // Send current time back to iPhone (optional)
          char timeStr[50];
          time_t now = time(nullptr) + getOffsetSeconds();
          struct tm timeinfo;
          localtime_r(&now, &timeinfo);
          strftime(timeStr, sizeof(timeStr), "%I:%M:%S %p", &timeinfo);
          pCharacteristic->setValue(timeStr);
          pCharacteristic->notify();
        }
      }
    }
};

// -------- Shared state --------
// We add an offset (seconds) when the switch is pressed.
// Accessed from both tasks -> protect with a mutex.
static int32_t g_timeOffsetSeconds = 0;
static SemaphoreHandle_t g_timeMutex;

// Simple helper to safely read the offset
static int32_t getOffsetSeconds() {
  int32_t v;
  xSemaphoreTake(g_timeMutex, portMAX_DELAY);
  v = g_timeOffsetSeconds;
  xSemaphoreGive(g_timeMutex);
  return v;
}

// Safely add to offset
static void addOffsetSeconds(int32_t delta) {
  xSemaphoreTake(g_timeMutex, portMAX_DELAY);
  g_timeOffsetSeconds += delta;
  xSemaphoreGive(g_timeMutex);
}

// -------- Display helpers --------
static void showMessage(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 && line2[0]) display.println(line2);
  display.display();
}

// -------- Task 1: Limit switch watcher --------
void limitSwitchTask(void* pv) {
  (void)pv;

  bool lastPressed = false;
  uint32_t lastChangeMs = 0;

  for (;;) {
    bool pressedNow = (digitalRead(LIMIT_PIN) == PRESSED_LEVEL);

    // Basic debounce: require 50ms stable change
    uint32_t nowMs = millis();
    if (pressedNow != lastPressed && (nowMs - lastChangeMs) > 50) {
      lastChangeMs = nowMs;
      lastPressed = pressedNow;

      // Trigger on press edge (not release)
      if (pressedNow) {
        addOffsetSeconds(3600); // +1 hour
        Serial.println("Limit switch pressed: +1 hour");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// -------- Task 2: OLED time display --------
void displayTask(void* pv) {
  (void)pv;

  for (;;) {
    // Get system epoch time (kept by ESP32 after initial NTP sync)
    time_t now = time(nullptr);

    // Apply our manual offset
    now += getOffsetSeconds();

    // Convert to local time (timezone already configured)
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (now > 100000) { // crude "time seems set" check
      char timeStr[16];
      char dateStr[24];

      strftime(timeStr, sizeof(timeStr), "%I:%M:%S %p", &timeinfo);
      strftime(dateStr, sizeof(dateStr), "%a %b %d, %Y", &timeinfo);

      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(timeStr);

      display.setTextSize(1);
      display.setCursor(0, 40);
      display.println(dateStr);

      display.setCursor(0, 56);
      display.print("+");
      display.print(getOffsetSeconds() / 3600);
      display.print("h ");
      
      // <-- NEW: Show BLE connection status on display
      if (deviceConnected) {
        display.print("📱");
      }
      
    } else {
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Time not set yet...");
    }

    display.display();
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// <-- NEW: BLE Task to handle advertising
void bleTask(void* pv) {
  for (;;) {
    if (!deviceConnected) {
      // Advertise every 5 seconds when not connected
      vTaskDelay(pdMS_TO_TICKS(5000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n🚀 Chicken Door Controller Starting...");

  // Mutex for shared offset
  g_timeMutex = xSemaphoreCreateMutex();

  // I2C + OLED
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed. Try 0x3D or check wiring.");
    while (true) delay(1000);
  }

  // Limit switch pin
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  showMessage("OLED OK", "Starting BLE...");

  // <-- NEW: Initialize BLE
  BLEDevice::init("ChickenDoor"); // This name will appear on your iPhone
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create a Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // helps with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("📱 BLE Started - Waiting for iPhone connection...");
  showMessage("BLE Ready", "iPhone can connect");

  // Connect WiFi (only for initial NTP sync)
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    showMessage("WiFi FAILED", "Time may be wrong");
  } else {
    showMessage("WiFi connected", "Syncing time...");

    // Set timezone + start SNTP
    configTzTime(TZ_INFO, NTP1, NTP2);

    // Wait up to ~15s for time
    struct tm tmp;
    start = millis();
    while (!getLocalTime(&tmp) && millis() - start < 15000) {
      delay(200);
    }

    // Optional: disconnect WiFi after time sync (keeps time locally)
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi disconnected to save power");
  }

  // Create three tasks
  xTaskCreatePinnedToCore(limitSwitchTask, "LimitSwitch", 2048, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask,     "Display",     4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(bleTask,         "BLE",         2048, nullptr, 1, nullptr, 1); // <-- NEW
  
  Serial.println("All tasks started!");
}

void loop() {
  // Nothing here; tasks do the work.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

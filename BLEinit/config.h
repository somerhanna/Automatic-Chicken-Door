#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// WiFi Credentials
// =====================================================
extern const char* ssid;
extern const char* password;

// =====================================================
// NTP Server Settings
// NOTE: Pacific Time with DST
// =====================================================
extern const char* ntpServer;
extern const long gmtOffset_sec;
extern const int daylightOffset_sec;

// =====================================================
// Motor Timing
// =====================================================
extern const unsigned long MOTOR_RUN_TIME_MS;

// =====================================================
// BLE UUIDs
// =====================================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SCHEDULE_UUID       "2a5c6d8e-1fb5-459e-8fcc-c5c9c331914b"

// =====================================================
// Pins
// =====================================================
#define IN1                 26
#define IN2                 27
#define LIMIT_SWITCH_TOP    19
#define LIMIT_SWITCH_BOTTOM 18
#define LED_BUILTIN         2

// OLED I2C pins
#define OLED_SDA            21
#define OLED_SCL            22

#endif // CONFIG_H

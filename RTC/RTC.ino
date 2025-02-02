#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>

#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 18
#define LIMIT_SWITCH_PIN 27
#define LIMIT_SWITCH_PIN2 25

RTC_PCF8523 rtc;

const char* ssid     = "Fairuz";
const char* password = "ILoveMyKids1!";

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 86400000; // 24 hours in milliseconds

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -25200, 60000);

unsigned long motorStartTime = 0;
bool motorRunning = false;
bool isOpeningDoor = false;

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);
  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), handleLimitSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN2), handleLimitSwitch2, FALLING);
  
  Wire.begin(21, 22);
  
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");

  timeClient.begin();
  updateRTCTime();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastUpdate >= updateInterval) {
    updateRTCTime();
  }

  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();

  if (hour == 5 && minute == 0 && !motorRunning) {
    startMotor(false); // Open door (CCW) at 5 AM
    isOpeningDoor = true;
  } else if (hour == 21 && minute == 0 && !motorRunning) {
    startMotor(true); // Close door (CW) at 9 PM
    isOpeningDoor = false;
  }

  if (motorRunning && (millis() - motorStartTime >= 7000)) {
    stopMotor();
  }
}

void startMotor(bool clockwise) {
  if (clockwise) {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  }
  motorStartTime = millis();
  motorRunning = true;
}

void stopMotor() {
  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  motorRunning = false;
}

void handleLimitSwitch() {
  if (!isOpeningDoor) {
    stopMotor();
  }
}

void handleLimitSwitch2() {
  if (isOpeningDoor) {
    stopMotor();
  }
}

void updateRTCTime() {
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    lastUpdate = millis();
  }
}

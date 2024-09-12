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

unsigned long lastUpdate = 0; // To store the time of the last RTC update
const unsigned long updateInterval = 86400000; // 24 hours in milliseconds

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -25200, 60000);

unsigned long motorStartTime = 0; // Track when the motor was started
bool motorRunning = false; // Track if the motor is currently running

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);

  // Initialize motor control pins to LOW (motor off initially)
  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);

  // Set limit switch pins as input with internal pull-up resistors
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN2, INPUT_PULLUP);

  // Attach interrupts to handle limit switch presses
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), handleLimitSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN2), handleLimitSwitch2, FALLING);
  
  Wire.begin(21, 22); // Initialize I2C with SDA on GPIO21 and SCL on GPIO22
  
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

  // Initialize NTP client
  timeClient.begin();
  timeClient.update(); // Get the current time from NTP server

  // Update RTC with current time from NTP
  rtc.adjust(DateTime(timeClient.getEpochTime()));
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if it's time to update RTC
  if (currentMillis - lastUpdate >= updateInterval) {
    timeClient.update(); // Update time from NTP server
    DateTime now = DateTime(timeClient.getEpochTime());

    // Set RTC time
    rtc.adjust(now);
    
    // Update lastUpdate timestamp
    lastUpdate = currentMillis;
  }

  DateTime now = rtc.now();
  int hour = now.hour();
  int min = now.minute();
  int second = now.second();

  // Check if motor should be running based on time
  if (hour == 15 && min == 5 && second == 0) {
    startMotor(true); // Start motor CW
  } else if (hour == 15 && min == 6 && second == 0) {
    startMotor(false); // Start motor CCW
  }

  // Check if the motor should be stopped
  if (motorStartTime > 0) {
    if (millis() - motorStartTime >= 6000) {
      stopMotor();
    }
}}

void startMotor(bool clockwise) {
  if (clockwise) {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  }
  motorStartTime = millis(); // Record the time when the motor starts
}

void stopMotor() {
  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  motorStartTime = 0; // Reset the start time
}

void handleLimitSwitch() {
  stopMotor(); // Stop the motor if limit switch 1 is pressed
}

void handleLimitSwitch2() {
  stopMotor(); // Stop the motor if limit switch 2 is pressed
}
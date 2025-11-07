#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>

#define MOTOR_FORWARD_PIN 5
#define MOTOR_BACKWARD_PIN 18
#define LIMIT_SWITCH_PIN 27   // Closed position switch 
#define LIMIT_SWITCH_PIN2 25  // Open position switch 

RTC_PCF8523 rtc;

const char* ssid = "Fairuz";
const char* password = "ILoveMyKids1!";

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 86400000UL; // 24 hours

WiFiUDP ntpUDP;
// -25200 = UTC-7;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -25200, 60000);

// Motor / motion state
unsigned long motorStartTime = 0;
bool motorRunning = false;
bool isOpeningDoor = false;

// ISR flags (set in ISRs, consumed in loop)
volatile bool limit1Triggered = false;
volatile bool limit2Triggered = false;

// Daily scheduling guards
int lastDayOfYearSeen = -1;
bool openDoneToday = false;
bool closeDoneToday = false;

void IRAM_ATTR handleLimitSwitch() {
    // LIMIT_SWITCH_PIN (closed position). With INPUT_PULLUP + FALLING, LOW means pressed.
    limit1Triggered = true;
}

void IRAM_ATTR handleLimitSwitch2() {
    // LIMIT_SWITCH_PIN2 (open position).
    limit2Triggered = true;
}

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
        while (1) delay(100);
    }

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");

    timeClient.begin();
    updateRTCTime(); // initializes lastUpdate as well

    // Initialize day guards using current RTC
    DateTime now = rtc.now();
    lastDayOfYearSeen = now.dayOfYear();
}

void loop() {
    unsigned long currentMillis = millis();

    // Periodically resync RTC from NTP
    if (currentMillis - lastUpdate >= updateInterval) {
        updateRTCTime();
    }

    DateTime now = rtc.now();

    // Reset daily flags at midnight day change
    if (now.dayOfYear() != lastDayOfYearSeen) {
        lastDayOfYearSeen = now.dayOfYear();
        openDoneToday = false;
        closeDoneToday = false;
    }

    int hour = now.hour();
    int minute = now.minute();

    // ---- Scheduled actions (run once per day) ----
    if (!motorRunning) {
        // OPEN at 06:00, once/day
        if (hour == 6 && minute == 0 && !openDoneToday) {
            isOpeningDoor = true;           // set BEFORE starting motor for ISR logic correctness
            startMotor(/*clockwise=*/false); // CCW for open
            openDoneToday = true;
        }
        // CLOSE at 20:00, once/day
        else if (hour == 20 && minute == 0 && !closeDoneToday) {
            isOpeningDoor = false;          // set BEFORE starting motor
            startMotor(/*clockwise=*/true);  // CW for close
            closeDoneToday = true;
        }
    }

    // ---- Handle limit switches (from ISRs) ----
    if (motorRunning) {
        // Snapshot & clear flags atomically
        bool l1 = limit1Triggered;
        bool l2 = limit2Triggered;
        if (l1) limit1Triggered = false;
        if (l2) limit2Triggered = false;

        // If we are CLOSING (isOpeningDoor == false), stop on LIMIT_SWITCH_PIN (closed switch)
        if (!isOpeningDoor && l1) {
            stopMotor();
        }
        // If we are OPENING (isOpeningDoor == true), stop on LIMIT_SWITCH_PIN2 (open switch)
        if (isOpeningDoor && l2) {
            stopMotor();
        }

        // Safety cut-off at 7s
        if (motorRunning && (millis() - motorStartTime >= 7000UL)) {
            stopMotor();
        }
    }
}

void startMotor(bool clockwise) {
    if (motorRunning) return; // guard

    if (clockwise) {
        // CW
        digitalWrite(MOTOR_FORWARD_PIN, LOW);
        digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    }
    else {
        // CCW
        digitalWrite(MOTOR_FORWARD_PIN, HIGH);
        digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    }
    motorStartTime = millis();
    motorRunning = true;

    // clear any stale ISR flags at start
    limit1Triggered = false;
    limit2Triggered = false;
}

void stopMotor() {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    motorRunning = false;

    // After a motion completes, no pending flags
    limit1Triggered = false;
    limit2Triggered = false;
}

void updateRTCTime() {
    if (WiFi.status() == WL_CONNECTED) {
        // forced update to avoid using cached stale time
        if (!timeClient.update()) {
            timeClient.forceUpdate();
        }
        rtc.adjust(DateTime(timeClient.getEpochTime()));
        lastUpdate = millis();
        Serial.println("RTC updated from NTP");
    }
    else {
        Serial.println("WiFi not connected; skipping RTC update");
    }

}

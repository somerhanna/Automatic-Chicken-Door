#include "serial_commands.h"
#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "display.h"
#include "schedule.h"
#include "time_sync.h"
#include "ble_comm.h"
#include "utils.h"

// =====================================================
// Serial
// =====================================================
void handleSerialCommands() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "f" || command == "forward") {
    Serial.println("Manual: FORWARD");
    setMotorState(MOTOR_FORWARD);
  }
  else if (command == "r" || command == "reverse") {
    Serial.println("Manual: REVERSE");
    setMotorState(MOTOR_REVERSE);
  }
  else if (command == "s" || command == "stop") {
    Serial.println("Manual: STOP");
    setMotorState(MOTOR_IDLE);
  }
  else if (command == "status") {
    Serial.print("Motor: ");
    switch (currentState) {
      case MOTOR_IDLE: Serial.print("IDLE"); break;
      case MOTOR_FORWARD: Serial.print("FORWARD"); break;
      case MOTOR_REVERSE: Serial.print("REVERSE"); break;
    }

    Serial.print(" | Top: ");
    Serial.print(limitTopActive ? "ACTIVE" : "OK");
    Serial.print(" | Bottom: ");
    Serial.println(limitBottomActive ? "ACTIVE" : "OK");

    Serial.printf("Schedule: %s | Open: %s | Close: %s\n",
                  scheduleEnabled ? "ENABLED" : "DISABLED",
                  formatTime12Hour(openHour, openMinute).c_str(),
                  formatTime12Hour(closeHour, closeMinute).c_str());

    Serial.printf("Last executed day - Open: %d, Close: %d\n",
                  lastExecutedOpenDay, lastExecutedCloseDay);

    if (motorTimingActive) {
      unsigned long elapsed = millis() - motorStartTime;
      Serial.printf("Motor running for: %lu ms of %lu ms max\n", elapsed, MOTOR_RUN_TIME_MS);
    }

    printTime();
    Serial.printf("BLE connected: %s\n", deviceConnected ? "YES" : "NO");
    Serial.printf("Advertising active: %s\n", advertisingActive ? "YES" : "NO");
    sendStatus(true);
    updateDisplay(true);
  }
  else if (command == "resettime") {
    Serial.println("Manual: Resyncing time...");
    timeInitialized = syncTimeWithNTP();
    if (!timeInitialized) setDisplayMode(DISPLAY_NO_TIME, 2500);
    updateDisplay(true);
  }
  else if (command == "readv") {
    Serial.println("Manual: Restarting advertising...");
    advertisingActive = false;
    restartAdvertisingRequested = true;
  }
  else if (command == "oled") {
    Serial.println("Manual: Refreshing OLED...");
    updateDisplay(true);
  }
  else if (command == "resetdefaults") {
    Serial.println("Manual: Resetting schedule to defaults...");
    resetScheduleToDefaults();
    sendSchedule();
    updateDisplay(true);
  }
  else if (command == "help") {
    Serial.println("\nAvailable commands:");
    Serial.println("  forward / f    - Open door");
    Serial.println("  reverse / r    - Close door");
    Serial.println("  stop / s       - Stop motor");
    Serial.println("  status         - Show detailed status");
    Serial.println("  resettime      - Resync time");
    Serial.println("  readv          - Restart BLE advertising");
    Serial.println("  oled           - Force OLED refresh");
    Serial.println("  resetdefaults  - Reset schedule to defaults");
    Serial.println("  help           - Show this help");
  }
  else if (command.length() > 0) {
    Serial.println("Unknown command. Type 'help'");
  }
}

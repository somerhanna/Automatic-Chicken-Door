#include "motor.h"
#include "config.h"
#include "display.h"
#include "ble_comm.h"
#include <freertos/FreeRTOS.h>

// =====================================================
// Initalize GPIO pins
// =====================================================
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED; 

  void init_Pins(){
    
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
  Serial.println("Chicken Door Motor Controller");
  Serial.println("==========================================");}

// =====================================================
// Motor State
// ====================================================

MotorState currentState = MOTOR_IDLE;
MotorState requestedState = MOTOR_IDLE;

// Limit switch state
bool limitTopActive = false;
bool limitBottomActive = false;

// Motor timing
unsigned long motorStartTime = 0;
bool motorTimingActive = false;
bool timeoutTriggered = false;

// =====================================================
// Emergency Stop
// =====================================================
void emergencyStop(const char* reason) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  portENTER_CRITICAL(&motorMux);
  currentState = MOTOR_IDLE;
  requestedState = MOTOR_IDLE;
  motorTimingActive = false;
  timeoutTriggered = false;
  portEXIT_CRITICAL(&motorMux);

  Serial.printf("EMERGENCY STOP: %s\n", reason);

  if (strcmp(reason, "Top limit switch") == 0) {
    setDisplayMode(DISPLAY_LIMIT_TOP, 2500);
  } else if (strcmp(reason, "Bottom limit switch") == 0) {
    setDisplayMode(DISPLAY_LIMIT_BOTTOM, 2500);
  }

  sendStatus(true);
  updateDisplay(true);
}

// =====================================================
// Limit Switches
// =====================================================
void checkLimitSwitches() {
  bool topPressed = (digitalRead(LIMIT_SWITCH_TOP) == HIGH);
  bool bottomPressed = (digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH);

  digitalWrite(LED_BUILTIN, (topPressed || bottomPressed) ? HIGH : LOW);

  if (topPressed != limitTopActive) {
    limitTopActive = topPressed;
    if (limitTopActive) {
      Serial.println("TOP LIMIT SWITCH TRIGGERED!");
      if (currentState == MOTOR_FORWARD) {
        emergencyStop("Top limit switch");
      } else {
        sendStatus(true);
        setDisplayMode(DISPLAY_LIMIT_TOP, 2000);
      }
    } else {
      Serial.println("TOP limit switch released");
      sendStatus(true);
      updateDisplay(true);
    }
  }

  if (bottomPressed != limitBottomActive) {
    limitBottomActive = bottomPressed;
    if (limitBottomActive) {
      Serial.println("BOTTOM LIMIT SWITCH TRIGGERED!");
      if (currentState == MOTOR_REVERSE) {
        emergencyStop("Bottom limit switch");
      } else {
        sendStatus(true);
        setDisplayMode(DISPLAY_LIMIT_BOTTOM, 2000);
      }
    } else {
      Serial.println("BOTTOM limit switch released");
      sendStatus(true);
      updateDisplay(true);
    }
  }
}

// =====================================================
// Motor
// =====================================================
void updateMotor() {
  MotorState localRequested;
  portENTER_CRITICAL(&motorMux);
  localRequested = requestedState;
  portEXIT_CRITICAL(&motorMux);

  if (motorTimingActive && (currentState == MOTOR_FORWARD || currentState == MOTOR_REVERSE)) {
    if (millis() - motorStartTime >= MOTOR_RUN_TIME_MS) {
      if (!timeoutTriggered) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        portENTER_CRITICAL(&motorMux);
        currentState = MOTOR_IDLE;
        requestedState = MOTOR_IDLE;
        motorTimingActive = false;
        timeoutTriggered = true;
        portEXIT_CRITICAL(&motorMux);

        Serial.println("Motor timeout reached (6 seconds) - stopping");
        setDisplayMode(DISPLAY_TIMEOUT, 2500);
        sendStatus(true);
        updateDisplay(true);
      }
      return;
    }
  }

  if (currentState == MOTOR_IDLE) {
    timeoutTriggered = false;
  }

  if (localRequested != currentState) {
    if (localRequested == MOTOR_FORWARD && limitTopActive) {
      Serial.println("Cannot go FORWARD - Top limit switch active!");
      portENTER_CRITICAL(&motorMux);
      requestedState = MOTOR_IDLE;
      portEXIT_CRITICAL(&motorMux);
      setDisplayMode(DISPLAY_LIMIT_TOP, 2000);
      sendStatus(true);
      return;
    }

    if (localRequested == MOTOR_REVERSE && limitBottomActive) {
      Serial.println("Cannot go REVERSE - Bottom limit switch active!");
      portENTER_CRITICAL(&motorMux);
      requestedState = MOTOR_IDLE;
      portEXIT_CRITICAL(&motorMux);
      setDisplayMode(DISPLAY_LIMIT_BOTTOM, 2000);
      sendStatus(true);
      return;
    }


    portENTER_CRITICAL(&motorMux);
    currentState = requestedState;

    switch (currentState) {
      case MOTOR_IDLE:
        Serial.println("Motor: STOPPED");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        motorTimingActive = false;
        timeoutTriggered = false;
        setDisplayMode(DISPLAY_STOPPED, 1200);
        break;

      case MOTOR_FORWARD:
        Serial.println("Motor: FORWARD (opening door) - will run max 4 seconds");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        motorStartTime = millis();
        motorTimingActive = true;
        timeoutTriggered = false;
        setDisplayMode(DISPLAY_OPENING);
        break;

      case MOTOR_REVERSE:
        Serial.println("Motor: REVERSE (closing door) - will run max 6 seconds");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        motorStartTime = millis();
        motorTimingActive = true;
        timeoutTriggered = false;
        setDisplayMode(DISPLAY_CLOSING);
        break;
    }
    portEXIT_CRITICAL(&motorMux);

    sendStatus(true);
    updateDisplay(true);
  }
}

void setMotorState(MotorState state) {
  portENTER_CRITICAL(&motorMux);
  requestedState = state;
  portEXIT_CRITICAL(&motorMux);
}

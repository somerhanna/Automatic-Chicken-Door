#include "display.h"
#include <Wire.h>
#include "config.h"
#include "motor.h"
#include "schedule.h"
#include "time_sync.h"
#include "utils.h"

// =====================================================
// Display State
// =====================================================
DisplayMode displayMode = DISPLAY_BOOT;
unsigned long displayModeUntil = 0;
unsigned long lastDisplayUpdate = 0;

// =====================================================
// SSD1309 Display (128x64, I2C)
// =====================================================
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
bool oledReady = false;

// =====================================================
// OLED / SSD1309 UI
// =====================================================
void initOLED(bool timeInitSuccess) {
  Wire.begin(OLED_SDA, OLED_SCL);
  //Wire.setClock(400000); // 400kHz I2C speed
  display.begin();
  display.setContrast(255);
  oledReady = true;
  setDisplayMode(DISPLAY_BOOT, 1800);
  updateDisplay(true);
    // Handle time sync failure on the display
  if (!timeInitSuccess) {
    handleTimeInitFailure();
  }
}

void printInitStatus() {
  Serial.println("\nLimit switches (NC type):");
  Serial.println("  - Top switch: GPIO19 (stops OPEN)");
  Serial.println("  - Bottom switch: GPIO18 (stops CLOSE)");
  Serial.printf("\nMotor will run for maximum %lu seconds\n", MOTOR_RUN_TIME_MS / 1000);
}

void printScheduleStatus() {
  Serial.printf("Schedule: %s\n", scheduleEnabled ? "ENABLED" : "DISABLED");
  if (scheduleEnabled) {
    Serial.printf("  Open at: %s\n", formatTime12Hour(openHour, openMinute).c_str());
    Serial.printf("  Close at: %s\n", formatTime12Hour(closeHour, closeMinute).c_str());
    Serial.println("  Time will resync automatically once per day");
  }
}

void setDisplayMode(DisplayMode mode, unsigned long durationMs) {
  displayMode = mode;
  displayModeUntil = (durationMs > 0) ? millis() + durationMs : 0;
}

void handleTimeInitFailure() {
  Serial.println("Could not get initial time - schedule disabled");
  scheduleEnabled = false;
  setDisplayMode(DISPLAY_NO_TIME, 2500);
}

void drawCenteredText(const String& text, int y, const uint8_t* font) {
  display.setFont(font);
  int w = display.getStrWidth(text.c_str());
  int x = (128 - w) / 2;
  if (x < 0) x = 0;
  display.drawStr(x, y, text.c_str());
}

void drawTextAtX(const String& text, int x, int y, const uint8_t* font) {
  display.setFont(font);
  display.drawStr(x, y, text.c_str());
}

void drawHeaderBar(const char* title) {
  display.drawBox(0, 0, 128, 11);
  display.setDrawColor(0);
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(3, 8, title);
  display.setDrawColor(1);
}

void drawDoorMiniIcon(int x, int y, bool openDoor) {
  display.drawFrame(x + 3, y + 8, 14, 14);
  display.drawTriangle(x + 2, y + 8, x + 10, y, x + 18, y + 8);

  if (openDoor) {
    display.drawLine(x + 6, y + 21, x + 13, y + 14);
    display.drawLine(x + 13, y + 14, x + 13, y + 21);
  } else {
    display.drawFrame(x + 7, y + 12, 6, 10);
    display.drawPixel(x + 11, y + 17);
  }
}

void drawProgressBar(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  int x = 10;
  int y = 50;
  int w = 108;
  int h = 10;

  display.drawRFrame(x, y, w, h, 3);

  int fillW = ((w - 4) * percent) / 100;
  if (fillW > 0) {
    display.drawRBox(x + 2, y + 2, fillW, h - 4, 2);
  }
}

int getMotorRunPercent() {
  if (!motorTimingActive) return 0;
  unsigned long elapsed = millis() - motorStartTime;
  if (elapsed >= MOTOR_RUN_TIME_MS) return 100;
  return (int)((elapsed * 100UL) / MOTOR_RUN_TIME_MS);
}

void drawSchedulePanel() {
  int x = 84;
  int y = 14;
  int w = 40;
  int h = 48;

  display.drawRFrame(x, y, w, h, 4);

  display.setFont(u8g2_font_4x6_tf);
  display.drawStr(x + 4, y + 8, "AUTO");

  if (scheduleEnabled) {
    display.drawRBox(x + 21, y + 2, 15, 8, 2);
    display.setDrawColor(0);
    display.drawStr(x + 25, y + 8, "ON");
    display.setDrawColor(1);
  } else {
    display.drawRFrame(x + 20, y + 2, 18, 8, 2);
    display.drawStr(x + 24, y + 8, "OFF");
  }

  display.drawHLine(x + 3, y + 11, w - 6);

  display.setFont(u8g2_font_4x6_tf);
  display.drawStr(x + 4, y + 19, "OPEN");
  display.setFont(u8g2_font_3x5im_tr);
  display.drawStr(x + 4, y + 26, formatTime12Hour(openHour, openMinute).c_str());

  display.setFont(u8g2_font_4x6_tf);
  display.drawStr(x + 4, y + 36, "CLOSE");
  display.setFont(u8g2_font_3x5im_tr);
  display.drawStr(x + 4, y + 43, formatTime12Hour(closeHour, closeMinute).c_str());
}
void drawFooter() {
  String left = getCurrentDateString() + "    " + getMotorStateText();

  display.setFont(u8g2_font_3x5im_tr);

  display.drawRFrame(4, 52, 77, 10, 2);
  int lw = display.getStrWidth(left.c_str());
  display.drawStr(4 + (77 - lw) / 2, 59, left.c_str());
  
}

void drawBootScreen() {
  display.clearBuffer();

  drawHeaderBar("Automatic Chicken Door");

  drawDoorMiniIcon(10, 18, false);
  drawDoorMiniIcon(100, 18, true);

  drawCenteredText("SMART", 28, u8g2_font_logisoso16_tr);
  drawCenteredText("CHICKEN DOOR", 46, u8g2_font_6x12_tr);

  for (int i = 0; i < 6; i++) {
    int x = 22 + i * 14;
    int h = (i % 2 == 0) ? 4 : 8;
    display.drawBox(x, 61 - h, 8, h);
  }

  display.sendBuffer();
}

void drawBigStatusScreen(const char* line1, const char* line2, bool animated) {
  display.clearBuffer();

  unsigned long phase = millis() / 250;
  bool invert = animated && ((phase % 2) == 0);

  if (invert) {
    display.drawBox(0, 0, 128, 64);
    display.setDrawColor(0);
  } else {
    display.setDrawColor(1);
  }

  display.setFont(u8g2_font_logisoso18_tr);
  int w1 = display.getStrWidth(line1);
  display.drawStr((128 - w1) / 2, (line2 ? 28 : 36), line1);

  if (line2 != nullptr) {
    display.setFont(u8g2_font_6x12_tf);
    int w2 = display.getStrWidth(line2);
    display.drawStr((128 - w2) / 2, 46, line2);
  }

  if (animated) {
    int stage = (millis() / 130) % 5;
    for (int i = 0; i <= stage; i++) {
      int x = 18 + i * 18;
      display.drawTriangle(x, 58, x + 8, 50, x + 16, 58);
    }
  }

  display.setDrawColor(1);
  display.sendBuffer();
}

void drawActionScreen(bool opening) {
  display.clearBuffer();

  drawHeaderBar(opening ? "OPENING" : "CLOSING");

  if (opening) {
    drawCenteredText("OPENING", 30, u8g2_font_logisoso18_tr);
    drawCenteredText("DOOR", 46, u8g2_font_6x12_tf);
  } else {
    drawCenteredText("CLOSING", 30, u8g2_font_logisoso18_tr);
    drawCenteredText("DOOR", 46, u8g2_font_6x12_tf);
  }

  drawProgressBar(getMotorRunPercent());

  int stage = (millis() / 120) % 5;
  for (int i = 0; i < 5; i++) {
    bool active = (i <= stage);
    int x = opening ? (12 + i * 20) : (116 - i * 20);

    if (active) {
      if (opening) {
        display.drawTriangle(x, 48, x + 6, 42, x + 12, 48);
      } else {
        display.drawTriangle(x + 12, 48, x + 6, 42, x, 48);
      }
    }
  }

  display.sendBuffer();
}

void drawNormalScreen() {
  display.clearBuffer();

  drawHeaderBar("Automatic Chicken Door");

  // Main time panel
  display.drawRFrame(4, 14, 76, 34, 5);

  if (timeInitialized) {
    // nudged left so large digits fit in the box
    drawTextAtX(getCurrentTimeMainString(), 32, 37, u8g2_font_logisoso16_tr);
    drawTextAtX(getCurrentTimeSuffix(), 65, 45, u8g2_font_5x8_tf);
  } else {
    drawTextAtX("--:--", 32, 37, u8g2_font_logisoso16_tr);
    drawTextAtX("--", 31, 42, u8g2_font_4x6_tf);
    drawTextAtX("TIME NOT SYNCED", 23, 25, u8g2_font_3x5im_tr);
  }

  if (limitTopActive && !limitBottomActive) {
    drawDoorMiniIcon(6, 18, true);
  } else {
    drawDoorMiniIcon(6, 18, false);
  }

  drawSchedulePanel();
  drawFooter();

  display.sendBuffer();
}

void updateDisplay(bool force) {
  if (!oledReady) return;

  unsigned long now = millis();
  unsigned long interval = 250;

  if (currentState == MOTOR_FORWARD || currentState == MOTOR_REVERSE) {
    interval = 100;
  }

  if (!force && (now - lastDisplayUpdate < interval)) return;
  lastDisplayUpdate = now;

  if (displayModeUntil > 0 && now > displayModeUntil) {
    displayMode = DISPLAY_NORMAL;
    displayModeUntil = 0;
  }

  if (currentState == MOTOR_FORWARD) {
    drawActionScreen(true);
    return;
  }

  if (currentState == MOTOR_REVERSE) {
    drawActionScreen(false);
    return;
  }

  switch (displayMode) {
    case DISPLAY_BOOT:
      drawBootScreen();
      break;

    case DISPLAY_STOPPED:
      drawBigStatusScreen("STOPPED", "MANUAL STOP", true);
      break;

    case DISPLAY_LIMIT_TOP:
      drawBigStatusScreen("OPEN", "TOP LIMIT", true);
      break;

    case DISPLAY_LIMIT_BOTTOM:
      drawBigStatusScreen("CLOSED", "BOTTOM LIMIT", true);
      break;

    case DISPLAY_TIMEOUT:
      drawBigStatusScreen("TIMEOUT", "MOTOR STOPPED", true);
      break;

    case DISPLAY_NO_TIME:
      drawBigStatusScreen("NO TIME", "CHECK WIFI", true);
      break;

    case DISPLAY_OPENING:
      drawActionScreen(true);
      break;

    case DISPLAY_CLOSING:
      drawActionScreen(false);
      break;

    case DISPLAY_NORMAL:
    default:
      drawNormalScreen();
      break;
  }
}

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>

// =====================================================
// Display State
// =====================================================
enum DisplayMode {
  DISPLAY_BOOT,
  DISPLAY_NORMAL,
  DISPLAY_OPENING,
  DISPLAY_CLOSING,
  DISPLAY_STOPPED,
  DISPLAY_LIMIT_TOP,
  DISPLAY_LIMIT_BOTTOM,
  DISPLAY_TIMEOUT,
  DISPLAY_NO_TIME
};

extern DisplayMode displayMode;
extern unsigned long displayModeUntil;
extern unsigned long lastDisplayUpdate;

// =====================================================
// SSD1309 Display (128x64, I2C)
// =====================================================
extern U8G2_SSD1309_128X64_NONAME0_F_HW_I2C display;
extern bool oledReady;

void initOLED();
void updateDisplay(bool force = false);
void setDisplayMode(DisplayMode mode, unsigned long durationMs = 0);

void drawBootScreen();
void drawNormalScreen();
void drawBigStatusScreen(const char* line1, const char* line2 = nullptr, bool animated = false);
void drawActionScreen(bool opening);
void drawHeaderBar(const char* title);
void drawCenteredText(const String& text, int y, const uint8_t* font);
void drawTextAtX(const String& text, int x, int y, const uint8_t* font);
void drawProgressBar(int percent);
void drawDoorMiniIcon(int x, int y, bool openDoor);
void drawSchedulePanel();
void drawFooter();
int getMotorRunPercent();
void printInitStatus();
void printScheduleStatus();
void handleTimeInitFailure();
void initBLE();

#endif // DISPLAY_H

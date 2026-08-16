/*
 * ============================================================================
 * FILE: lcd_ui.cpp
 *
 * PURPOSE:
 *    Character LCD driver implementation.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/15/2026
 *
 * LAST MODIFIED:
 *    08/16/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - Wire.h
 *    - LiquidCrystal_I2C.h: "LiquidCrystal I2C by Frank de Brabander"
 *    - board_config.h: pin assignments, address and geometry
 *    - lcd_ui.h
 *
 * NOTES:
 *    Driver layer. It renders whatever string it is handed and has no idea
 *    what the text means. Composing that text is tasks_ui.cpp's job.
 *
 *    THE CALLER HOLDS THE I2C MUTEX. This bus is shared with the encoder,
 *    which is read by a task on the other core.
 *
 *    A shadow buffer skips redundant writes. Repainting 32 unchanged
 *    characters at 5 Hz is 160 needless bus transactions per second on a
 *    bus the 100 Hz control loop is also waiting for.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "board_config.h"
#include "lcd_ui.h"

static LiquidCrystal_I2C s_lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

/* What is currently on the panel. */
static char s_shadow[LCD_ROWS][LCD_COLS + 1];

static bool s_ready;

bool lcd_ui_init(void) {
    s_ready = false;

    /* Wire.begin() is idempotent and may already have been called by the
     * encoder driver; calling it again with the same parameters is safe and
     * keeps this driver independent of init order. */
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_HZ);

    /* Probe before initialising. The display library's init sequence is
     * write-only, so a wrong address or a dead bus produces a lit backlight
     * and no error at all -- the single most misleading LCD failure. */
    Wire.beginTransmission(LCD_I2C_ADDR);
    if (Wire.endTransmission() != 0) return false;

    s_lcd.init();
    s_lcd.backlight();
    s_lcd.clear();

    memset(s_shadow, 0, sizeof(s_shadow));
    s_ready = true;
    return true;
}

void lcd_ui_clear(void) {
    if (!s_ready) return;
    s_lcd.clear();
    memset(s_shadow, 0, sizeof(s_shadow));
}

void lcd_ui_write_at(uint8_t row, uint8_t col, const char *text) {
    if (!s_ready || text == NULL) return;
    if (row >= LCD_ROWS || col >= LCD_COLS) return;

    /* Build the full row as it should appear, padded with spaces so that a
     * shorter string erases whatever was longer before it. Truncate rather
     * than wrap: wrapping onto the next row would silently destroy the
     * other line. */
    char line[LCD_COLS + 1];
    memcpy(line, s_shadow[row], LCD_COLS + 1);
    line[LCD_COLS] = '\0';

    uint8_t i = col;
    while (i < LCD_COLS && *text) line[i++] = *text++;
    while (i < LCD_COLS)          line[i++] = ' ';

    if (memcmp(line, s_shadow[row], LCD_COLS) == 0) return;   /* unchanged */

    s_lcd.setCursor(0, row);
    s_lcd.print(line);
    memcpy(s_shadow[row], line, LCD_COLS + 1);
}

bool lcd_ui_is_alive(void) {
    if (!s_ready) return false;
    Wire.beginTransmission(LCD_I2C_ADDR);
    return (Wire.endTransmission() == 0);
}

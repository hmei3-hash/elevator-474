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
 *    08/15/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - board_config.h: pin assignments and geometry
 *    - lcd_ui.h
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "lcd_ui.h"

/* Shadow copy of what is currently on the panel, used to skip redundant
 * I2C traffic when the text has not changed. */
static char s_shadow[LCD_ROWS][LCD_COLS + 1];

bool lcd_ui_init(void) {
    // TODO: begin I2C on the configured pins, run the controller's power-on
    //       sequence, and confirm the expander acknowledges LCD_I2C_ADDR
    return false;
}

void lcd_ui_clear(void) {
    // TODO: issue the clear command and blank the shadow buffer
    return;
}

void lcd_ui_write_at(uint8_t row, uint8_t col, const char *text) {
    // TODO: bounds-check row and col, set the DDRAM address, write the
    //       truncated string, and update the shadow buffer
    (void)row;
    (void)col;
    (void)text;
    return;
}

bool lcd_ui_is_alive(void) {
    // TODO: probe LCD_I2C_ADDR and report whether it acknowledged
    return false;
}

/*
 * ============================================================================
 * FILE: lcd_ui.h
 *
 * PURPOSE:
 *    Driver for a character LCD behind an I2C expander. Exposes cursor
 *    placement and text output. It renders whatever string it is given.
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
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *    Deciding what text to show is the job of tasks_ui.cpp.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/**
 * Bring up the I2C bus and initialise the display controller.
 *
 * @return true if the expander acknowledged its address
 * Called from: setup(), before tasks start.
 */
bool lcd_ui_init(void);

/**
 * Clear the whole display.
 *
 * @return void
 * Called from: lcdTask (Core 0, 5 Hz).
 */
void lcd_ui_clear(void);

/**
 * Write a null-terminated string starting at the given cell. Text longer
 * than the remaining row is truncated, never wrapped.
 *
 * @param row   zero-based row index
 * @param col   zero-based column index
 * @param text  null-terminated string, must not be NULL
 * @return void
 * Called from: lcdTask (Core 0, 5 Hz).
 */
void lcd_ui_write_at(uint8_t row, uint8_t col, const char *text);

/**
 * Check that the expander still acknowledges on the bus.
 *
 * @return true if the display is healthy
 * Called from: lcdTask (Core 0, 5 Hz), periodically.
 */
bool lcd_ui_is_alive(void);

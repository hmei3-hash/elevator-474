/*
 * ElevatorA.ino
 *
 * Arduino IDE entry shim for:
 *   https://github.com/hmei3-hash/elevator-474
 *
 * IMPORTANT:
 * Keep this file inside the repo's ElevatorA/ folder together with:
 *   ElevatorA_main.cpp
 *   board_config.h
 *   app_types.h
 *   shared_state.*
 *   control.*
 *   cmd.cpp
 *   link.*
 *   stepper.*
 *   as5600.*
 *   hcsr04.*
 *   rc522.*
 *   lcd_ui.*
 *   tasks_ui.cpp
 *
 * The real setup(), loop(), FreeRTOS tasks, and application assembly remain
 * in ElevatorA_main.cpp. Arduino IDE compiles every .cpp file in the sketch
 * folder, so duplicating those functions here would cause duplicate-symbol
 * linker errors.
 *
 * Arduino IDE settings from the project:
 *   Board:            ESP32S3 Dev Module
 *   PSRAM:            OPI PSRAM
 *   Flash Size:       16MB (128Mb)
 *   USB CDC On Boot:  Enabled
 *   Partition Scheme: Default 4MB with spiffs
 *   Serial Monitor:   115200 baud
 */

#include <Arduino.h>

/*
 * setup() and loop() are implemented in ElevatorA_main.cpp.
 * These declarations make that intentional when this sketch is opened
 * in Arduino IDE.
 */
extern void setup();
extern void loop();

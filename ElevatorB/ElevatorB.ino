/*
 * ElevatorB.ino
 *
 * Arduino IDE entry shim for:
 *   https://github.com/hmei3-hash/elevator-474
 *
 * IMPORTANT:
 * Keep this file inside the repo's ElevatorB/ folder together with:
 *   ElevatorB_main.cpp
 *   board_config.h
 *   link_protocol.h
 *   mpu6050.*
 *   fall_detect.*
 *
 * The real setup(), loop(), FreeRTOS tasks, MPU6050 sampling, fall detection,
 * and ESP-NOW transmit path remain in ElevatorB_main.cpp and the driver files.
 * Arduino IDE compiles every .cpp file in the sketch folder, so duplicating
 * those functions here would cause duplicate-symbol linker errors.
 *
 * Current project radio configuration:
 *   Wi-Fi channel: 1
 *   Board A peer MAC: 80:B5:4E:E3:19:58
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
 * setup() and loop() are implemented in ElevatorB_main.cpp.
 */
extern void setup();
extern void loop();

#pragma once

#include <Arduino.h>
#include "app_types.h"

/* Primary controller lifecycle. */
bool control_init(void);
bool control_request_floor(uint8_t floor, req_source_t source);
uint8_t control_select_target(void);
void control_step(uint32_t dt_ms);
void control_emergency_stop(uint32_t cause);

/* Feed one NEW valid HC-SR04 sample to the controller.
 * Called only by ultrasonicTask.
 */
void control_update_ultrasonic(uint32_t raw_mm, uint32_t timestamp_ms);

/* Bench tuning / diagnostics. */
void control_set_gains(float kp, float ki, float kd);
void control_get_gains(float *kp, float *ki, float *kd);

void control_set_target_mm(float mm);
bool control_get_debug(float *target_mm, float *pos_mm, float *rate_sps);

/* Hold the current ultrasonic position. */
bool control_hold_here(void);

/* Clears the controller's internal stall latch.
 * Note: the shared FAULT_STEPPER_STALL bit remains latched by the existing
 * shared-state design; a boot-time TMC2209 init failure still requires fixing.
 */
void control_clear_stall(void);

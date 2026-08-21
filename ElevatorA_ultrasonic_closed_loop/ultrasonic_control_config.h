#pragma once

/*
 * Ultrasonic closed-loop tuning/configuration.
 *
 * The HC-SR04 reading is now the primary position feedback.
 *
 * IMPORTANT:
 * 1) Measure the HC-SR04 reading at each real floor and replace the -1 values.
 * 2) Leave CTRL_ULTRA_KP/KI/KD at zero for the first power-up.
 * 3) Use Serial command "g <mm>" with a small move and "p <kp> 0 0" to tune.
 * 4) If increasing the target makes the car move AWAY from the target,
 *    change CTRL_ULTRA_DIRECTION_SIGN from +1.0f to -1.0f.
 */

/* Floor setpoints: HC-SR04 distance in millimetres at each floor.
 * -1 disables that floor until you measure it.
 */
#define CTRL_FLOOR0_MM               (-1.0f)
#define CTRL_FLOOR1_MM               (-1.0f)
#define CTRL_FLOOR2_MM               (-1.0f)

/* Low-pass filter:
 * filtered = old + alpha * (raw - old)
 * Repo logs showed ~1-2 mm standstill jitter, so do not use raw readings
 * directly for the controller.
 */
#define CTRL_ULTRA_FILTER_ALPHA       0.35f

/* Position tolerance around the target. */
#define CTRL_ULTRA_DEADBAND_MM        5.0f

/* If no fresh valid ultrasonic sample arrives for this long, stop motion. */
#define CTRL_ULTRA_STALE_MS           250u

/* Conservative ultrasonic-feedback motion limits.
 * Raise only after the loop direction and sensing are verified.
 */
#define CTRL_ULTRA_MAX_SPS            800.0f
#define CTRL_ULTRA_MAX_ACCEL_SPS2     3000.0f

/* +1: positive PID output -> positive step rate.
 * -1: positive PID output -> negative step rate.
 */
#define CTRL_ULTRA_DIRECTION_SIGN     (+1.0f)

/* PID gains. Output is directly in steps/s.
 * Start at zero intentionally; enable P on the bench after direction check.
 */
#define CTRL_ULTRA_KP                 0.0f
#define CTRL_ULTRA_KI                 0.0f
#define CTRL_ULTRA_KD                 0.0f

/* Stall detection:
 * if a meaningful step rate is commanded for this many ultrasonic samples
 * but the net position change is smaller than MIN_MOVE_MM, declare a stall.
 * 8 samples at the existing 60 ms ultrasonic period is ~0.48 s.
 */
#define CTRL_ULTRA_STALL_MIN_RATE_SPS 250.0f
#define CTRL_ULTRA_STALL_SAMPLES      8u
#define CTRL_ULTRA_STALL_MIN_MOVE_MM  3.0f

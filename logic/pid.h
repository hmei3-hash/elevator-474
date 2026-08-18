/*
 * ============================================================================
 * FILE: pid.h
 *
 * PURPOSE:
 *    Discrete PID controller, expressed in plain C with no platform
 *    dependencies. Holds gains and integrator state; the caller supplies the
 *    setpoint, the measurement, and the elapsed time.
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
 *    - stdint.h: fixed-width integer types
 *
 *    LINKAGE
 *    The declarations below are wrapped in extern "C". This file is C, but
 *    every translation unit on the target that includes it is C++ (Arduino
 *    compiles .cpp and .ino as C++). Without the wrapper the caller emits a
 *    mangled C++ symbol while the definition is a plain C symbol, and the
 *    build fails at link time with "undefined reference" to a function that
 *    plainly exists.
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 * ============================================================================
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Controller gains and accumulated state. The caller owns the storage.
 *
 * NAMED pid_ctl_t, NOT pid_t. POSIX <sys/types.h> already defines pid_t as
 * a process id, and the ESP32 toolchain pulls that header in through
 * Arduino.h. The collision is a hard compile error the moment any
 * translation unit includes both, which is every file on the target.
 */
typedef struct {
    float kp;              /* proportional gain */
    float ki;              /* integral gain, per second */
    float kd;              /* derivative gain, seconds */

    float integrator;      /* accumulated error-seconds */
    float prev_error;      /* error from the previous update */

    float out_min;         /* lower output clamp */
    float out_max;         /* upper output clamp */
} pid_ctl_t;

/**
 * Set the gains and output clamps and clear the accumulated state.
 *
 * @param c        controller to initialise, must not be NULL
 * @param kp       proportional gain
 * @param ki       integral gain, per second
 * @param kd       derivative gain, seconds
 * @param out_min  lower output clamp
 * @param out_max  upper output clamp, must be greater than out_min
 * @return void
 * Called from: control_init() on Board A, and from the host test harness.
 */
void pid_init(pid_ctl_t *c, float kp, float ki, float kd, float out_min, float out_max);

/**
 * Run one controller update.
 *
 * @param c          controller state, must not be NULL
 * @param setpoint   desired value, same units as measurement
 * @param measured   observed value
 * @param dt_s       elapsed time since the previous update, seconds,
 *                   must be greater than zero
 * @return control output, clamped to [out_min, out_max]
 * Called from: controlTask (Core 1, 100 Hz), and from the host test harness.
 */
float pid_update(pid_ctl_t *c, float setpoint, float measured, float dt_s);

/**
 * Clear the integrator and derivative history without changing the gains.
 * Call whenever the loop is re-entered after being open.
 *
 * @param c  controller state, must not be NULL
 * @return void
 * Called from: control_emergency_stop(), and from the host test harness.
 */
void pid_reset(pid_ctl_t *c);

#ifdef __cplusplus
}   /* extern "C" */
#endif

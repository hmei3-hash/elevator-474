/*
 * ============================================================================
 * FILE: cmd.cpp
 *
 * PURPOSE:
 *    Serial command interface for bench work. Lets the control gains, the
 *    target angle and the fault state be changed while the system runs.
 *
 *    WHY THIS EXISTS
 *    The gains have to move from the prototype's 1 kHz values to something
 *    that works at 100 Hz, and that takes dozens of attempts. Without this,
 *    each attempt is a full rebuild-and-flash cycle -- minutes each, an
 *    afternoon in total. With it, an attempt is one line typed into a
 *    terminal.
 *
 *    It is a bench facility, not elevator behaviour. Nothing in the
 *    application calls it, and it can be removed by dropping this file and
 *    the one task creation line without touching anything else.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/18/2026
 *
 * LAST MODIFIED:
 *    08/18/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - board_config.h, app_types.h
 *    - shared_state.h, control.h, stepper.h, link.h
 *
 * NOTES:
 *    NON-BLOCKING BY CONSTRUCTION.
 *    The prototype used Serial.readStringUntil(), which blocks for up to a
 *    second when a line arrives without a newline. That was inside a 1 kHz
 *    control loop, with the motor running. Here the reader takes whatever
 *    characters are available and returns; a partial line simply waits for
 *    the next pass. Nothing in this file can stall for longer than one of
 *    its own iterations.
 *
 *    This task also owns no hardware. It reads and writes application state
 *    through the same accessors every other task uses.
 *
 * COMMANDS
 *    p <kp> <ki> <kd>   set gains and clear the integrator
 *    k                  print the gains in force
 *    g <deg>            command an absolute shaft angle
 *    r <deg>            command a relative move
 *    z                  take the current angle as zero
 *    s                  stop and hold here
 *    c                  clear a latched stall
 *    d                  toggle the 100 Hz telemetry stream (mutes the log)
 *    m                  mute or unmute the log stream
 *    ?                  print status
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "stepper.h"
#include "link.h"

#define CMD_LINE_MAX   64

/* Telemetry stream, off by default. During a tuning run it is the only view
 * of what the loop is doing.
 *
 * IT RUNS AT THE CONTROL RATE, NOT SLOWER. It was 10 Hz, against a 100 Hz
 * loop: a step response that settles in 200 ms left two samples, from which
 * neither overshoot nor settling time can be read. Sampling a loop ten times
 * slower than it runs does not produce a coarse picture of it; it produces no
 * picture of it. */
static bool s_stream;

/* Defined in tasks_ui.cpp. Suppresses log OUTPUT only -- the queue is still
 * drained and the suppressed lines are counted and reported. */
extern volatile bool g_log_muted;

/* Whether the log was already muted before the stream turned it off, so
 * turning the stream off restores what the operator had, not a guess. */
static bool s_mute_before_stream;

/*
 * ============================================================================
 * FUNCTION: cmd_print_status
 *
 * PURPOSE:
 *    One-shot dump of everything worth seeing during a tuning session.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    cmdTask (Core 0, priority 1).
 * ============================================================================
 */
static void cmd_print_status(void) {
    float kp, ki, kd, tgt, pos, rate;
    control_get_gains(&kp, &ki, &kd);
    bool ok = control_get_debug(&tgt, &pos, &rate);

    system_state_t st;
    shared_state_get(&st);

    Serial.println(F("--- status ---"));
    Serial.printf("  gains       kp %.3f  ki %.3f  kd %.4f\n", kp, ki, kd);
    Serial.printf("  target      %.2f deg\n", tgt);
    Serial.printf("  position    %.2f deg %s\n", pos, ok ? "" : "(READ FAILED)");
    Serial.printf("  error       %.2f deg\n", tgt - pos);
    Serial.printf("  rate        %.0f steps/s\n", rate);
    Serial.printf("  steps       %ld commanded\n", (long)stepper_get_position());
    Serial.printf("  mode        %d   faults 0x%08lX\n",
                  (int)st.mode, (unsigned long)st.fault_flags);
    Serial.printf("  heartbeat   %lu ms ago   link drops %lu\n",
                  (unsigned long)(millis() - st.last_heartbeat_ms),
                  (unsigned long)link_get_dropped_count());
}

/*
 * ============================================================================
 * FUNCTION: cmd_execute
 *
 * PURPOSE:
 *    Act on one complete command line.
 *
 * PARAMETERS:
 *    line (char*) - null-terminated, already trimmed of the newline
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    cmdTask (Core 0, priority 1).
 * ============================================================================
 */
static void cmd_execute(char *line) {
    if (line[0] == '\0') return;

    char  c    = line[0];
    char *args = line + 1;
    while (*args == ' ') args++;

    float a, b, d;

    switch (c) {
    case 'p':
        if (sscanf(args, "%f %f %f", &a, &b, &d) == 3) {
            control_set_gains(a, b, d);
            Serial.printf("-> gains kp %.3f  ki %.3f  kd %.4f\n", a, b, d);
        } else {
            Serial.println(F("!! usage: p <kp> <ki> <kd>"));
        }
        break;

    case 'k':
        control_get_gains(&a, &b, &d);
        Serial.printf("   kp %.3f  ki %.3f  kd %.4f\n", a, b, d);
        break;

    case 'g':
        if (sscanf(args, "%f", &a) == 1) {
            control_set_target_deg(a);
            Serial.printf("-> target %.2f deg\n", a);
        } else {
            Serial.println(F("!! usage: g <deg>"));
        }
        break;

    case 'r':
        if (sscanf(args, "%f", &a) == 1) {
            float tgt, pos, rate;
            control_get_debug(&tgt, &pos, &rate);
            control_set_target_deg(tgt + a);
            Serial.printf("-> target %.2f deg\n", tgt + a);
        } else {
            Serial.println(F("!! usage: r <deg>"));
        }
        break;

    case 'z':
        if (control_zero_here()) Serial.println(F("-> zeroed here"));
        else                     Serial.println(F("!! encoder did not answer"));
        break;

    case 's': {
        /* Hold where we are, rather than commanding zero: a stop that flies
         * the car back to the origin is not a stop. */
        float tgt, pos, rate;
        if (control_get_debug(&tgt, &pos, &rate)) {
            control_set_target_deg(pos);
            Serial.printf("-> holding at %.2f deg\n", pos);
        } else {
            stepper_emergency_stop();
            Serial.println(F("-> encoder unreadable; motion stopped"));
        }
        break;
    }

    case 'c':
        control_clear_stall();
        Serial.println(F("-> stall cleared"));
        break;

    case 'd':
        s_stream = !s_stream;
        if (s_stream) {
            /* Mute the log for the duration. Interleaving log lines into the
             * CSV corrupts the one artifact this stream exists to produce,
             * and the two writers share a port with no arbitration between
             * them. Remembering the previous setting means turning the
             * stream off restores what the operator had. */
            s_mute_before_stream = g_log_muted;
            g_log_muted = true;
            Serial.println(F("-> telemetry on (100 Hz), log muted"));
            Serial.println(F("ms,target_deg,pos_deg,error_deg,rate_sps"));
        } else {
            g_log_muted = s_mute_before_stream;
            Serial.printf("-> telemetry off, log %s\n",
                          g_log_muted ? "still muted" : "back on");
        }
        break;

    case 'm':
        g_log_muted = !g_log_muted;
        Serial.printf("-> log %s\n", g_log_muted ? "MUTED" : "on");
        break;

    case '?':
        cmd_print_status();
        break;

    default:
        Serial.println(F("? p<kp ki kd>  k  g<deg>  r<deg>  z  s  c  d  m  ?"));
        break;
    }
}

/*
 * ============================================================================
 * FUNCTION: cmdTask
 *
 * PURPOSE:
 *    Assemble serial input into lines without blocking, execute them, and
 *    emit the optional telemetry stream.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 1, 20 Hz.
 *
 * NOTE:
 *    Priority 1 and Core 0 on purpose. A human typing is the least urgent
 *    thing on this board, and nothing here may compete with the control
 *    loop on Core 1.
 * ============================================================================
 */
void cmdTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    static char line[CMD_LINE_MAX];
    static uint8_t len = 0;

    for (;;) {
        /* Drain whatever has arrived. Characters are consumed one at a time
         * and never waited for, so an unterminated line costs nothing. */
        while (Serial.available() > 0) {
            int ch = Serial.read();
            if (ch < 0) break;

            if (ch == '\n' || ch == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    cmd_execute(line);
                    len = 0;
                }
            } else if (len < CMD_LINE_MAX - 1) {
                line[len++] = (char)ch;
            } else {
                /* Overlong line: drop it rather than truncating into a
                 * command that was never typed. */
                len = 0;
                Serial.println(F("!! line too long, discarded"));
            }
        }

        if (s_stream) {
            static uint32_t next_ms = 0;
            uint32_t now = millis();
            if ((int32_t)(now - next_ms) >= 0) {
                next_ms = now + 10;                   /* 100 Hz, = control rate */
                float tgt, pos, rate;
                control_get_debug(&tgt, &pos, &rate);
                /* Columns, so the stream can be pasted straight into a
                 * spreadsheet and plotted as setpoint versus measured --
                 * which is exactly the evidence VV A-VAL1 asks for. */
                Serial.printf("%lu,%.2f,%.2f,%.2f,%.0f\n",
                              (unsigned long)now, tgt, pos, tgt - pos, rate);
            }
        }

        /* 20 Hz is plenty for a human typing, but the telemetry stream above
         * cannot emit faster than this task wakes. Raising the period only
         * while streaming keeps the idle cost at 20 Hz and still gives the
         * CSV one sample per control iteration. */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(s_stream ? 10 : 50));
    }
}

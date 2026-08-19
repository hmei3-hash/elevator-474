#include <Arduino.h>

#include "board_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "stepper.h"
#include "link.h"

#define CMD_LINE_MAX 64

extern volatile bool g_log_muted;

static bool s_stream;
static bool s_mute_before_stream;

static void cmd_print_status(void) {
    float kp, ki, kd, tgt, pos, rate;
    control_get_gains(&kp, &ki, &kd);
    bool ok = control_get_debug(&tgt, &pos, &rate);

    system_state_t st;
    shared_state_get(&st);

    Serial.println(F("--- status ---"));
    Serial.printf("  gains       kp %.3f  ki %.3f  kd %.4f\n", kp, ki, kd);
    Serial.printf("  target      %.1f mm\n", tgt);
    Serial.printf("  position    %.1f mm %s\n", pos, ok ? "" : "(NO ULTRASONIC SAMPLE)");
    Serial.printf("  error       %.1f mm\n", tgt - pos);
    Serial.printf("  raw         %ld mm\n", (long)st.position_raw_mm);
    Serial.printf("  rate        %.0f steps/s\n", rate);
    Serial.printf("  steps       %ld commanded\n", (long)stepper_get_position());
    Serial.printf("  mode        %d   faults 0x%08lX\n",
                  (int)st.mode, (unsigned long)st.fault_flags);
    Serial.printf("  heartbeat   %lu ms ago   link drops %lu\n",
                  (unsigned long)(millis() - st.last_heartbeat_ms),
                  (unsigned long)link_get_dropped_count());
}

static void cmd_execute(char *line) {
    if (line[0] == '\0') return;

    char c = line[0];
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
            control_set_target_mm(a);
            Serial.printf("-> target %.1f mm\n", a);
        } else {
            Serial.println(F("!! usage: g <mm>"));
        }
        break;

    case 'r':
        if (sscanf(args, "%f", &a) == 1) {
            float tgt, pos, rate;
            control_get_debug(&tgt, &pos, &rate);
            (void)pos;
            (void)rate;
            control_set_target_mm(tgt + a);
            Serial.printf("-> target %.1f mm\n", tgt + a);
        } else {
            Serial.println(F("!! usage: r <delta_mm>"));
        }
        break;

    case 's':
    case 'z':
        if (control_hold_here())
            Serial.println(F("-> holding current ultrasonic position"));
        else {
            stepper_emergency_stop();
            Serial.println(F("!! no ultrasonic position; motion stopped"));
        }
        break;

    case 'c':
        control_clear_stall();
        Serial.println(F("-> internal stall latch cleared"));
        break;

    case 'd':
        s_stream = !s_stream;
        if (s_stream) {
            s_mute_before_stream = g_log_muted;
            g_log_muted = true;
            Serial.println(F("-> telemetry on (~20 Hz), log muted"));
            Serial.println(F("ms,target_mm,pos_mm,error_mm,rate_sps"));
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
        Serial.println(F("? p<kp ki kd>  k  g<mm>  r<delta_mm>  s  c  d  m  ?"));
        break;
    }
}

void cmdTask(void *arg) {
    (void)arg;

    static char line[CMD_LINE_MAX];
    size_t used = 0;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        while (Serial.available() > 0) {
            char ch = (char)Serial.read();

            if (ch == '\r') continue;

            if (ch == '\n') {
                line[used] = '\0';
                cmd_execute(line);
                used = 0;
            } else if (used + 1 < sizeof(line)) {
                line[used++] = ch;
            } else {
                used = 0;
                Serial.println(F("!! command too long"));
            }
        }

        if (s_stream) {
            float tgt, pos, rate;
            bool ok = control_get_debug(&tgt, &pos, &rate);
            if (ok) {
                Serial.printf("%lu,%.2f,%.2f,%.2f,%.0f\n",
                              (unsigned long)millis(),
                              tgt, pos, tgt - pos, rate);
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

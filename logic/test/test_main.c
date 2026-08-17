/*
 * ============================================================================
 * FILE: test_main.c
 *
 * PURPOSE:
 *    Host test harness for the platform-independent logic. Builds and runs on
 *    a development machine with plain gcc; it never touches hardware.
 *    
 *    Build and run:
 *        gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c ../filter.c -lm
 *        ./test_logic
 *    
 *    Every case below is empty. A harness that reports PASS while asserting
 *    nothing is worse than no harness, so each case must be filled in before
 *    its result means anything.
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
 *    - stdio.h: printf
 *    - pid.h, filter.h: units under test
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 * ============================================================================
 */

#include <stdio.h>
#include <math.h>
#include "../pid.h"
#include "../filter.h"

/* Count of cases run and cases failed. */
static int s_run;
static int s_failed;

/*
 * ============================================================================
 * FUNCTION: check
 *
 * PURPOSE:
 *    Record one assertion result and print it.
 *
 * PARAMETERS:
 *    name (const char*) - case label
 *    condition (int) - non-zero if the assertion held
 *
 * RETURN VALUE:
 *    void
 * ============================================================================
 */
static int nearly(float a, float b) {
    return fabsf(a - b) < 1e-4f;
}

static void check(const char *name, int condition) {
    s_run++;
    if (!condition) {
        s_failed++;
        printf("FAIL  %s\n", name);
    } else {
        printf("pass  %s\n", name);
    }
}

/*
 * ============================================================================
 * FUNCTION: test_pid_reset_clears_state
 *
 * PURPOSE:
 *    A reset controller must produce no integral contribution on its next
 *    update.
 * ============================================================================
 */
static void test_pid_reset_clears_state(void) {
    pid_ctl_t c;
    pid_init(&c, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f);   /* integral only */

    /* Ten seconds of unit error accumulates ten error-seconds. */
    for (int i = 0; i < 10; i++) pid_update(&c, 1.0f, 0.0f, 1.0f);
    float before = pid_update(&c, 1.0f, 0.0f, 1.0f);

    pid_reset(&c);
    float after = pid_update(&c, 1.0f, 0.0f, 1.0f);

    /* After a reset the first update should contribute one error-second,
     * not eleven. */
    check("pid_reset clears integrator", before > 5.0f && nearly(after, 1.0f));
}

/*
 * ============================================================================
 * FUNCTION: test_pid_output_is_clamped
 *
 * PURPOSE:
 *    Output must never leave [out_min, out_max] regardless of error size.
 * ============================================================================
 */
static void test_pid_output_is_clamped(void) {
    pid_ctl_t c;
    pid_init(&c, 1000.0f, 0.0f, 0.0f, -5.0f, 5.0f);

    float hi = pid_update(&c, 1000.0f, 0.0f, 0.01f);
    pid_reset(&c);
    float lo = pid_update(&c, -1000.0f, 0.0f, 0.01f);

    check("pid_update clamps output", nearly(hi, 5.0f) && nearly(lo, -5.0f));
}

/*
 * ============================================================================
 * FUNCTION: test_pid_zero_error_holds
 *
 * PURPOSE:
 *    With the measurement already at setpoint and no accumulated state, the
 *    output must be zero.
 * ============================================================================
 */
static void test_pid_zero_error_holds(void) {
    pid_ctl_t c;
    pid_init(&c, 5.0f, 2.0f, 0.5f, -100.0f, 100.0f);
    float out = pid_update(&c, 42.0f, 42.0f, 0.01f);
    check("pid_update returns zero at setpoint", nearly(out, 0.0f));
}

/*
 * ============================================================================
 * FUNCTION: test_moving_avg_rejects_bad_window
 *
 * PURPOSE:
 *    A window of zero or one larger than the maximum must be refused.
 * ============================================================================
 */
static void test_moving_avg_rejects_bad_window(void) {
    moving_avg_t f;
    int zero = moving_avg_init(&f, 0);
    int over = moving_avg_init(&f, FILTER_MAX_WINDOW + 1);
    int ok   = moving_avg_init(&f, FILTER_MAX_WINDOW);
    check("moving_avg_init rejects bad window",
          zero < 0 && over < 0 && ok == 0);
}

/*
 * ============================================================================
 * FUNCTION: test_moving_avg_constant_input
 *
 * PURPOSE:
 *    Feeding the same value repeatedly must converge to that value.
 * ============================================================================
 */
static void test_moving_avg_constant_input(void) {
    moving_avg_t f;
    moving_avg_init(&f, 8);
    float avg = 0.0f;
    for (int i = 0; i < 40; i++) avg = moving_avg_push(&f, 3.5f);
    check("moving_avg converges on constant input", nearly(avg, 3.5f));
}

/*
 * ============================================================================
 * FUNCTION: test_moving_avg_partial_window
 *
 * PURPOSE:
 *    Before the window is full the average must divide by the number of
 *    samples actually held, not by the window length.
 * ============================================================================
 */
static void test_moving_avg_partial_window(void) {
    moving_avg_t f;
    moving_avg_init(&f, 10);

    /* Three samples into a window of ten: the divisor must be 3, not 10.
     * Dividing by the window would report 1.8 instead of 6. */
    moving_avg_push(&f, 4.0f);
    moving_avg_push(&f, 6.0f);
    float avg = moving_avg_push(&f, 8.0f);

    check("moving_avg handles a partial window", nearly(avg, 6.0f));
}

/*
 * ============================================================================
 * FUNCTION: test_residual_tracks_peak
 *
 * PURPOSE:
 *    The peak residual must retain the largest magnitude seen, including
 *    from a negative excursion.
 * ============================================================================
 */
static void test_residual_tracks_peak(void) {
    residual_t r;
    residual_init(&r);

    residual_push(&r, 0.0f, 12.0f);    /* residual -12 */
    residual_push(&r, 3.0f, 0.0f);     /* residual  +3 */

    /* The peak must be the magnitude of the negative excursion. Keeping a
     * signed maximum would report 3 and hide the larger event. */
    check("residual_push tracks peak magnitude", nearly(r.peak_residual, 12.0f));
}

/*
 * ============================================================================
 * FUNCTION: main
 *
 * PURPOSE:
 *    Run every case and report the tally.
 *
 * RETURN VALUE:
 *    int - 0 if every case passed, 1 otherwise
 * ============================================================================
 */
int main(void) {
    printf("=== logic test harness ===\n");

    test_pid_reset_clears_state();
    test_pid_output_is_clamped();
    test_pid_zero_error_holds();
    test_moving_avg_rejects_bad_window();
    test_moving_avg_constant_input();
    test_moving_avg_partial_window();
    test_residual_tracks_peak();

    printf("=== %d run, %d failed ===\n", s_run, s_failed);

    return (s_failed == 0) ? 0 : 1;
}

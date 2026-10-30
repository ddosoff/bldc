/* Host harness: the runner includes the actual SkyPUFF functions below.
 * Only the clock, measurements and hardware interfaces are simulated.
 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint32_t systime_t;
#include "production_types.inc"

#define MS2ST(ms) ((systime_t)(ms) * (TEST_CLOCK_HZ / 1000U))
#define ST2MS(ticks) (((ticks) * 1000U + TEST_CLOCK_HZ - 1U) / TEST_CLOCK_HZ)

static systime_t test_time;
static systime_t chVTGetSystemTime(void) { return test_time; }
static systime_t chTimeDiffX(systime_t start, systime_t end) { return end - start; }

static struct {
    float pull_current, amps_per_kg, pre_pull_k, takeoff_pull_k, fast_pull_k, manual_brake_current;
    float unwinding_current, unwinding_strong_current, unwinding_strong_erpm;
    float antisex_acceleration_off_mss;
    int pull_applying_period, braking_applying_period, braking_length, slowing_length;
} config;
static int loop_step;
static systime_t unwinding_start_time;
static struct { float filtered_speed_tac_ms, filtered_acceleration_tac_mss; } measurement;
static skypuff_state state;
static float antisex_amps, antisex_amps_gain;
static float output_current;
static smooth_motor_mode output_mode;
static bool expect_braking_zone_message;

static void mc_interface_set_current(float current) {
    output_current = current;
    output_mode = MOTOR_CURRENT;
}
static void mc_interface_set_brake_current(float current) {
    output_current = current;
    output_mode = MOTOR_BRAKING;
}
static void mc_interface_release_motor(void) {
    output_current = 0.0f;
    output_mode = MOTOR_RELEASED;
}
static void mc_interface_set_pid_speed(float erpm) {
    (void)erpm;
    output_mode = MOTOR_SPEED;
}
static const char *state_str(skypuff_state s) { (void)s; return "test"; }
static const char *motor_mode_str(smooth_motor_mode mode) { (void)mode; return "test"; }
static void save_custom_msg(const char *format, ...) {
    if (expect_braking_zone_message && strstr(format, "braking/slowing zone detected"))
        return;
    fprintf(stderr, "Unexpected motor diagnostic: %s\n", format);
    abort();
}
// Use identical numeric speed and ERPM thresholds in the host fixture.
static float erpm_to_ms(float erpm) { return erpm; }
#ifdef DEBUG_SMOOTH_MOTOR
static void commands_printf(const char *format, ...) { (void)format; }
#endif

#include "production_functions.inc"

static const int test_position = 1000000; // Outside instant braking/slowing zones.

static void reset(smooth_motor_mode mode, float current) {
    loop_step = 0;
    test_time = 0;
    state = UNINITIALIZED; // Motor rules must not depend on the application FSM.
    current_motor_state.mode = mode;
    current_motor_state.param.current = current;
    target_motor_state = current_motor_state;
    terminal_motor_state = current_motor_state;
    output_mode = mode;
    output_current = current_motor_state.param.current;
    next_smooth_motor_adjustment = 0;
    smooth_stage.active = false;
    smooth_stage.paused = false;
    measurement.filtered_acceleration_tac_mss = 0.0f;
    antisex_amps = 0.0f;
    antisex_amps_gain = 1.0f;
}

static void tick(void) {
    test_time = MS2ST(loop_step);
    // Deliberately change the application state without changing motor commands.
    state = (skypuff_state)(loop_step % (DISCONNECTED + 1));
    if (smooth_motor_adjustment_due())
        smooth_motor_adjustment(test_position, test_position);
}

static void advance_to(int time_ms) {
    while (loop_step < time_ms) {
        loop_step++;
        tick();
    }
}

static void record(const char *scenario, int time_ms) {
    printf("%s,%d,%d,%.9g,%.9g,%.9g,%d,%u,%d,%.9g,%.9g\n",
           scenario, time_ms, output_mode, (double)output_current,
           (double)(output_current / config.amps_per_kg),
           (double)(target_motor_state.param.current / config.amps_per_kg),
           smooth_stage.active, smooth_stage.start_time, smooth_stage.duration_ms,
           (double)(smooth_stage.start_current / config.amps_per_kg),
           (double)(smooth_stage.target_current / config.amps_per_kg));
}

static void run_scenario(int scenario, int end_ms) {
    const char *names[] = {"unwinding_pre_pull", "strong_unwinding_pre_pull",
                          "pull_unwinding", "pull_strong_unwinding", "pull_manual_braking",
                          "unwinding_manual_braking", "manual_braking_unwinding",
                          "pre_pull_takeoff_pull", "takeoff_pull_pull",
                          "pre_pull_unwinding", "takeoff_pull_unwinding", "pull_fast_pull",
                          "fast_pull_manual_braking", "takeoff_pull_manual_braking",
                          "pre_pull_manual_braking", "strong_unwinding_manual_braking"};
    float initial = config.unwinding_current;
    if (scenario == 1) initial = config.unwinding_strong_current;
    if (scenario >= 2 && scenario <= 4) initial = config.pull_current;
    if (scenario == 6) initial = config.manual_brake_current;
    if (scenario == 7 || scenario == 9) initial = config.pre_pull_k * config.pull_current;
    if (scenario == 8 || scenario == 10) initial = config.takeoff_pull_k * config.pull_current;
    if (scenario == 11) initial = config.pull_current;
    if (scenario == 12) initial = config.fast_pull_k * config.pull_current;
    if (scenario == 13) initial = config.takeoff_pull_k * config.pull_current;
    if (scenario == 14) initial = config.pre_pull_k * config.pull_current;
    if (scenario == 15) initial = config.unwinding_strong_current;
    reset(scenario == 6 ? MOTOR_BRAKING : MOTOR_CURRENT, initial);
    record(names[scenario], -200);
    record(names[scenario], -1);

    switch (scenario) {
    case 0:
    case 1:
        smooth_motor_current(test_position, test_position, config.pre_pull_k * config.pull_current);
        break;
    case 2:
    case 6:
    case 9:
    case 10:
        smooth_motor_current(test_position, test_position, config.unwinding_current);
        break;
    case 3:
        smooth_motor_current(test_position, test_position, config.unwinding_strong_current);
        break;
    case 7:
        smooth_motor_current(test_position, test_position, config.takeoff_pull_k * config.pull_current);
        break;
    case 8:
        smooth_motor_current(test_position, test_position, config.pull_current);
        break;
    case 11:
        smooth_motor_current(test_position, test_position, config.fast_pull_k * config.pull_current);
        break;
    default:
        smooth_motor_brake(test_position, test_position, config.manual_brake_current);
        break;
    }
    record(names[scenario], 0);

    for (loop_step = 1; loop_step <= end_ms; loop_step++) {
        tick();
        assert(isfinite(output_current));
        record(names[scenario], loop_step);
    }
}

static void assert_instant(smooth_motor_mode mode, float current) {
    assert(output_mode == mode && output_current == current);
    assert(!smooth_stage.active && next_smooth_motor_adjustment == 0);
}

// Verify the full direct curve, its deadline, and the final motor mode.
static void check_transition(float start, float target, bool braking, bool smooth) {
    float final_drive = braking ? copysignf(target, start) : target;
    int duration = braking ? config.braking_applying_period : config.pull_applying_period;
    reset(MOTOR_CURRENT, start);
    if (braking)
        smooth_motor_brake(test_position, test_position, target);
    else
        smooth_motor_current(test_position, test_position, target);
    if (!smooth) {
        assert_instant(braking ? MOTOR_BRAKING : MOTOR_CURRENT, target);
        return;
    }
    assert(output_current == start && smooth_stage.active);
    for (int time = 0; time < duration; time++) {
        advance_to(time);
        int sample_ms = time / smooth_max_step_delay * smooth_max_step_delay;
        double u = (double)sample_ms / duration;
        double expected = start + (final_drive - (double)start) * (1.0 - pow(1.0 - u, smooth_motor_p));
        assert(output_mode == MOTOR_CURRENT && smooth_stage.active);
        assert(fabs(output_current - expected) < 2e-4);
    }
    advance_to(duration);
    assert_instant(braking ? MOTOR_BRAKING : MOTOR_CURRENT, target);
}

static void check_edge_cases(void) {
    float pre = config.pre_pull_k * config.pull_current;
    int pull_ms = config.pull_applying_period;
    int brake_ms = config.braking_applying_period;
    for (int sign = -1; sign <= 1; sign += 2) {
        float normal = (float)sign * config.unwinding_current;
        float strong = (float)sign * config.unwinding_strong_current;
        float boundary = (float)sign * pre;
        float pull = (float)sign * config.pull_current;
        float above_pre = (float)sign * nextafterf(pre, INFINITY);
        float below_pre = (float)sign * nextafterf(pre, 0.0f);

        // Magnitude increases up to and including PrePull are instant.
        check_transition(normal, strong, false, 0);
        check_transition(normal, boundary, false, 0);
        check_transition(normal, -strong, false, 0);
        check_transition(boundary, boundary, false, 0);
        // Every decrease is smooth, including small currents and the boundary.
        check_transition(strong, normal, false, 1);
        check_transition(boundary, normal, false, 1);
        check_transition(boundary, 0.0f, false, 1);
        check_transition(normal, 0.0f, false, 1);
        check_transition(pull, boundary, false, 1);
        check_transition(above_pre, normal, false, 1);
        check_transition(pull, normal, false, 1);
        check_transition(pull, strong, false, 1);
        check_transition(pull, -normal, false, 1);
        // Larger rises start at the actual current. Large reversals stay smooth.
        check_transition(normal, above_pre, false, 1);
        check_transition(normal, pull, false, 1);
        check_transition(pull, -pull, false, 1);

        // Brake ramps retain the drive-current sign until the final mode switch.
        check_transition(pull, config.manual_brake_current, true, 1);
        check_transition(boundary, config.manual_brake_current, true, 1);
        check_transition(below_pre, config.manual_brake_current, true, 0);
        check_transition(normal, config.manual_brake_current, true, 0);
        check_transition(strong, config.manual_brake_current, true, 0);
        check_transition(above_pre, config.manual_brake_current, true, 1);
        check_transition(pull, pre, true, 1);
        check_transition(normal, pre, true, 0);
        check_transition(boundary, pre, true, 0);
        check_transition(normal, fabsf(above_pre), true, 1);

        // A brake ramp from PrePull must survive repeated commands below it,
        // including a pause imposed at the start of the transition.
        reset(MOTOR_CURRENT, boundary);
        measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
        for (int time = 0; time <= smooth_max_step_delay; time++) {
            advance_to(time);
            smooth_motor_brake(test_position, test_position, config.manual_brake_current);
            assert(smooth_stage.active && smooth_stage.paused);
            assert(output_mode == MOTOR_CURRENT && output_current == boundary);
        }
        measurement.filtered_acceleration_tac_mss = 0.0f;
        advance_to(2 * smooth_max_step_delay);
        systime_t resumed_start = smooth_stage.start_time;
        int finish = 2 * smooth_max_step_delay + brake_ms;
        for (int time = 2 * smooth_max_step_delay; time < finish; time++) {
            advance_to(time);
            smooth_motor_brake(test_position, test_position, config.manual_brake_current);
            assert(smooth_stage.active && output_mode == MOTOR_CURRENT);
            assert(smooth_stage.start_time == resumed_start);
        }
        advance_to(finish);
        assert_instant(MOTOR_BRAKING, config.manual_brake_current);

        reset(MOTOR_BRAKING, config.manual_brake_current);
        smooth_motor_current(test_position, test_position, strong);
        assert_instant(MOTOR_CURRENT, strong);
    }

    // Repeated commands preserve the curve, even below the PrePull threshold.
    for (int braking = 0; braking <= 1; braking++) {
        int duration = braking ? brake_ms : pull_ms;
        float goal = braking ? config.manual_brake_current : config.unwinding_current;
        reset(MOTOR_CURRENT, -config.pull_current);
        for (int time = 0; time < duration; time++) {
            advance_to(time);
            if (braking)
                smooth_motor_brake(test_position, test_position, goal);
            else
                smooth_motor_current(test_position, test_position, -goal);
            assert(smooth_stage.active && output_mode == MOTOR_CURRENT);
            assert(smooth_stage.start_time == 0);
        }
        advance_to(duration);
        assert_instant(braking ? MOTOR_BRAKING : MOTOR_CURRENT, braking ? goal : -goal);
    }

    // Any changed small target starts a fresh full curve at the current setpoint.
    for (int reverse = 0; reverse <= 1; reverse++) {
        float first = reverse ? config.unwinding_current : config.unwinding_strong_current;
        float last = reverse ? config.unwinding_strong_current : config.unwinding_current;
        reset(MOTOR_CURRENT, config.pull_current);
        smooth_motor_current(test_position, test_position, first);
        advance_to(pull_ms / 4);
        float previous = output_current;
        smooth_motor_current(test_position, test_position, last);
        assert(smooth_stage.start_time == test_time && output_current == previous);
        assert(smooth_stage.target_current == last);
        advance_to(pull_ms + pull_ms / 4 - 1);
        assert(smooth_stage.active);
        advance_to(pull_ms + pull_ms / 4);
        assert_instant(MOTOR_CURRENT, last);
    }

    // A changed brake magnitude follows the same rule, retaining the initial sign.
    reset(MOTOR_CURRENT, -config.pull_current);
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    advance_to(brake_ms / 4);
    float previous = output_current;
    float new_brake = config.manual_brake_current * 0.5f;
    smooth_motor_brake(test_position, test_position, new_brake);
    assert(smooth_stage.start_time == test_time && output_current == previous);
    assert(smooth_stage.target_current == -new_brake);
    advance_to(brake_ms + brake_ms / 4 - 1);
    assert(smooth_stage.active);
    advance_to(brake_ms + brake_ms / 4);
    assert_instant(MOTOR_BRAKING, new_brake);

    // Retargeting below PrePull also starts directly from the actual setpoint.
    reset(MOTOR_CURRENT, config.pull_current);
    smooth_motor_current(test_position, test_position, config.unwinding_strong_current);
    advance_to(3 * pull_ms / 4);
    previous = output_current;
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    assert(smooth_stage.start_time == test_time && output_current == previous);
    advance_to(pull_ms + 3 * pull_ms / 4 - 1);
    assert(smooth_stage.active);
    advance_to(pull_ms + 3 * pull_ms / 4);
    assert_instant(MOTOR_CURRENT, config.unwinding_current);

    // A new brake request within the small-current range is instantaneous.
    // Repeats of an existing request must still finish its curve (checked above).
    reset(MOTOR_CURRENT, -config.pull_current);
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    advance_to(3 * brake_ms / 4);
    assert(smooth_stage.active && fabsf(output_current) < pre);
    smooth_motor_brake(test_position, test_position, config.manual_brake_current * 0.5f);
    assert_instant(MOTOR_BRAKING, config.manual_brake_current * 0.5f);

    // A new higher target within PrePull is instant, even during an active descent.
    reset(MOTOR_CURRENT, pre);
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    advance_to(pull_ms / 2);
    smooth_motor_current(test_position, test_position, pre);
    assert_instant(MOTOR_CURRENT, pre);

    // Retargeting to PrePull follows the same fresh-curve rule.
    reset(MOTOR_CURRENT, config.pull_current);
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    advance_to(pull_ms / 4);
    smooth_motor_current(test_position, test_position, pre);
    assert(smooth_stage.start_time == test_time);
    advance_to(pull_ms + pull_ms / 4);
    assert_instant(MOTOR_CURRENT, pre);

    // A target above PrePull restarts from the current setpoint.
    reset(MOTOR_CURRENT, -config.pull_current);
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    advance_to(brake_ms / 4);
    previous = output_current;
    new_brake = (config.pull_current + pre) * 0.5f;
    smooth_motor_brake(test_position, test_position, new_brake);
    assert(smooth_stage.start_time == test_time && output_current == previous);
    assert(smooth_stage.target_current == -new_brake);
    advance_to(brake_ms + brake_ms / 4);
    assert_instant(MOTOR_BRAKING, new_brake);

    // A delayed update catches up without extending the curve, including wraparound.
    for (int wrap = 0; wrap <= 1; wrap++) {
        reset(MOTOR_CURRENT, config.pull_current);
        test_time = wrap ? UINT32_MAX - MS2ST(1000) : 0;
        smooth_motor_current(test_position, test_position, config.unwinding_current);
        test_time += MS2ST(pull_ms / 2);
        smooth_motor_adjustment(test_position, test_position);
        double expected = config.pull_current + (config.unwinding_current - (double)config.pull_current) *
                (1.0 - pow(0.5, smooth_motor_p));
        assert(smooth_stage.active);
        assert(fabs(output_current - expected) < 2e-4);
        test_time += MS2ST(pull_ms);
        smooth_motor_adjustment(test_position, test_position);
        assert_instant(MOTOR_CURRENT, config.unwinding_current);
    }

    // Zero duration, release, speed, and near-zero braking retain their overrides.
    config.pull_applying_period = 0;
    reset(MOTOR_CURRENT, config.pull_current);
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    assert_instant(MOTOR_CURRENT, config.unwinding_current);
    config.pull_applying_period = pull_ms;
    reset(MOTOR_CURRENT, config.pull_current);
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    smooth_motor_release();
    assert_instant(MOTOR_RELEASED, 0.0f);
    smooth_motor_current(test_position, test_position, config.pull_current);
    smooth_motor_speed(1000.0f);
    assert(output_mode == MOTOR_SPEED && !smooth_stage.active);
    reset(MOTOR_CURRENT, config.pull_current);
    expect_braking_zone_message = true;
    smooth_motor_brake(0, 0, config.manual_brake_current);
    expect_braking_zone_message = false;
    assert_instant(MOTOR_BRAKING, config.manual_brake_current);
    fprintf(stderr, "PASS signed direct curves, small decreases, PrePull boundaries, repeated commands, retargets, late updates, cancellation\n");
}

static void check_acceleration_pause(void) {
    const int step = smooth_max_step_delay;
    for (int sign = -1; sign <= 1; sign += 2) {
        for (int braking = 0; braking <= 1; braking++) {
            int duration = braking ? config.braking_applying_period : config.pull_applying_period;
            float goal = braking ? config.manual_brake_current : sign * config.unwinding_current;
            reset(MOTOR_CURRENT, sign * config.pull_current);
            if (braking) smooth_motor_brake(test_position, test_position, goal);
            else smooth_motor_current(test_position, test_position, goal);
            advance_to(step);
            float held = output_current;
            measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
            advance_to(5 * step);
            assert(smooth_stage.paused && output_current == held);
            // A repeat must not restart the ramp or bypass a pause.
            if (braking) smooth_motor_brake(test_position, test_position, goal);
            else smooth_motor_current(test_position, test_position, goal);
            assert(smooth_stage.paused && output_current == held);
            measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss;
            advance_to(6 * step);
            assert(smooth_stage.paused && output_current == held);
            measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss + 0.1f;
            advance_to(7 * step);
            assert(!smooth_stage.paused && output_current == held);
            advance_to(8 * step);
            assert(fabsf(output_current) < fabsf(held));
            assert(smooth_stage.start_time == MS2ST(6 * step));
            advance_to(duration / 2 + 6 * step);
            held = output_current;
            // A second pause can pass the original deadline without completing.
            measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
            advance_to(2 * duration + 6 * step);
            assert(smooth_stage.active && smooth_stage.paused && output_current == held);
            measurement.filtered_acceleration_tac_mss = 0.0f;
            advance_to(2 * duration + 7 * step);
            assert(output_current == held && smooth_stage.active);
            int finish = 2 * duration + 7 * step + duration - duration / 2;
            advance_to(finish - 1);
            assert(smooth_stage.active && output_mode == MOTOR_CURRENT);
            advance_to(finish);
            assert_instant(braking ? MOTOR_BRAKING : MOTOR_CURRENT, goal);
        }
    }
    // Equality alone does not start a pause; it only preserves an existing pause.
    reset(MOTOR_CURRENT, config.pull_current);
    measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss;
    smooth_motor_current(test_position, test_position, config.unwinding_current);
    advance_to(step);
    assert(!smooth_stage.paused && output_current < config.pull_current);

    // A fresh higher target and immediate overrides can cancel a held descent.
    measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
    advance_to(2 * step);
    assert(smooth_stage.paused);
    smooth_motor_current(test_position, test_position, config.pull_current);
    assert(!smooth_stage.paused);
    advance_to(config.pull_applying_period + 2 * step);
    assert_instant(MOTOR_CURRENT, config.pull_current);
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    assert(smooth_stage.paused);
    smooth_motor_release();
    assert_instant(MOTOR_RELEASED, 0.0f);
    reset(MOTOR_CURRENT, config.unwinding_current);
    measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    assert_instant(MOTOR_BRAKING, config.manual_brake_current);

    // A long pause across system-clock wraparound cannot expire a stage.
    reset(MOTOR_CURRENT, config.pull_current);
    test_time = UINT32_MAX - MS2ST(50);
    measurement.filtered_acceleration_tac_mss = config.antisex_acceleration_off_mss - 1.0f;
    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
    test_time += MS2ST(3 * config.braking_applying_period);
    smooth_motor_adjustment(test_position, test_position);
    assert(smooth_stage.active && output_current == config.pull_current);
    measurement.filtered_acceleration_tac_mss = 0.0f;
    test_time += MS2ST(step);
    smooth_motor_adjustment(test_position, test_position);
    assert(output_current == config.pull_current);
    test_time += MS2ST(2 * config.braking_applying_period);
    smooth_motor_adjustment(test_position, test_position);
    assert_instant(MOTOR_BRAKING, config.manual_brake_current);
    fprintf(stderr, "PASS acceleration pause: both signs/modes, repeated pauses, threshold equality, resume without catch-up, repeats, overrides, clock wrap\n");
}

static void check_configured_acceleration_threshold(void) {
    float original = config.antisex_acceleration_off_mss;
    const int step = smooth_max_step_delay;
    for (int sign = -1; sign <= 1; sign += 2) {
        for (int braking = 0; braking <= 1; braking++) {
            reset(MOTOR_CURRENT, sign * config.pull_current);
            // At -2 m/s², the INI threshold -3 allows the next decrease.
            config.antisex_acceleration_off_mss = -3.0f;
            measurement.filtered_acceleration_tac_mss = -2.0f;
            if (braking)
                smooth_motor_brake(test_position, test_position, config.manual_brake_current);
            else
                smooth_motor_current(test_position, test_position, sign * config.unwinding_current);
            advance_to(step);
            assert(smooth_stage.active && !smooth_stage.paused);
            assert(fabsf(output_current) < config.pull_current);
            float held = output_current;

            // Changing only the config threshold must immediately change the decision.
            config.antisex_acceleration_off_mss = -1.0f;
            advance_to(2 * step);
            assert(smooth_stage.paused && output_current == held);
            config.antisex_acceleration_off_mss = -3.0f;
            advance_to(3 * step);
            assert(!smooth_stage.paused && output_current == held);
            advance_to(4 * step);
            assert(fabsf(output_current) < fabsf(held));
        }
    }
    config.antisex_acceleration_off_mss = original;
    fprintf(stderr, "PASS runtime acceleration_off_mss changes: both signs and modes, hold/resume without catch-up\n");
}

static void check_wall_clock_ramps(void) {
    const int loop_us[] = {1000, 1500, 2000, 3000};
    for (unsigned profile = 0; profile < sizeof(loop_us) / sizeof(loop_us[0]); profile++) {
        for (int wrap = 0; wrap <= 1; wrap++) {
            for (int braking = 0; braking <= 1; braking++) {
                reset(MOTOR_CURRENT, config.takeoff_pull_k * config.pull_current);
                systime_t started = wrap ? UINT32_MAX - MS2ST(1000) : 0;
                test_time = started;
                if (braking)
                    smooth_motor_brake(test_position, test_position, config.manual_brake_current);
                else
                    smooth_motor_current(test_position, test_position, config.pull_current);
                systime_t duration = MS2ST(braking ? config.braking_applying_period : config.pull_applying_period);
                systime_t iteration = (systime_t)loop_us[profile] * TEST_CLOCK_HZ / 1000000U;
                systime_t updated = started;
                while (smooth_stage.active) {
                    loop_step++;
                    test_time += iteration;
                    if (smooth_motor_adjustment_due()) {
                        assert(test_time - updated <= MS2ST(smooth_max_step_delay) + iteration);
                        smooth_motor_adjustment(test_position, test_position);
                        updated = test_time;
                    }
                    assert(test_time - started < duration + iteration);
                }
                assert(test_time - started >= duration);
                assert_instant(braking ? MOTOR_BRAKING : MOTOR_CURRENT,
                               braking ? config.manual_brake_current : config.pull_current);
                if (!wrap)
                    fprintf(stderr, "PASS wall clock %s, loop %.1fms: completion %ums\n",
                            braking ? "braking" : "pull", (double)loop_us[profile] / 1000.0,
                            (unsigned)ST2MS(test_time - started));
            }
        }
    }
}

static void check_unwinding_timer(void) {
    const int loop_us[] = {1000, 1500, 2000, 3000};
    for (unsigned profile = 0; profile < sizeof(loop_us) / sizeof(loop_us[0]); profile++) {
        for (int wrap = 0; wrap <= 1; wrap++) {
            reset(MOTOR_CURRENT, config.takeoff_pull_k * config.pull_current);
            systime_t started = wrap ? UINT32_MAX - MS2ST(1000) : 0;
            test_time = started;
            unwinding_start_time = started;
            measurement.filtered_speed_tac_ms = -2.0f;
            assert(unwinding_current() == config.unwinding_strong_current);
            smooth_motor_current(test_position, test_position, unwinding_current());
            systime_t last_measurement = started;
            systime_t normal_at = 0;
            bool changed = false;
            for (loop_step = 1; loop_step <= 20000; loop_step++) {
                test_time += (systime_t)loop_us[profile] * TEST_CLOCK_HZ / 1000000U;
                if (test_time - last_measurement >= MS2ST(5)) {
                    last_measurement = test_time;
                    float goal = unwinding_current();
                    if (target_motor_state.param.current != goal) {
                        assert(!changed && goal == config.unwinding_current);
                        normal_at = test_time - started;
                        changed = true;
                        smooth_motor_current(test_position, test_position, goal);
                    }
                }
                if (smooth_motor_adjustment_due())
                    smooth_motor_adjustment(test_position, test_position);
                if (!smooth_stage.active && changed) break;
            }
            assert(loop_step <= 20000 && changed);
            assert(normal_at >= MS2ST(strong_unwinding_period));
            assert(normal_at <= MS2ST(strong_unwinding_period + 8));
            systime_t finished = test_time - started;
            systime_t expected_end = normal_at + MS2ST(config.pull_applying_period);
            systime_t iteration = (systime_t)loop_us[profile] * TEST_CLOCK_HZ / 1000000U;
            assert(finished >= expected_end && finished < expected_end + iteration);
            assert_instant(MOTOR_CURRENT, config.unwinding_current);
            if (!wrap)
                fprintf(stderr, "PASS Takeoff -> Unwinding, loop %.1fms: normal target %ums, completion %ums\n",
                        (double)loop_us[profile] / 1000.0, (unsigned)ST2MS(normal_at),
                        (unsigned)ST2MS(test_time - started));
        }
    }

    // Speed can still request strong unwinding after the initial two seconds.
    reset(MOTOR_CURRENT, config.unwinding_current);
    unwinding_start_time = 0;
    test_time = MS2ST(strong_unwinding_period + 100);
    measurement.filtered_speed_tac_ms = 0.0f;
    assert(unwinding_current() == config.unwinding_strong_current);
    measurement.filtered_speed_tac_ms = -2.0f;
    assert(unwinding_current() == config.unwinding_current);
}

int main(int argc, char **argv) {
    if (argc != 12) {
        fprintf(stderr, "Expected: pull_kg pull_ms brake_ms amps_per_kg pre_k unwind_kg strong_kg brake_kg takeoff_k fast_k acceleration_off_mss\n");
        return 2;
    }
    config.amps_per_kg = strtof(argv[4], NULL);
    config.pull_current = strtof(argv[1], NULL) * config.amps_per_kg;
    config.pull_applying_period = atoi(argv[2]);
    config.braking_applying_period = atoi(argv[3]);
    config.pre_pull_k = strtof(argv[5], NULL);
    config.unwinding_current = strtof(argv[6], NULL) * config.amps_per_kg;
    config.unwinding_strong_current = strtof(argv[7], NULL) * config.amps_per_kg;
    config.manual_brake_current = strtof(argv[8], NULL) * config.amps_per_kg;
    config.takeoff_pull_k = strtof(argv[9], NULL);
    config.fast_pull_k = strtof(argv[10], NULL);
    config.antisex_acceleration_off_mss = strtof(argv[11], NULL);
    config.unwinding_strong_erpm = -1.0f;
    config.braking_length = 10;
    config.slowing_length = 5;

    check_edge_cases();
    check_acceleration_pause();
    check_configured_acceleration_threshold();
    check_wall_clock_ramps();
    check_unwinding_timer();
    int end_ms = config.pull_applying_period;
    if (end_ms < config.braking_applying_period)
        end_ms = config.braking_applying_period;
    end_ms = 2 * end_ms + 500;
    printf("scenario,time_ms,mode,current_a,force_kg,target_kg,stage_active,stage_start_ticks,stage_duration_ms,stage_start_kg,stage_target_kg\n");
    for (int scenario = 0; scenario < 16; scenario++)
        run_scenario(scenario, end_ms);
    return 0;
}

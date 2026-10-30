/* Reuse the production functions and hardware stubs used to plot linear ramps. */
#define main rate_trace_main
#include "rate_harness.c"
#undef main

static const int position = 1000000;

static void reset_current(float current) {
    config.amps_per_kg = 1;
    config.pull_current = 100;
    config.unwinding_current = 2.2f;
    config.pull_applying_period = 4000;
    config.braking_applying_period = 2000;
    config.braking_length = 10;
    config.slowing_length = 5;
    config.antisex_acceleration_off_mss = -3;
    measurement.filtered_acceleration_tac_mss = -4;
    loop_step = 0;
    target_motor_state.mode = MOTOR_CURRENT;
    target_motor_state.param.current = current;
    smooth_motor_instant_current();
    loop_step = 1000;
}

static void tick(int time) {
    loop_step = time;
    if (loop_step >= next_smooth_motor_adjustment)
        smooth_motor_adjustment(position, position);
}

static void check_hold_resume(float sign, bool braking) {
    reset_current(sign*20);
    if (braking)
        smooth_motor_brake(position, position, 1.8f);
    else
        smooth_motor_current(position, position, sign*5);
    assert(smooth_motor_paused && output_current == sign*20);
    for (int t = 1001; t <= 2000; ++t)
        tick(t);
    assert(output_mode == MOTOR_CURRENT && output_current == sign*20);
    // Repeated commands retain the pause, including at threshold equality.
    measurement.filtered_acceleration_tac_mss = -3;
    loop_step = 2020;
    if (braking)
        smooth_motor_brake(position, position, 1.8f);
    else
        smooth_motor_current(position, position, sign*5);
    assert(smooth_motor_paused && output_current == sign*20);
    // A late resume must not consume the time spent waiting.
    measurement.filtered_acceleration_tac_mss = -2;
    tick(10000);
    assert(!smooth_motor_paused && output_current == sign*20);
    tick(10020);
    float step = braking ? 1.0f : 0.5f;
    assert(fabsf(output_current-sign*(20-step)) < 1e-5f);
    // A second pause and a live threshold change follow the same rules.
    float held = output_current;
    measurement.filtered_acceleration_tac_mss = -4;
    tick(10040);
    assert(smooth_motor_paused && output_current == held);
    config.antisex_acceleration_off_mss = -5;
    tick(10060);
    assert(!smooth_motor_paused && output_current == held);
    tick(10080);
    assert(fabsf(output_current-(held-sign*step)) < 1e-5f);
}

static void check_increase_and_equality(float sign) {
    reset_current(sign*20);
    smooth_motor_current(position, position, sign*5);
    assert(smooth_motor_paused);
    loop_step += 20;
    smooth_motor_current(position, position, sign*40);
    assert(!smooth_motor_paused && fabsf(output_current) > 20);
    reset_current(sign*20);
    measurement.filtered_acceleration_tac_mss = -3;
    smooth_motor_current(position, position, sign*5);
    assert(!smooth_motor_paused && fabsf(output_current) < 20);
    reset_current(sign*20);
    smooth_motor_current(position, position, sign*20);
    assert(!smooth_motor_paused && next_smooth_motor_adjustment == INT_MAX);
}

static void check_final_step_and_overrides(void) {
    reset_current(2.3f);
    smooth_motor_brake(position, position, 1.8f);
    assert(smooth_motor_paused && output_mode == MOTOR_CURRENT && output_current == 2.3f);
    measurement.filtered_acceleration_tac_mss = -2;
    tick(1020);
    assert(output_mode == MOTOR_CURRENT && output_current == 2.3f);
    tick(1040);
    assert(output_mode == MOTOR_BRAKING && output_current == 1.8f && !smooth_motor_paused);

    reset_current(1);
    smooth_motor_current(position, position, .5f);
    assert(smooth_motor_paused && output_current == 1);
    measurement.filtered_acceleration_tac_mss = -2;
    tick(1020);
    assert(output_current == 1);
    tick(1040);
    assert(output_current == .5f && !smooth_motor_paused);

    reset_current(20);
    smooth_motor_current(position, position, 5);
    smooth_motor_brake(0, 0, 1.8f);
    assert(output_mode == MOTOR_BRAKING && output_current == 1.8f && !smooth_motor_paused);
    reset_current(20);
    smooth_motor_current(position, position, 5);
    smooth_motor_release();
    assert(output_mode == MOTOR_RELEASED && !smooth_motor_paused);
    reset_current(20);
    smooth_motor_current(position, position, 5);
    smooth_motor_speed(100);
    assert(output_mode == MOTOR_SPEED && !smooth_motor_paused);
}

int main(void) {
    for (int sign = -1; sign <= 1; sign += 2) {
        check_hold_resume(sign, false);
        check_hold_resume(sign, true);
        check_increase_and_equality(sign);
    }
    check_final_step_and_overrides();
    puts("PASS current/prebrake holds, both signs, equality, repeated commands, late resume, live threshold, growth, final step and instant overrides");
    return 0;
}

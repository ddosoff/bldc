/* Execute the rate-based production controller; Python supplies acceleration. */
#define main rate_trace_main
#include "rate_harness.c"
#undef main

static float requested_current;
static bool requested_braking;

void controller_reset(float pull_kg, float amps_per_kg, float pre_k,
                      int pull_ms, int brake_ms, float target_kg, int braking_target,
                      float initial_factor, float acceleration_off_mss) {
    (void)pre_k;
    config.amps_per_kg = amps_per_kg;
    config.pull_current = pull_kg*amps_per_kg;
    config.pull_applying_period = pull_ms;
    config.braking_applying_period = brake_ms;
    config.braking_length = 10;
    config.slowing_length = 5;
    config.antisex_acceleration_off_mss = acceleration_off_mss;
    antisex_amps = 0;
    antisex_amps_gain = 1;
    measurement.filtered_acceleration_tac_mss = 0;
    requested_current = target_kg*amps_per_kg;
    requested_braking = braking_target != 0;
    loop_step = 0;
    target_motor_state.mode = MOTOR_CURRENT;
    target_motor_state.param.current = initial_factor*config.pull_current;
    smooth_motor_instant_current();
}

void controller_set_unwinding(float kg) {
    config.unwinding_current = kg*config.amps_per_kg;
}

float controller_tick(int time_ms, float acceleration) {
    float previous_current = output_current;
    smooth_motor_mode previous_mode = output_mode;
    // The starting current was settled before the transition command.
    loop_step = 1000+time_ms;
    measurement.filtered_acceleration_tac_mss = acceleration;
    if (time_ms == 0) {
        if (requested_braking)
            smooth_motor_brake(1000000, 1000000, requested_current);
        else
            smooth_motor_current(1000000, 1000000, requested_current);
    }
    if (loop_step >= next_smooth_motor_adjustment)
        smooth_motor_adjustment(1000000, 1000000);
    // Check the exact signal used by C, not the later 5 ms plot sample.
    if (previous_mode == MOTOR_CURRENT && output_mode == MOTOR_CURRENT &&
        fabsf(output_current) < fabsf(previous_current)-1e-5f)
        assert(acceleration >= config.antisex_acceleration_off_mss);
    return output_current/config.amps_per_kg;
}

int controller_braking(void) { return output_mode == MOTOR_BRAKING; }
int controller_paused(void) { return smooth_motor_paused; }
int controller_active(void) { return next_smooth_motor_adjustment != INT_MAX; }

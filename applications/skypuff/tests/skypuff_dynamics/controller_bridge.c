/* Execute the same production-code extraction and host interfaces as the tests.
 * Python supplies system time and filtered acceleration; C owns the entire ramp.
 */
#define main smooth_harness_main
#include "../skypuff_smooth/harness.c"
#undef main

static bool controller_target_braking;
static float controller_target_current;

void controller_reset(float pull_kg, float amps_per_kg, float pre_k,
                      int pull_ms, int brake_ms, float target_kg, int braking_target,
                      float initial_factor, float acceleration_off_mss) {
    config.amps_per_kg = amps_per_kg;
    config.pull_current = pull_kg * amps_per_kg;
    config.pre_pull_k = pre_k;
    config.pull_applying_period = pull_ms;
    config.braking_applying_period = brake_ms;
    controller_target_current = target_kg * amps_per_kg;
    controller_target_braking = braking_target != 0;
    config.antisex_acceleration_off_mss = acceleration_off_mss;
    config.braking_length = 10;
    config.slowing_length = 5;
    // Match the production PrePull/FastPull expression at the exact boundary.
    reset(MOTOR_CURRENT, initial_factor * config.pull_current);
}

float controller_tick(int time_ms, float acceleration) {
    loop_step = time_ms;
    test_time = MS2ST(time_ms);
    measurement.filtered_acceleration_tac_mss = acceleration;
    if (time_ms == 0) {
        if (controller_target_braking)
            smooth_motor_brake(test_position, test_position, controller_target_current);
        else
            smooth_motor_current(test_position, test_position, controller_target_current);
    } else if (smooth_motor_adjustment_due())
        smooth_motor_adjustment(test_position, test_position);
    return output_current / config.amps_per_kg;
}

int controller_braking(void) { return output_mode == MOTOR_BRAKING; }
int controller_paused(void) { return smooth_stage.active && smooth_stage.paused; }
int controller_active(void) { return smooth_stage.active; }

/* Exercise target changes during the initial Unwinding descent. */
#define main smooth_harness_main
#include "harness.c"
#undef main

int main(int argc, char **argv) {
    if (argc != 8 && argc != 10) {
        fprintf(stderr, "Expected: pull_kg amps_per_kg pre_k unwind_kg strong_kg pull_ms acceleration_off_mss [fast_unwind|fast_unwind_direct fast_k]\n");
        return 2;
    }
    bool fast_unwind = argc == 10;
    bool direct = fast_unwind && strcmp(argv[8], "fast_unwind_direct") == 0;
    if (fast_unwind && !direct && strcmp(argv[8], "fast_unwind") != 0)
        return 2;
    config.amps_per_kg = strtof(argv[2], NULL);
    config.pull_current = strtof(argv[1], NULL) * config.amps_per_kg;
    config.pre_pull_k = strtof(argv[3], NULL);
    config.unwinding_current = strtof(argv[4], NULL) * config.amps_per_kg;
    config.unwinding_strong_current = strtof(argv[5], NULL) * config.amps_per_kg;
    config.pull_applying_period = atoi(argv[6]);
    config.antisex_acceleration_off_mss = strtof(argv[7], NULL);
    config.unwinding_strong_erpm = -1.0f;
    config.fast_pull_k = fast_unwind ? strtof(argv[9], NULL) : 1.25f;
    reset(MOTOR_CURRENT, (fast_unwind ? config.fast_pull_k : config.pre_pull_k) * config.pull_current);
    // Isolate the initial strong-unwinding timer from the speed condition.
    measurement.filtered_speed_tac_ms = -2.0f;
    unwinding_start_time = 0;

    const char *name = direct ? "fast_pull_unwinding_direct" :
                       fast_unwind ? "fast_pull_unwinding" : "pre_pull_unwinding_pull";
    const int retarget_ms = direct ? 0 : fast_unwind ? strong_unwinding_period : 2000;
    const int finish_ms = retarget_ms + config.pull_applying_period;
    const float final_current = fast_unwind ? config.unwinding_current : config.pull_current;
    printf("scenario,time_ms,mode,current_a,force_kg,target_kg,stage_active,stage_start_ticks,stage_duration_ms,stage_start_kg,stage_target_kg\n");
    record(name, -200);
    record(name, -1);
    for (loop_step = 0; loop_step <= finish_ms + 500; loop_step++) {
        test_time = MS2ST(loop_step);
        // Production processes a new command before the due smoothing update.
        if (fast_unwind || loop_step < retarget_ms) {
            float goal = direct ? config.unwinding_current : unwinding_current();
            if (loop_step % 5 == 0 && target_motor_state.param.current != goal) {
                float previous = output_current;
                smooth_motor_current(test_position, test_position, goal);
                assert(output_current == previous && smooth_stage.active);
                assert(smooth_stage.start_current == previous);
            }
        } else if (loop_step == retarget_ms) {
            float previous = output_current;
            smooth_motor_current(test_position, test_position, config.pull_current);
            assert(output_current == previous && smooth_stage.active);
            assert(smooth_stage.start_current == previous);
        }
        if (smooth_motor_adjustment_due())
            smooth_motor_adjustment(test_position, test_position);
        assert(output_mode == MOTOR_CURRENT && isfinite(output_current));
        assert(!smooth_stage.paused);
        if (loop_step >= retarget_ms && loop_step < finish_ms) {
            assert(smooth_stage.active);
            assert(smooth_stage.start_time == MS2ST(retarget_ms));
        }
        if (loop_step >= finish_ms)
            assert_instant(MOTOR_CURRENT, final_current);
        record(name, loop_step);
    }
    fprintf(stderr, "PASS %s: continuous setpoint, fresh full period, target reached at %d ms\n", name, finish_ms);
    return 0;
}

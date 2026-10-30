/* Test the production tachometer measurement/filter function with known motion. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;
#define MS2ST(ms) ((systime_t)(ms) * (CH_CFG_ST_FREQUENCY / 1000U))
static systime_t test_time;
static systime_t chVTGetSystemTime(void) { return test_time; }
static systime_t chTimeDiffX(systime_t start, systime_t end) { return end - start; }
// A fine virtual encoder isolates filter response from hardware quantization.
static float tac_steps_to_meters(int steps) { return steps * 0.000001f; }
#include "production_measurement.inc"

static int check_motion(float period, double acceleration) {
    measurement_speed_filter_period_secs = period;
    measurement.time = 0;
    measurement.tac = 0;
    measurement.speed_tac_ms = measurement.filtered_speed_tac_ms = 7.0f;
    measurement.acceleration_tac_mss = measurement.filtered_acceleration_tac_mss = 0;
    measurement.updated = false;
    int first_below_minus_one = -1;
    const int sample_ms = measurement_delay / MS2ST(1);
    for (int ms = 1; ms <= 1000; ms++) {
        double t = ms / 1000.0;
        int tac = (int)lround((7*t + acceleration*t*t/2) * 1e6);
        test_time = MS2ST(ms);
        bool updated = measurement_tick(tac);
        assert(updated == (ms % sample_ms == 0));
        assert(measurement.updated == updated);
        assert(isfinite(measurement.filtered_speed_tac_ms));
        assert(isfinite(measurement.filtered_acceleration_tac_mss));
        if (first_below_minus_one < 0 && measurement.filtered_acceleration_tac_mss < -1)
            first_below_minus_one = ms;
    }
    assert(fabs(measurement.filtered_acceleration_tac_mss - acceleration) < .01);
    if (acceleration == 0)
        assert(fabs(measurement.filtered_speed_tac_ms - 7) < .0001);
    return first_below_minus_one;
}

int main(void) {
    float period = measurement_speed_filter_period_secs;
    assert(period > (float)measurement_delay / CH_CFG_ST_FREQUENCY);
    assert(check_motion(period, 0) == -1);
    int configured_ms = check_motion(period, -2);
    int slower_ms = check_motion(2*period, -2);
    assert(configured_ms > 0 && configured_ms < slower_ms);
    printf("PASS production measurement filter: %.0f ms vs %.0f ms; -1 threshold crossed at %d vs %d ms for -2 m/s^2; steady speed preserved\n",
           (double)period*1000, (double)period*2000, configured_ms, slower_ms);
    return 0;
}

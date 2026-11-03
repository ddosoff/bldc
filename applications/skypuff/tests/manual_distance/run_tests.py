#!/usr/bin/env python3
"""Host checks of the production manual-distance command and manual FSM cases."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[4]
HARNESS = r'''
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "production_types.inc"

static skypuff_state state;
static skypuff_main_loop_command main_loop_command;
static float manual_distance_m, terminal_manual_distance_m, erpm_filtered, prev_erpm;
static int manual_start_tac, loop_step, prev_print, prev_printed_tac;
static int brake_calls, brake_tac, rpm_reads, messages;
static const int timeout_reset_interval = 100;
static struct { float manual_slow_erpm, manual_slow_speed_up_current, manual_slow_max_current; }
    config = {1000, 10, 50};
static struct { float si_wheel_diameter, si_gear_ratio; int si_motor_poles; }
    drive = {0.2f, 2, 14}, *mc_conf = &drive;
static void commands_printf(const char *fmt, ...) { (void)fmt; messages++; }
static void save_custom_msg(const char *fmt, ...) { (void)fmt; }
static float erpm_to_ms(float rpm) { return rpm; }
static float mc_interface_get_rpm(void) { rpm_reads++; return 0; }
static float mc_interface_get_tot_current_directional_filtered(void) { return 0; }
static bool is_filtered_speed_too_low(float rpm) { (void)rpm; return false; }
static void save_pulling_too_high_error(float amps) { (void)amps; abort(); }
static void brake_or_manual_brake(int tac, int abs_tac) { (void)tac; (void)abs_tac; abort(); }
static void timeout_reset(void) {}
static void smooth_motor_speed(float rpm) { (void)rpm; }
static void pull_state(int tac, float amps, skypuff_state next, const char *msg) {
    (void)tac; (void)amps; (void)msg; state = next;
}
static void manual_brake(int tac) { brake_calls++; brake_tac = tac; state = MANUAL_BRAKING; }
#include "production_functions.inc"

static void check_parser(void) {
    const char *bad[] = {"", "abc", "3m", "1 2", "-1", "nan", "inf", "-inf", "1e100"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(*bad); i++) {
        const char *argv[] = {"manual_distance", bad[i]};
        main_loop_command = DO_NOTHING; terminal_manual_distance_m = 7; messages = 0;
        terminal_manual_distance(2, argv);
        assert(main_loop_command == DO_NOTHING && terminal_manual_distance_m == 7 && messages == 1);
    }
    const char *values[] = {"0", "2.5"};
    for (unsigned i = 0; i < 2; i++) {
        const char *argv[] = {"manual_distance", values[i], "extra"};
        main_loop_command = DO_NOTHING;
        terminal_manual_distance(1, argv);
        assert(main_loop_command == DO_NOTHING);
        terminal_manual_distance(3, argv);
        assert(main_loop_command == DO_NOTHING);
        terminal_manual_distance(2, argv);
        assert(main_loop_command == SET_MANUAL_DISTANCE);
        assert(terminal_manual_distance_m == (i ? 2.5f : 0));
    }
}

int main(void) {
    check_parser();
    // Exercise the production command case in every state, including disabling.
    for (skypuff_state s = UNINITIALIZED; s <= DISCONNECTED; s++) {
        bool allowed = s == UNINITIALIZED || s == BRAKING || s == BRAKING_EXTENSION || s == MANUAL_BRAKING;
        for (int enabled = 0; enabled <= 1; enabled++) {
            state = s; manual_distance_m = 7; terminal_manual_distance_m = enabled * 2.5f;
            main_loop_command = SET_MANUAL_DISTANCE;
            apply_distance_command();
            assert(manual_distance_m == (allowed ? terminal_manual_distance_m : 7));
        }
    }
    // One motor revolution is an exact boundary in the production conversion.
    const int steps = 42;
    float one_rev = meters_per_rev();
    const skypuff_state moving[] = {MANUAL_SLOW_SPEED_UP, MANUAL_SLOW_BACK_SPEED_UP, MANUAL_SLOW, MANUAL_SLOW_BACK};
    const int starts[] = {-200, -20, 0, 20, 200};
    for (unsigned s = 0; s < sizeof(moving) / sizeof(*moving); s++) {
        for (unsigned p = 0; p < sizeof(starts) / sizeof(*starts); p++) {
            for (int sign = -1; sign <= 1; sign += 2) {
                state = moving[s]; manual_start_tac = starts[p]; manual_distance_m = one_rev;
                assert(!manual_distance_reached(starts[p]));
                assert(!manual_distance_reached(starts[p] + sign * (steps - 1)));
                assert(manual_distance_reached(starts[p] + sign * steps));
                assert(manual_distance_reached(starts[p] + sign * (steps + 1)));
                brake_calls = 0; rpm_reads = 0;
                process_states(starts[p] + sign * (steps - 1), 0);
                assert(brake_calls == 0 && rpm_reads == 1 && state == moving[s]);
                process_states(starts[p] + sign * steps, 0);
                assert(brake_calls == 1 && brake_tac == starts[p] + sign * steps);
                assert(state == MANUAL_BRAKING && rpm_reads == 1);
            }
        }
    }
    for (skypuff_state s = UNINITIALIZED; s <= DISCONNECTED; s++) {
        state = s; manual_start_tac = 0; manual_distance_m = 0;
        assert(!manual_distance_reached(1000));
        brake_calls = 0;
        process_states(1000, 1000);
        assert(brake_calls == 0 && state == s);
        bool moving_state = s == MANUAL_SLOW_SPEED_UP || s == MANUAL_SLOW_BACK_SPEED_UP ||
                            s == MANUAL_SLOW || s == MANUAL_SLOW_BACK;
        manual_distance_m = one_rev;
        process_states(1000, 1000);
        assert(brake_calls == (moving_state ? 1 : 0));
        assert(state == (moving_state ? MANUAL_BRAKING : s));
    }
    // Starting again replaces the origin; changing from acceleration to cruise does not.
    manual_slow_speed_up(-20);
    assert(state == MANUAL_SLOW_SPEED_UP && manual_start_tac == -20);
    manual_slow(0, 1000);
    assert(state == MANUAL_SLOW && manual_start_tac == -20 && manual_distance_reached(22));
    manual_slow_back_speed_up(100);
    assert(state == MANUAL_SLOW_BACK_SPEED_UP && manual_start_tac == 100);
    manual_slow_back(80, -1000);
    assert(state == MANUAL_SLOW_BACK && manual_start_tac == 100 && manual_distance_reached(58));
    manual_start_tac = INT_MIN;
    assert(manual_distance_reached(INT_MAX)); // Subtraction must not overflow signed int.
    puts("manual_distance: parsing, command states, both directions, start/cruise and braking passed");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'gcc'))
    parser.add_argument('--output', type=Path, default=ROOT / 'build/skypuff_manual_distance_tests')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'applications/skypuff/app_skypuff.c').read_text()
    header = (ROOT / 'applications/skypuff/app_skypuff.h').read_text()
    types = [re.search(r'typedef enum\s*\{\s*' + first + r',.*?\} ' + name + ';', text, re.S)[0]
             for text, first, name in [(header, 'UNINITIALIZED', 'skypuff_state'),
                                       (source, 'DO_NOTHING', 'skypuff_main_loop_command')]]
    names = ('meters_per_rev', 'steps_per_rev', 'manual_slow_speed_up', 'manual_slow_back_speed_up',
             'speed_state', 'manual_slow', 'manual_slow_back', 'manual_distance_reached', 'terminal_manual_distance')
    functions = [re.search(r'^(?:inline static|static) [\w ]+\b' + name + r'\([^;]*?\) \{.*?^\}',
                           source, re.M | re.S)[0] for name in names]
    # Keep all four production manual case bodies. Other cases need no motor stubs;
    # verify their source and the FSM entry contain no distance checks before omitting them.
    fsm = re.search(r'^inline static void process_states\(.*?^\}', source, re.M | re.S)[0]
    cases = re.search(r'\t\tcase MANUAL_SLOW_SPEED_UP:.*?(?=\t\tcase PRE_PULL:)', fsm, re.S)[0]
    assert 'manual_distance_reached' not in fsm.replace(cases, '')
    functions.append('static void process_states(int cur_tac, int abs_tac) {\n'
                     'float cur_erpm, cur_current, abs_current;\nswitch(state) {\n' + cases +
                     '\ndefault: break;\n} }')
    case = source.split('\t\tcase SET_MANUAL_DISTANCE:', 1)[1].split('\n\t\tcase SET_MANUAL_SLOW:', 1)[0]
    functions.append('static void apply_distance_command(void) { switch(main_loop_command) {\n'
                     'case SET_MANUAL_DISTANCE:' + case + '\ndefault: abort();\n} }')
    (output / 'production_types.inc').write_text('\n'.join(types))
    (output / 'production_functions.inc').write_text('\n'.join(functions))
    (output / 'harness.c').write_text(HARNESS)
    executable = output / 'manual_distance'
    subprocess.run(shlex.split(args.cc) + ['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-fsanitize=undefined', '-fno-sanitize-recover=all',
        str(output / 'harness.c'), '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()

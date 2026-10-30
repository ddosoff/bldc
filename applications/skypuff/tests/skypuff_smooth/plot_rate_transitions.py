#!/usr/bin/env python3
"""Plot PrePull / Unwinding using the restored, rate-based production C code."""
import argparse
import csv
import hashlib
import json
import math
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "production_types.inc"

static struct {
    float pull_current, amps_per_kg, unwinding_current;
    float antisex_acceleration_off_mss;
    int pull_applying_period, braking_applying_period, braking_length, slowing_length;
} config;
static int loop_step, state;
static struct { float filtered_acceleration_tac_mss; } measurement;
static float antisex_amps = 0.0f, antisex_amps_gain = 1.0f, output_current;
static smooth_motor_mode output_mode;
static void mc_interface_set_current(float value) { output_current = value; output_mode = MOTOR_CURRENT; }
static void mc_interface_set_brake_current(float value) { output_current = value; output_mode = MOTOR_BRAKING; }
static void mc_interface_release_motor(void) { output_current = 0; output_mode = MOTOR_RELEASED; }
static void mc_interface_set_pid_speed(float value) { (void)value; output_mode = MOTOR_SPEED; }
static const char *state_str(int value) { (void)value; return "host"; }
static const char *motor_mode_str(smooth_motor_mode mode) { (void)mode; return "host"; }
static void save_custom_msg(const char *format, ...) {
    if (strstr(format, "braking/slowing zone detected")) return;
    abort();
}
#ifdef DEBUG_SMOOTH_MOTOR
static float erpm_to_ms(float value) { return value; }
static void commands_printf(const char *format, ...) { (void)format; }
#endif
#include "production_functions.inc"

static void record(const char *name, int time_ms) {
    assert(current_motor_state.mode == MOTOR_CURRENT);
    assert(output_mode == MOTOR_CURRENT);
    assert(isfinite(output_current));
    printf("%s,%d,%.9g,%.9g,%d\n", name, time_ms,
           (double)output_current, (double)(output_current / config.amps_per_kg),
           next_smooth_motor_adjustment != INT_MAX);
}

static void run(const char *name, float start, float target) {
    // The initial setpoint has been steady for 1000 control-loop iterations.
    loop_step = 0;
    target_motor_state.mode = MOTOR_CURRENT;
    target_motor_state.param.current = start;
    smooth_motor_instant_current();
    terminal_motor_state = current_motor_state;
    record(name, -200);
    record(name, -1);
    for (int t = 0; t <= 6000; ++t) {
        loop_step = 1000 + t;
        if (t == 0)
            smooth_motor_current(1000000, 1000000, target);
        if (loop_step >= next_smooth_motor_adjustment)
            smooth_motor_adjustment(1000000, 1000000);
        record(name, t);
    }
    assert(output_current == target);
}

int main(int argc, char **argv) {
    assert(argc == 6);
    config.amps_per_kg = strtof(argv[4], NULL);
    config.pull_current = strtof(argv[1], NULL) * config.amps_per_kg;
    config.unwinding_current = strtof(argv[3], NULL) * config.amps_per_kg;
    config.pull_applying_period = atoi(argv[5]);
    config.antisex_acceleration_off_mss = -3.0f;
    float pre = config.pull_current * strtof(argv[2], NULL);
    printf("scenario,time_ms,current_a,force_kgf,active\n");
    run("pre_pull_unwinding", pre, config.unwinding_current);
    run("unwinding_pre_pull", config.unwinding_current, pre);
    return 0;
}
'''


def prepare_production_code(output):
    output.mkdir(parents=True, exist_ok=True)
    source = (ROOT/'applications/skypuff/app_skypuff.c').read_text()
    header = (ROOT/'applications/skypuff/app_skypuff.h').read_text()
    assert 'smooth_stage' not in source, 'This runner is for the original rate-based controller'
    mode = re.search(r'typedef enum\s*\{\s*MOTOR_RELEASED,.*?\} smooth_motor_mode;', header, re.S)[0]
    constant = re.search(r'^const int smooth_max_step_delay = (\d+);', source, re.M)
    step_ms = int(constant[1])
    filter_ms = 1000*float(re.search(r'measurement_speed_filter_period_secs = ([0-9.]+)f;', source)[1])
    declarations = source.split('// Smooth force apply and release\n', 1)[1].split('\n/*', 1)[0]
    functions = source.split('// Smooth motor functions\n', 1)[1].split('\nstatic void save_custom_msg', 1)[0]
    functions += source.split('// Process all posible motor mode transitions', 1)[1].split(
        '// Helper functions to check limits', 1)[0]
    (output/'production_types.inc').write_text(mode+'\n'+constant[0]+'\n'+declarations)
    (output/'production_functions.inc').write_text(functions)
    (output/'rate_harness.c').write_text(HARNESS)
    measurement_ms = int(re.search(r'measurement_delay = MS2ST\((\d+)\)', source)[1])
    return dict(smooth_max_step_delay=step_ms, measurement_period_ms=measurement_ms,
                measurement_speed_filter_period_secs=filter_ms/1000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'gcc'))
    parser.add_argument('--output', type=Path, default=ROOT/'build/skypuff_rate_pre_unwind_140kg')
    parser.add_argument('--pull-kg', type=float, default=140.)
    parser.add_argument('--pre-k', type=float, default=.3)
    parser.add_argument('--unwind-kg', type=float, default=2.2)
    parser.add_argument('--amps-per-kg', type=float, default=2.45)
    parser.add_argument('--pull-ms', type=int, default=4000)
    args = parser.parse_args()
    assert 0 < args.unwind_kg < args.pull_kg*args.pre_k < args.pull_kg
    assert args.amps_per_kg > 0 and args.pull_ms > 0
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = (ROOT/'applications/skypuff/app_skypuff.c').read_text()
    constants = prepare_production_code(output)
    step_ms = constants['smooth_max_step_delay']
    filter_ms = constants['measurement_speed_filter_period_secs']*1000
    executable = output/'rate_transitions'
    subprocess.run(shlex.split(args.cc)+['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-fsanitize=undefined', '-fno-sanitize-recover=all',
        str(output/'rate_harness.c'), '-lm', '-o', str(executable)], check=True)
    with (output/'transitions.csv').open('w') as stream:
        subprocess.run([str(executable), *map(str, [args.pull_kg, args.pre_k, args.unwind_kg,
            args.amps_per_kg, args.pull_ms])], stdout=stream, check=True)
    traces = {name: [] for name in ('pre_pull_unwinding', 'unwinding_pre_pull')}
    with (output/'transitions.csv').open() as stream:
        for row in csv.DictReader(stream):
            name = row.pop('scenario')
            traces[name].append({key: float(value) for key, value in row.items()})
    pre = args.pull_kg*args.pre_k
    rate = args.pull_kg/(args.pull_ms/1000)
    completion = {}
    for name, rows in traces.items():
        start, target = (pre, args.unwind_kg) if name.startswith('pre_') else (args.unwind_kg, pre)
        direction = 1 if target > start else -1
        end_ms = next(row['time_ms'] for row in rows if row['time_ms'] >= 0 and not row['active'])
        # A settled initial setpoint allows the first full step at command time.
        ideal_end_ms = abs(target-start)/rate*1000-step_ms
        assert abs(end_ms-ideal_end_ms) < 2.1, (name, end_ms, ideal_end_ms)
        for before, after in zip(rows, rows[1:]):
            assert direction*(after['force_kgf']-before['force_kgf']) >= -1e-4
            assert min(start, target)-1e-4 <= after['force_kgf'] <= max(start, target)+1e-4
            assert math.isclose(after['current_a']/args.amps_per_kg, after['force_kgf'], abs_tol=1e-4)
            t = after['time_ms']
            if 0 <= t < end_ms:
                progress_ms = (t//step_ms+1)*step_ms
                expected = start+direction*min(abs(target-start), rate*progress_ms/1000)
                assert math.isclose(after['force_kgf'], expected, abs_tol=2e-4)
        assert math.isclose(rows[-1]['force_kgf'], target, abs_tol=1e-4)
        completion[name] = end_ms/1000
        print(f'PASS {name}: {start:g} -> {target:g} kgf; {end_ms/1000:g} s')

    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.4), sharex=True, sharey=True)
    fig.subplots_adjust(left=.07, right=.98, bottom=.23, top=.70, wspace=.12)
    fig.suptitle('PrePull ↔ Unwinding · исходный smooth_motor', fontsize=21, fontweight='bold', y=.97)
    fig.text(.5, .88, f'Номинал {args.pull_kg:g} кгс · PrePull {pre:g} кгс · Unwinding {args.unwind_kg:g} кгс',
             ha='center', fontsize=13)
    fig.text(.5, .82, f'pull_applying_period = {args.pull_ms/1000:g} с · скорость {rate:g} кгс/с · шаг {step_ms} циклов · фильтр {filter_ms:g} мс',
             ha='center', color='#536376')
    end = max(completion.values())+.3
    for ax, (name, rows), title, color in zip(axes, traces.items(),
            ('PrePull → Unwinding', 'Unwinding → PrePull'), ('#2463c5', '#d97720')):
        visible = [r for r in rows if r['time_ms']/1000 <= end]
        ax.step([r['time_ms']/1000 for r in visible], [r['force_kgf'] for r in visible],
                where='post', color=color, lw=2)
        ax.axvline(0, color='#9aa6b2', ls=':', lw=1)
        ax.axvline(completion[name], color='#25825a', ls='--', lw=1)
        ax.text(completion[name]+.025, pre*.54, f'Цель\n{completion[name]:.3f} с', color='#25825a')
        for force in (pre, args.unwind_kg):
            ax.axhline(force, color='#9aa6b2', ls=':', lw=.8)
        ax.set(title=title, xlabel='Время после команды, с', xlim=(-.12, end), ylim=(0, pre*1.13))
        ax.grid(alpha=.18)
    axes[0].set_ylabel('Уставка тяги, кгс')
    fig.text(.07, .13, 'Данные из текущего C-кода. Один цикл принят равным 1 мс; исходная уставка установилась заранее.',
             fontsize=10.5, color='#536376')
    fig.text(.07, .075, f'Фиксированная цель Unwinding {args.unwind_kg:g} кгс. Без динамики троса, Strong Unwinding и коррекции тока antisex.',
             fontsize=10.5, color='#536376')
    for ext in ('png', 'svg'):
        fig.savefig(output/f'pre_pull_unwinding.{ext}', dpi=170, facecolor='white')
    plt.close(fig)
    (output/'results.json').write_text(json.dumps(dict(
        settings=dict(pull_kg=args.pull_kg, pre_k=args.pre_k, unwind_kg=args.unwind_kg,
                      amps_per_kg=args.amps_per_kg, pull_ms=args.pull_ms, step_loops=step_ms,
                      loop_ms=1, filter_ms=filter_ms, initial_settled_loops=1000),
        completion_s=completion, rate_kgf_per_s=rate,
        checks='production C + UBSan, monotonicity, endpoints, rate and timing passed',
        production_source_sha256=hashlib.sha256(source.encode()).hexdigest()), indent=2)+'\n')
    print(f'Plot: {output / "pre_pull_unwinding.png"}')


if __name__ == '__main__':
    main()

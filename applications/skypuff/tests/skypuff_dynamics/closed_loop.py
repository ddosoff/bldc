#!/usr/bin/env python3
"""Couple production smooth_motor C code to the drum/rope model in real time."""
import argparse
import ctypes
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import numpy as np
from model import Parameters, CableDrum, integrate
from run_simulation import algebra_checks, compare, metrics, save_rows, SOURCES

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'skypuff_smooth'))
from run_tests import prepare_production_code


def build_controller(output, compiler):
    constants = prepare_production_code(output)
    library = output/'smooth_controller.so'
    subprocess.run(shlex.split(compiler) + ['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-shared', '-fPIC', '-I', str(output),
        str(Path(__file__).with_name('controller_bridge.c')), '-lm', '-o', str(library)], check=True)
    return library, constants


def initial_current_factor(settings):
    if 'initial_factor' in settings:
        return settings['initial_factor']
    start = settings.get('start_state', 'pull')
    return settings['pre_k'] if start == 'pre_pull' else settings['fast_k'] if start == 'fast_pull' else 1.0


class Controller:
    def __init__(self, library, settings, feedback=True, initial_speed_mps=7.0):
        self.lib = ctypes.CDLL(str(library))
        f, i = ctypes.c_float, ctypes.c_int
        self.lib.controller_reset.argtypes = [f, f, f, i, i, f, i, f, f]
        self.lib.controller_reset.restype = None
        self.lib.controller_tick.argtypes = [i, f]
        self.lib.controller_tick.restype = f
        for name in ['braking', 'paused', 'active']:
            getattr(self.lib, 'controller_'+name).argtypes = []
            getattr(self.lib, 'controller_'+name).restype = i
        braking_target = settings.get('target_state', 'manual_braking') == 'manual_braking'
        target_kg = settings.get('target_kg', settings['brake_kg'] if braking_target else settings['unwind_kg'])
        self.lib.controller_reset(settings['pull_kg'], settings['amps_per_kg'], settings['pre_k'],
            settings['pull_ms'], settings['brake_ms'], target_kg, braking_target,
            initial_current_factor(settings), settings['acceleration_off_mss'])
        if hasattr(self.lib, 'controller_set_unwinding'):
            self.lib.controller_set_unwinding.argtypes = [f]
            self.lib.controller_set_unwinding.restype = None
            self.lib.controller_set_unwinding(settings['unwind_kg'])
        self.feedback = feedback
        self.measurement_period_ms = settings['measurement_period_ms']
        self.measurement_period_s = self.measurement_period_ms / 1000
        self.filter_gain = self.measurement_period_s / settings['measurement_speed_filter_period_secs']
        self.filtered_v = self.previous_v = self.previous_raw = initial_speed_mps
        self.filtered_a = self.previous_derivative = self.distance = 0.0
        self.encoder_step = settings.get('tachometer_step_m', 0.0)
        self.encoder_phase = settings.get('tachometer_phase', 0.0)
        self.encoder_position = 0.0
        self.encoder_ticks = math.floor(self.encoder_phase)
        if self.encoder_step:
            # Establish the filtered signal during one second of steady motion
            # before the command, retaining the encoder phase at time zero.
            warm_samples = round(1 / self.measurement_period_s)
            previous_ticks = math.floor(-warm_samples*self.measurement_period_s*initial_speed_mps/self.encoder_step+self.encoder_phase)
            for sample in range(-warm_samples+1, 1):
                ticks = math.floor(sample*self.measurement_period_s*initial_speed_mps/self.encoder_step+self.encoder_phase)
                self.filter_speed((ticks-previous_ticks)*self.encoder_step/self.measurement_period_s)
                previous_ticks = ticks
        self.previous_ms = 0
        self.events = []
        self.was_paused = False
        self.brake_switch_s = None
        self.transition_complete_s = None

    def filter_speed(self, raw):
        previous_filtered = self.filtered_v
        self.filtered_v += self.filter_gain*((raw+self.previous_raw)/2-self.filtered_v)
        derivative = (self.filtered_v-previous_filtered)/self.measurement_period_s
        self.filtered_a += self.filter_gain*((derivative+self.previous_derivative)/2-self.filtered_a)
        self.previous_raw, self.previous_derivative = raw, derivative

    def __call__(self, time_s, model, y):
        ms = round(time_s*1000)
        v = float(y[-1])
        dt = (ms-self.previous_ms)/1000
        self.distance += (v+self.previous_v)/2*dt
        if ms and ms % self.measurement_period_ms == 0:
            raw = self.distance/self.measurement_period_s
            if self.encoder_step:
                self.encoder_position += self.distance
                ticks = math.floor(self.encoder_position/self.encoder_step+self.encoder_phase)
                raw = (ticks-self.encoder_ticks)*self.encoder_step/self.measurement_period_s
                self.encoder_ticks = ticks
            self.filter_speed(raw)
            self.distance = 0.0
        self.previous_v, self.previous_ms = v, ms
        kgf = self.lib.controller_tick(ms, self.filtered_a if self.feedback else 0.0)
        braking = bool(self.lib.controller_braking())
        paused = bool(self.lib.controller_paused())
        if paused != self.was_paused:
            self.events.append(dict(time_s=time_s, paused=paused, kgf=kgf,
                                    acceleration=self.filtered_a))
            self.was_paused = paused
        if braking and self.brake_switch_s is None:
            self.brake_switch_s = time_s
        if not self.lib.controller_active() and self.transition_complete_s is None:
            self.transition_complete_s = time_s
        return kgf, braking

    def observe(self):
        return dict(filtered_speed_mps=self.filtered_v, filtered_acceleration_mps2=self.filtered_a,
            paused=self.lib.controller_paused(), stage_active=self.lib.controller_active())


def simulate(p, settings, library, output, name, feedback, dt=.00025, end=30.0,
             stop_at_ground=True, stop_at_strain_limit=False):
    controller = Controller(library, settings, feedback, initial_speed_mps=p.initial_drum_speed_mps)
    model = CableDrum(p)
    # Shorter cable segments need a smaller explicit step, especially for damping.
    # Halving preserves alignment with the 1 ms controller and 5 ms measurements.
    segment_length = model.length_initial / p.segments
    wave_limit = .25 * segment_length / math.sqrt(max(p.low_ea, p.high_ea) / p.mass_per_m)
    damping_limit = (.5 * p.mass_per_m * segment_length**2 / p.axial_damping_ns
                     if p.axial_damping_ns > 0 else math.inf)
    while dt > min(wave_limit, damping_limit):
        dt /= 2
    started = time.perf_counter()
    rows, shapes = integrate(model, controller, dt=dt, end_time=end, stop_at_ground=stop_at_ground,
                             stop_at_strain_limit=stop_at_strain_limit)
    save_rows(output/(name+'.csv'), rows)
    result = metrics(rows)
    if not stop_at_ground:
        result['stop_reason'] = 'time_limit'
    if stop_at_strain_limit and rows[-1]['max_strain'] >= p.strain_at_break:
        result['stop_reason'] = 'catalogue_strain_limit'
    result['integration_step_s'] = dt
    result['brake_switch_s'] = controller.brake_switch_s
    result['transition_complete_s'] = controller.transition_complete_s
    result['at_brake_switch'] = next((r for r in rows if r['braking']), None)
    result['pause_events'] = controller.events
    result['pause_count'] = sum(e['paused'] for e in controller.events)
    print(f'{name}: end {rows[-1]["time_s"]:.3f}s, complete {controller.transition_complete_s}s, '
          f'{result["pause_count"]} pauses; dt {dt:g}s; calculation {time.perf_counter()-started:.1f}s', flush=True)
    return rows, result, shapes


def check_feedback(rows, events, threshold):
    # Check the external interface, independently of the C stage internals.
    for before, after in zip(rows, rows[1:]):
        if after['paused']:
            assert after['command_kgf'] == before['command_kgf'], 'Current changed during a hold'
            assert after['stage_active'] and not after['braking']
    for e in events:
        if e['paused']:
            assert e['acceleration'] < threshold
        else:
            assert e['acceleration'] > threshold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    parser.add_argument('--control-dir', type=Path, default=ROOT/'build/skypuff_smooth_direct_140kg')
    parser.add_argument('--output', type=Path, default=ROOT/'build/skypuff_dynamics_configured')
    parser.add_argument('--end', type=float, default=30.0)
    parser.add_argument('--acceleration-off-mss', type=float,
                        help='Override antisex off / smooth pause threshold in host config (m/s²)')
    parser.add_argument('--start-state', choices=['pull', 'pre_pull', 'fast_pull'], default='pull',
                        help='Initial current before the transition')
    parser.add_argument('--target-state', choices=['manual_braking', 'unwinding'], default='manual_braking',
                        help='Fixed motor target; unwinding does not request the strong-current stage')
    parser.add_argument('--rope-length-m', type=float, default=1500.0,
                        help='Initial stretched length of the airborne rope, metres')
    parser.add_argument('--skip-convergence', action='store_true')
    args = parser.parse_args()
    if args.acceleration_off_mss is not None and (not math.isfinite(args.acceleration_off_mss) or args.acceleration_off_mss >= 0):
        parser.error('--acceleration-off-mss must be a finite negative acceleration')
    if not math.isfinite(args.rope_length_m) or args.rope_length_m <= 0:
        parser.error('--rope-length-m must be a finite positive length')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    settings = json.loads((args.control_dir/'results.json').read_text())['settings']
    # Older reports predate the shared config field; use the example/INI value.
    settings.pop('smooth_motor_a_min', None)
    settings.setdefault('acceleration_off_mss', -3.0)
    if args.acceleration_off_mss is not None:
        settings['acceleration_off_mss'] = args.acceleration_off_mss
    library, constants = build_controller(output, args.cc)
    settings.update(constants)
    settings['start_state'] = args.start_state
    settings['target_state'] = args.target_state
    initial_kgf = settings['pull_kg'] * initial_current_factor(settings)
    p = Parameters(initial_pull_kgf=initial_kgf, airborne_length_m=args.rope_length_m)
    checks = algebra_checks(p)
    baseline, baseline_result, _ = simulate(p, settings, library, output, 'without_pause', False, end=args.end)
    rows, result, shapes = simulate(p, settings, library, output, 'with_pause', True, end=args.end)
    check_feedback(rows, result['pause_events'], settings['acceleration_off_mss'])
    checks['feedback_hold_and_resume'] = 'pass'
    if args.target_state == 'manual_braking' and args.start_state == 'pre_pull' and settings['brake_kg'] < initial_kgf:
        assert not rows[0]['braking'] and rows[0]['stage_active']
        assert abs(rows[0]['command_kgf'] - initial_kgf) < 1e-5
        assert baseline_result['brake_switch_s'] == settings['brake_ms'] / 1000
        assert result['brake_switch_s'] is None or result['brake_switch_s'] >= baseline_result['brake_switch_s']
        checks['smooth_pre_pull_braking'] = 'pass'
    if args.target_state == 'unwinding':
        assert baseline_result['transition_complete_s'] == settings['pull_ms']/1000
        for trace in (baseline, rows):
            assert not any(r['braking'] for r in trace)
            assert abs(trace[0]['command_kgf'] - initial_kgf) < 1e-4
            assert all(after['command_kgf'] <= before['command_kgf']+1e-5
                       for before, after in zip(trace, trace[1:]))
        for before, after in zip(rows, rows[1:]):
            if after['command_kgf'] < before['command_kgf']-1e-5:
                assert after['filtered_acceleration_mps2'] >= settings['acceleration_off_mss']-1e-6
        if result['transition_complete_s'] is not None:
            assert result['transition_complete_s'] >= settings['pull_ms']/1000
            assert abs(rows[-1]['command_kgf'] - settings['unwind_kg']) < 1e-5
        checks['single_current_transition'] = 'pass'
    if not args.skip_convergence:
        finer_dt = result['integration_step_s'] / 2
        fine_time, _, _ = simulate(p, settings, library, output, 'check_half_timestep', True,
                                   dt=finer_dt, end=args.end)
        fine_space, _, _ = simulate(replace(p, segments=240), settings, library, output,
                                    'check_240_segments', True, dt=finer_dt, end=args.end)
        checks['half_timestep_max_difference'] = compare(rows, fine_time)
        checks['double_segments_max_difference'] = compare(rows, fine_space)
        assert checks['half_timestep_max_difference']['speed_mps'] < .02
        assert checks['double_segments_max_difference']['speed_mps'] < .15
    source = ROOT/'applications/skypuff/app_skypuff.c'
    report = dict(parameters=asdict(p), settings=settings, sources=SOURCES,
        production_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        host_config_overrides={} if args.acceleration_off_mss is None else {'acceleration_off_mss': args.acceleration_off_mss},
        compiled_types_sha256=hashlib.sha256((output/'production_types.inc').read_bytes()).hexdigest(),
        checks=checks, with_pause=result, without_pause=baseline_result)
    (output/'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    np.savez(output/'rope_shapes.npz', **{f't_{k}':v for k,v in shapes.items()})
    from plot_feedback import plot_feedback
    plot_feedback(output, rows, baseline, report)
    print(json.dumps({k: result[k] for k in ['end_s', 'stop_reason', 'transition_complete_s', 'brake_switch_s', 'pause_count',
                     'speed_mps', 'acceleration_mps2', 'filtered_acceleration_mps2']}, indent=2))
    print(json.dumps(checks, indent=2))


if __name__ == '__main__':
    main()

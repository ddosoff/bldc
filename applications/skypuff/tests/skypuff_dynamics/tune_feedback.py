#!/usr/bin/env python3
"""Offline parameter trials; only generated host C includes are modified.

Compare physical acceleration over 10–90% of the current reduction, rather than
making a heavily filtered measurement look smoother. Production files are never
changed. Optional hysteresis is an experimental change to the generated C copy.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess

import numpy as np
from closed_loop import ROOT, prepare_production_code, simulate, check_feedback, initial_current_factor
from model import Parameters


def score(rows, initial, target, threshold):
    selected = [r for r in rows if .1 <= (initial-r['command_kgf'])/(initial-target) <= .9]
    assert selected
    a = np.array([r['acceleration_mps2'] for r in selected])
    filtered = np.array([r['filtered_acceleration_mps2'] for r in selected])
    return dict(window='10–90% of current reduction', samples=len(selected),
                start_s=selected[0]['time_s'], end_s=selected[-1]['time_s'],
                mean_mps2=float(a.mean()), std_mps2=float(a.std()),
                p05_mps2=float(np.quantile(a, .05)), p95_mps2=float(np.quantile(a, .95)),
                p90_span_mps2=float(np.quantile(a, .95)-np.quantile(a, .05)),
                rmse_to_threshold_mps2=float(np.sqrt(np.mean((a-threshold)**2))),
                filtered_std_mps2=float(filtered.std()))


def trial(output, control_dir, cc, step_ms, filter_ms, hysteresis, segments=120,
          dt=.00025, damping=200., stiffness=1., tachometer_step_mm=0., phase=0.,
          start_state='fast_pull', glider_speed_mps=7., acceleration_off_mss=-1.):
    output.mkdir(parents=True, exist_ok=True)
    settings = json.loads((control_dir/'results.json').read_text())['settings']
    settings.update(prepare_production_code(output))
    settings.update(smooth_max_step_delay=step_ms,
                    measurement_speed_filter_period_secs=filter_ms/1000,
                    acceleration_off_mss=acceleration_off_mss, start_state=start_state, target_state='unwinding',
                    experimental_resume_hysteresis_mps2=hysteresis,
                    tachometer_step_m=tachometer_step_mm/1000, tachometer_phase=phase)
    types = output/'production_types.inc'
    text, count = re.subn(r'const int smooth_max_step_delay = \d+;',
                         f'const int smooth_max_step_delay = {step_ms};', types.read_text())
    assert count == 1
    types.write_text(text)
    functions = output/'production_functions.inc'
    if hysteresis:
        original = '(was_paused && acceleration <= acceleration_off)'
        text = functions.read_text()
        assert text.count(original) == 1
        functions.write_text(text.replace(original,
            f'(was_paused && acceleration <= acceleration_off + {hysteresis:.6f}f)'))
    library = output/'tuning_controller.so'
    subprocess.run(shlex.split(cc)+['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-shared', '-fPIC', '-I', str(output),
        str(Path(__file__).with_name('controller_bridge.c')), '-lm', '-o', str(library)], check=True)
    p = Parameters(initial_pull_kgf=settings['pull_kg']*initial_current_factor(settings), segments=segments,
                   axial_damping_ns=damping, stiffness_scale=stiffness,
                   glider_speed_mps=glider_speed_mps, initial_drum_speed_mps=glider_speed_mps)
    rows, result, _ = simulate(p, settings, library, output, 'trial', True, dt=dt, end=16)
    check_feedback(rows, result['pause_events'], settings['acceleration_off_mss'])
    for event in result['pause_events']:
        if not event['paused']:
            assert event['acceleration'] > settings['acceleration_off_mss']+hysteresis-1e-6
    assert result['transition_complete_s'] is not None, 'Target not reached in this trial'
    assert abs(rows[0]['command_kgf']-p.initial_pull_kgf) < 1e-4, 'Incorrect starting setpoint'
    assert abs(rows[0]['speed_mps']-p.initial_drum_speed_mps) < 1e-9, 'Incorrect initial drum speed'
    assert not any(row['braking'] for row in rows)
    for before, after in zip(rows, rows[1:]):
        assert after['command_kgf'] <= before['command_kgf']+1e-5
        if after['command_kgf'] < before['command_kgf']-1e-5:
            assert after['filtered_acceleration_mps2'] >= settings['acceleration_off_mss']-1e-6
    report = dict(settings=settings, parameters=asdict(p), result=result,
                  score=score(rows, p.initial_pull_kgf, settings['unwind_kg'], settings['acceleration_off_mss']),
                  checks=dict(hold_resume='pass', monotonically_decreasing='pass'),
                  production_source_sha256=hashlib.sha256((ROOT/'applications/skypuff/app_skypuff.c').read_bytes()).hexdigest(),
                  generated_functions_sha256=hashlib.sha256(functions.read_bytes()).hexdigest())
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(step_ms=step_ms, filter_ms=filter_ms, hysteresis=hysteresis,
                         acceleration_off_mss=acceleration_off_mss,
                         complete_s=result['transition_complete_s'], **report['score'])), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--control-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step-ms', type=int, required=True)
    parser.add_argument('--filter-ms', type=float, required=True)
    parser.add_argument('--hysteresis', type=float, default=0.)
    parser.add_argument('--segments', type=int, default=120)
    parser.add_argument('--dt', type=float, default=.00025)
    parser.add_argument('--damping', type=float, default=200.)
    parser.add_argument('--stiffness', type=float, default=1.)
    parser.add_argument('--tachometer-step-mm', type=float, default=0.)
    parser.add_argument('--phase', type=float, default=0.)
    parser.add_argument('--start-state', choices=('pull', 'pre_pull', 'fast_pull'), default='fast_pull')
    parser.add_argument('--glider-speed-mps', type=float, default=7.,
                        help='Signed speed along the rope, positive towards winch; initializes drum speed too')
    parser.add_argument('--acceleration-off-mss', type=float, default=-1.,
                        help='Antisex off / smooth pause threshold in host configuration (m/s²)')
    args = parser.parse_args()
    assert args.step_ms > 0 and args.filter_ms >= 5 and args.hysteresis >= 0
    assert np.isfinite(args.glider_speed_mps)
    if not np.isfinite(args.acceleration_off_mss) or args.acceleration_off_mss >= 0:
        parser.error('--acceleration-off-mss must be a finite negative acceleration')
    trial(args.output.resolve(), args.control_dir.resolve(), args.cc,
          args.step_ms, args.filter_ms, args.hysteresis, args.segments, args.dt,
          args.damping, args.stiffness, args.tachometer_step_mm, args.phase, args.start_state,
          args.glider_speed_mps, args.acceleration_off_mss)


if __name__ == '__main__':
    main()

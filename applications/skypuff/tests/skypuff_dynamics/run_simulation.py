#!/usr/bin/env python3
"""Simulate the actual C smooth_motor command applied to a drum and airborne rope."""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from model import G, Parameters, CableDrum, initial_equilibrium, integrate

ROOT = Path(__file__).resolve().parents[4]
SOURCES = {
    'rope_mass': 'https://store.armareropes.com/eng/dynext-pu-dyneema-sk75.html',
    'rope_stretch': 'https://store.armareropes.com/pub/media/mageworx/downloads/attachment/file/cataloghiArmare/2021/armare-diving-line-ita.pdf',
    'discretization_reference': 'https://moordyn.readthedocs.io/en/latest/structure.html',
}


def load_command(directory):
    report = json.loads((directory / 'results.json').read_text())
    settings = report['settings']
    selected = []
    path = directory / 'transitions.csv'
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if row['scenario'] == 'pull_manual_braking' and int(row['time_ms']) >= 0:
                assert int(row['time_ms']) == len(selected), 'Expected 1 ms C trace'
                selected.append((float(row['force_kg']), int(row['mode']) == 2))
    assert selected and len(selected) >= 6001
    assert report['tests']['pull_manual_braking']['completed_ms'] == settings['brake_ms']
    return selected, settings, hashlib.sha256(path.read_bytes()).hexdigest()


def add_antisex_filter(rows, filter_period_s=0.050):
    """Same two moving-pair means and first-order filters as measurement_tick.

    Warm-start at the steady 7 m/s. Tachometer quantization is omitted. Interval
    velocity uses the trapezoidal integral of the simulated surface velocity.
    This is an observation only: antisex does not alter the motor command here.
    """
    raw_previous = filtered_v = rows[0]['speed_mps']
    derivative_previous = filtered_a = 0.0
    rows[0]['filtered_acceleration_mps2'] = 0.0
    rows[0]['filtered_speed_mps'] = filtered_v
    for previous, row in zip(rows, rows[1:]):
        dt = row['time_s'] - previous['time_s']
        raw = (row['speed_mps'] + previous['speed_mps']) / 2
        k = dt / filter_period_s
        old_v = filtered_v
        filtered_v += k * ((raw + raw_previous) / 2 - filtered_v)
        derivative = (filtered_v - old_v) / dt
        filtered_a += k * ((derivative + derivative_previous) / 2 - filtered_a)
        row['filtered_acceleration_mps2'] = filtered_a
        row['filtered_speed_mps'] = filtered_v
        raw_previous, derivative_previous = raw, derivative


def metrics(rows):
    answer = {'end_s': rows[-1]['time_s'],
              'stop_reason': 'first_ground_contact' if rows[-1]['min_height_m'] <= 0 else 'time_limit'}
    for key in ['rope_force_kgf', 'speed_mps', 'acceleration_mps2', 'filtered_acceleration_mps2']:
        values = np.array([r[key] for r in rows])
        i, j = int(values.argmin()), int(values.argmax())
        answer[key] = {'min': values[i], 'min_at_s': rows[i]['time_s'],
                       'max': values[j], 'max_at_s': rows[j]['time_s']}
    answer['first_pay_out_s'] = next((r['time_s'] for r in rows if r['speed_mps'] < 0), None)
    answer['at_brake_switch'] = next((r for r in rows if r['braking']), None)
    return answer


def save_rows(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def simulate_case(p, command, output, name, dt=0.00025, end=6.0, filter_period_s=0.050):
    started = time.perf_counter()
    model = CableDrum(p)
    rows, shapes = integrate(model, command, end_time=end, dt=dt)
    add_antisex_filter(rows, filter_period_s)
    save_rows(output / f'{name}.csv', rows)
    print(f'{name}: {len(rows)} samples; end {rows[-1]["time_s"]:.3f} s; '
          f'calculation {time.perf_counter()-started:.1f} s', flush=True)
    return model, rows, shapes


def compare(first, second):
    # All cases are recorded on the same 5 ms time grid.
    n = min(len(first), len(second))
    return {k: float(max(abs(a[k]-b[k]) for a, b in zip(first[:n], second[:n])))
            for k in ['rope_force_kgf', 'speed_mps', 'acceleration_mps2']}


def algebra_checks(p):
    assert np.isclose(p.tension(p.strain_at_30_percent), 0.3*p.breaking_load_n)
    assert np.isclose(p.tension(p.strain_at_break), p.breaking_load_n)
    assert np.allclose(p.tension(p.strain(np.array([0, 200, 861, 1373, 2870]))), [0, 200, 861, 1373, 2870])
    assert p.tension(-0.01) == 0
    stationary = replace(p, glider_speed_mps=0, initial_drum_speed_mps=0)
    m = CableDrum(stationary)
    r, u, tension, tangent, stretch, ds, feed = m.fields(0, m.y0)
    force = np.diff(tension[:, None]*tangent, axis=0)
    force[:, 1] -= p.mass_per_m*ds*p.gravity
    residual = float(np.max(np.abs(force)))
    assert residual < 1e-6
    assert abs(m.rhs(0, m.y0, p.initial_pull_kgf, False)[-1]) < 1e-6
    for v in [-10, -.01, 0, .01, 10]:
        assert m.motor_force(1.8, True, v)*v <= 0, 'Brake must never add mechanical energy'
    assert np.isclose(p.inertia, 0.234375)
    return {'static_nodal_force_residual_n': residual, 'catalogue_points': 'pass', 'passive_brake': 'pass'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-dir', type=Path, default=ROOT/'build/skypuff_smooth_direct_140kg')
    parser.add_argument('--output', type=Path, default=ROOT/'build/skypuff_dynamics_direct_140kg')
    parser.add_argument('--skip-convergence', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    command, settings, trace_hash = load_command(args.control_dir.resolve())
    # Older trace reports used a 50 ms filter and did not record its period.
    filter_period_s = settings.setdefault('measurement_speed_filter_period_secs', 0.050)
    p = Parameters(initial_pull_kgf=settings['pull_kg'])
    checks = algebra_checks(p)
    model, rows, shapes = simulate_case(p, command, output, 'pull_manual_braking', filter_period_s=filter_period_s)
    variants = []
    for name, label, other in [
        ('low_damping', 'BA = 50 Н·с', replace(p, axial_damping_ns=50)),
        ('high_damping', 'BA = 800 Н·с', replace(p, axial_damping_ns=800)),
        ('solid_cylinder', 'Сплошной цилиндр: J = 0,117 кг·м²', replace(p, inertia_factor=0.5)),
    ]:
        _, data, _ = simulate_case(other, command, output, name, dt=0.000125 if name=='high_damping' else 0.00025,
                                   filter_period_s=filter_period_s)
        variants.append((name, label, data))
    if not args.skip_convergence:
        _, fine_time, _ = simulate_case(p, command, output, 'check_half_timestep', dt=0.000125,
                                        filter_period_s=filter_period_s)
        _, fine_space, _ = simulate_case(replace(p, segments=240), command, output, 'check_240_segments', dt=0.000125,
                                         filter_period_s=filter_period_s)
        _, baseline, _ = simulate_case(p, [(p.initial_pull_kgf, False)]*len(command), output, 'check_constant_pull', end=4,
                                       filter_period_s=filter_period_s)
        checks['half_timestep_max_difference'] = compare(rows, fine_time)
        checks['double_segments_max_difference'] = compare(rows, fine_space)
        checks['constant_pull_speed_drift_mps'] = max(abs(r['speed_mps']-7) for r in baseline)
        assert checks['half_timestep_max_difference']['speed_mps'] < .02
        assert checks['half_timestep_max_difference']['rope_force_kgf'] < .1
        assert checks['double_segments_max_difference']['speed_mps'] < .15
        assert checks['double_segments_max_difference']['rope_force_kgf'] < .5
        assert checks['constant_pull_speed_drift_mps'] < .01
    states = []
    for name, factor in [('PrePull', settings['pre_k']), ('TakeoffPull', settings['takeoff_k']),
                         ('Pull', 1.0), ('FastPull', settings['fast_k'])]:
        pp = replace(p, initial_pull_kgf=p.initial_pull_kgf*factor)
        r, length, force, tensions = initial_equilibrium(pp)
        sag = r[:, 0]*np.tan(np.deg2rad(p.angle_deg))-r[:, 1]
        states.append(dict(name=name, kgf=pp.initial_pull_kgf, rest_length_m=length,
                           material_extension_m=p.airborne_length_m-length,
                           sag_vertical_m=float(sag.max()), coordinates=r.tolist()))
    report = dict(parameters=asdict(p), source_urls=SOURCES, controller_settings=settings,
                  command_sha256=trace_hash, checks=checks, metrics=metrics(rows),
                  derived=dict(inertia_kgm2=p.inertia, equivalent_drum_mass_kg=p.drum_effective_mass,
                      initial_rpm=7/p.radius_m*60/(2*np.pi), low_ea_n=p.low_ea, high_ea_n=p.high_ea,
                      initial_rest_length_m=model.length_initial,
                      initial_airborne_mass_kg=p.mass_per_m*model.length_initial,
                      initial_endpoint_m=model.endpoint_initial.tolist()),
                  equilibria=states, alternatives={name:metrics(data) for name, _, data in variants})
    (output/'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    np.savez(output/'rope_shapes.npz', **{f't_{k}':v for k,v in shapes.items()})
    from plot_results import plot_results
    plot_results(output, rows, variants, shapes, report)
    print(json.dumps({'metrics':report['metrics'], 'checks':checks}, ensure_ascii=False, indent=2), flush=True)
    print(f'Output: {output}', flush=True)


if __name__ == '__main__':
    main()

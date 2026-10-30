#!/usr/bin/env python3
"""Simulate the current linear controller, including acceleration holds."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

import numpy as np
from closed_loop import ROOT, Parameters, simulate, check_feedback, algebra_checks
from plot_rate_transitions import prepare_production_code

CASES = {
    'unwinding_pull': ('Unwinding', 'Pull', 'unwinding', 'pull'),
    'pre_pull_manual_braking': ('PrePull', 'Manual braking', 'pre_pull', 'manual_braking'),
    'pull_manual_braking': ('Pull', 'Manual braking', 'pull', 'manual_braking'),
    'pull_unwinding': ('Pull', 'Unwinding', 'pull', 'unwinding'),
}


def plot(output, name, rows, report):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    settings, p, result = report['settings'], report['parameters'], report['result']
    complete = result['transition_complete_s']
    end = rows[-1]['time_s'] if complete is None else min(rows[-1]['time_s'], complete+.5)
    rows = [r for r in rows if r['time_s'] <= end+1e-9]
    t = np.array([r['time_s'] for r in rows])
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.subplots_adjust(left=.07, right=.98, bottom=.17, top=.77, hspace=.40, wspace=.23)
    force = axes[0, 0]
    force.step(t, [r['command_kgf'] for r in rows], where='post', color='#2463c5', lw=1.5,
               label='Уставка тока, эквивалент кгс')
    force.plot(t, [r['rope_force_kgf'] for r in rows], color='#21836a', lw=1.3,
               label='Натяжение у барабана')
    if settings['target_state'] == 'manual_braking':
        force.plot(t, [r['motor_force_kgf'] for r in rows], color='#b65c26', ls='--', lw=1,
                   label='Сила двигателя со знаком')
    force.fill_between(t, 0, 1, where=np.array([r['paused'] for r in rows], dtype=bool),
                       step='post', transform=force.get_xaxis_transform(), color='#c9b8a0', alpha=.3,
                       label='Пауза снижения')
    force.set(title='Уставка и натяжение', ylabel='кгс')
    axes[0, 1].plot(t, [r['speed_mps'] for r in rows], color='#2463c5', lw=1.5, label='Барабан')
    axes[0, 1].axhline(p['glider_speed_mps'], color='#66727f', ls=':',
                      label=f'Крыло {p["glider_speed_mps"]:g} м/с')
    axes[0, 1].set(title='Скорость поверхности барабана', ylabel='м/с; + намотка, − отдача')
    for ax, key, color, title in (
        (axes[1, 0], 'acceleration_mps2', '#c34b45', 'Физическое ускорение барабана'),
        (axes[1, 1], 'filtered_acceleration_mps2', '#8053aa', 'Ускорение после фильтра')):
        ax.plot(t, [r[key] for r in rows], color=color, lw=1.3)
        ax.axhline(settings['acceleration_off_mss'], color='#303a47', ls=':',
                   label=f'Порог {settings["acceleration_off_mss"]:g} м/с²')
        ax.set(title=title, ylabel='м/с²')
    for ax in axes.flat:
        ax.set_xlabel('Время после команды, с')
        ax.grid(alpha=.2)
        if complete is not None:
            ax.axvline(complete, color='#25825a', ls='--', lw=.9)
        ax.legend(fontsize=8.5, loc='best')
    start_title, end_title, _, _ = CASES[name]
    fig.suptitle(f'{start_title} → {end_title}: {p["initial_pull_kgf"]:g} → {settings["target_kg"]:g} кгс',
                 y=.97, fontsize=22, weight='bold')
    status = 'цель не достигнута до остановки модели' if complete is None else f'цель через {complete:.3f} с'
    fig.text(.5, .91, f'Номинал {settings["pull_kg"]:g} кгс · крыло {p["glider_speed_mps"]:g} м/с · '
             f'порог {settings["acceleration_off_mss"]:g} м/с² · {status}', ha='center', fontsize=12)
    fig.text(.5, .86, f'Трос {p["airborne_length_m"]:g} м, 15° · барабан 15 кг, Ø250 мм · '
             f'шаг {settings["smooth_max_step_delay"]} циклов · фильтр {settings["measurement_speed_filter_period_secs"]*1000:g} мс',
             ha='center', color='#546477')
    fig.text(.5, .81, 'Линейная скорость изменения: Pull 25 кгс/с; перед торможением 50 кгс/с. Паузы продлевают переход.',
             ha='center', fontsize=10.5, color='#546477')
    if result['stop_reason'] == 'catalogue_strain_limit':
        note = f'Начальный провис 58 м ниже барабана; земли нет. Расчёт до табличного растяжения 3,15%: {result["end_s"]:.3f} с.'
    elif not report['stop_at_ground']:
        note = 'Для Unwinding → Pull принято свободное пространство под тросом; начальный провис до 58 м ниже барабана.'
    elif result['stop_reason'] == 'first_ground_contact' and end >= result['end_s']-1e-9:
        note = f'Расчёт остановлен при первом касании тросом уровня земли: {result["end_s"]:.3f} с.'
    else:
        note = 'Показан переход и до 0,5 с после достижения цели. Зелёная вертикаль — завершение перехода.'
    fig.text(.07, .105, note, fontsize=10, color='#546477')
    fig.text(.07, .065, 'Модель: цикл 1 мс, тахометр ≈8 мм. Ограничения мощности и скорости двигателя не учтены; отдельная коррекция antisex отключена.',
             fontsize=10, color='#546477')
    fig.text(.07, .025, 'Unwinding задан фиксированно 2,2 кгс. Пауза удерживает уставку, но не ограничивает физическое ускорение жёстко.',
             fontsize=10, color='#546477')
    for ext in ('png', 'svg'):
        fig.savefig(output/f'{name}.{ext}', dpi=155, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dt', type=float, default=.00025)
    parser.add_argument('--end', type=float, default=12.)
    parser.add_argument('--segments', type=int,
                        help='Cable segments; default 240 for the low-tension start, otherwise 120')
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    constants = prepare_production_code(output)
    library = output/'rate_controller.so'
    subprocess.run(shlex.split(args.cc)+['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-shared', '-fPIC', '-I', str(output),
        str(Path(__file__).with_name('rate_controller_bridge.c')), '-lm', '-o', str(library)], check=True)
    _, _, start, target = CASES[args.case]
    forces = dict(unwinding=2.2, pre_pull=30., pull=100., manual_braking=1.8)
    settings = dict(pull_kg=100., amps_per_kg=2.45, pre_k=.3, fast_k=1.25, unwind_kg=2.2, brake_kg=1.8,
                    pull_ms=4000, brake_ms=2000, acceleration_off_mss=-3., start_state=start,
                    target_state=target, initial_factor=forces[start]/100, target_kg=forces[target],
                    tachometer_step_m=.007999425738307205, tachometer_phase=0., **constants)
    p = Parameters(initial_pull_kgf=forces[start], glider_speed_mps=6., initial_drum_speed_mps=6.,
                   segments=args.segments or (240 if start == 'unwinding' else 120))
    checks = algebra_checks(p)
    # User explicitly chose free space below the drum for the low-tension start.
    stop_at_ground = start != 'unwinding'
    rows, result, _ = simulate(p, settings, library, output, 'trace', True,
                              dt=args.dt, end=args.end, stop_at_ground=stop_at_ground, stop_at_strain_limit=True)
    check_feedback(rows, result['pause_events'], settings['acceleration_off_mss'])
    for before, after in zip(rows, rows[1:]):
        if not before['braking'] and not after['braking']:
            assert (after['command_kgf']-before['command_kgf'])*(forces[target]-forces[start]) >= -1e-4
    if result['transition_complete_s'] is not None:
        assert abs(rows[-1]['command_kgf']-forces[target]) < 1e-4
        assert bool(rows[-1]['braking']) == (target == 'manual_braking')
    if start == 'unwinding':
        assert not any(row['paused'] for row in rows)
    checks['controller_hold_resume_and_direction'] = 'pass'
    report = dict(settings=settings, parameters=asdict(p), result=result, checks=checks,
                  stop_at_ground=stop_at_ground,
                  production_source_sha256=hashlib.sha256((ROOT/'applications/skypuff/app_skypuff.c').read_bytes()).hexdigest())
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    if not args.no_plot:
        plot(output, args.case, rows, report)
    print(json.dumps(dict(case=args.case, complete_s=result['transition_complete_s'],
                         pauses=result['pause_count'], stop=result['stop_reason']), indent=2), flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Plot current-control retargeting with the production C controller."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
from run_tests import HERE, ROOT, ease, prepare_production_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    parser.add_argument('--scenario', choices=['pre_unwind_pull', 'fast_unwind', 'fast_unwind_direct'], default='pre_unwind_pull')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    direct = args.scenario == 'fast_unwind_direct'
    fast_unwind = args.scenario in ('fast_unwind', 'fast_unwind_direct')
    default_dir = ('skypuff_fast_unwinding_direct' if direct else
                   'skypuff_fast_unwinding' if fast_unwind else 'skypuff_pre_unwind_pull_retarget')
    output = (args.output or ROOT/'build'/default_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    settings = dict(amps_per_kg=2.45, pre_k=.3, unwind_kg=2.2, strong_kg=5.2,
                    pull_ms=4000, acceleration_off_mss=-3.0, retarget_ms=2000,
                    filtered_acceleration_mps2=0.0, filtered_speed_mps=-2.0,
                    strong_speed_threshold_mps=-1.0, fast_k=1.25, scenario=args.scenario)
    settings.update(prepare_production_code(output))
    if fast_unwind:
        settings['retarget_ms'] = 0 if direct else int(settings['strong_unwinding_period'])
    executable = output/'retarget_test'
    subprocess.run(shlex.split(args.cc) + ['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-Wno-old-style-declaration', '-fsanitize=undefined', '-fno-sanitize-recover=all',
        '-I', str(output), str(HERE/'retarget_harness.c'), '-lm', '-o', str(executable)], check=True)
    assert settings['strong_unwinding_period'] >= settings['retarget_ms']
    cases = []
    for nominal in (100, 60):
        csv_path = output/f'{nominal}kg.csv'
        values = [nominal, settings['amps_per_kg'], settings['pre_k'], settings['unwind_kg'],
                  settings['strong_kg'], settings['pull_ms'], settings['acceleration_off_mss']]
        if fast_unwind:
            values += [args.scenario, settings['fast_k']]
        with csv_path.open('w') as stream:
            subprocess.run([str(executable), *map(str, values)], stdout=stream, check=True)
        with csv_path.open() as stream:
            rows = [{k: float(v) for k, v in r.items() if k != 'scenario'} for r in csv.DictReader(stream)]
        switch = settings['retarget_ms']
        step = int(settings['smooth_max_step_delay'])
        duration = settings['pull_ms']
        p = max(1.0, settings['smooth_motor_p'])
        last_update = (switch-1)//step*step
        initial = nominal*(settings['fast_k'] if fast_unwind else settings['pre_k'])
        final = settings['unwind_kg'] if fast_unwind else nominal
        turning = ease(initial, settings['strong_kg'], last_update, duration, p)
        for row in rows:
            time = int(row['time_ms'])
            if time < 0:
                expected = initial
            elif time < switch:
                expected = ease(initial, settings['strong_kg'], time//step*step, duration, p)
            else:
                expected = ease(turning, final, (time-switch)//step*step, duration, p)
            assert row['mode'] == 1
            assert math.isclose(row['force_kg'], expected, abs_tol=2e-4)
            assert math.isclose(row['current_a']/settings['amps_per_kg'], row['force_kg'], abs_tol=1e-4)
        cases.append(dict(nominal_kgf=nominal, initial_kgf=initial, final_kgf=final,
                          at_retarget_kgf=turning, completed_ms=switch+duration, rows=rows))
        print(f'{nominal} kgf nominal: {initial:g} -> {turning:g} at {switch/1000:g} s -> {final:g} at {(switch+duration)/1000:g} s; all {len(rows)} samples passed')

    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 12,
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.edgecolor': '#b7c3cf', 'grid.color': '#e0e6ed',
        'xtick.color': '#4d5b6c', 'ytick.color': '#4d5b6c', 'svg.fonttype': 'none'})
    fig, ax = plt.subplots(figsize=(13, 7.8))
    fig.subplots_adjust(left=.085, right=.97, bottom=.24, top=.78)
    title = 'FastPull → Unwinding' if fast_unwind else 'PrePull → Unwinding → Pull'
    subtitle = ('Один переход с фиксированной целью 2,2 кгс' if direct else
                'Цель: усиленный Unwinding 5,2 кгс, через 2 с — обычный 2,2 кгс' if fast_unwind
                else 'Команда Pull через 2 с после начала снижения')
    fig.suptitle(title, fontsize=24, fontweight='bold', y=.96, color='#162a40')
    fig.text(.5, .90, subtitle, ha='center', fontsize=15, color='#3e5269')
    fig.text(.5, .85, f"p = {settings['smooth_motor_p']:g}  ·  период {duration/1000:g} с  ·  шаг {step:g} мс",
             ha='center', fontsize=12, color='#536376')
    switch_s, finish_s = switch/1000, (switch+duration)/1000
    if not direct:
        ax.axvspan(0, switch_s, color='#c9b8a0', alpha=.14)
        ax.axvline(switch_s, color='#526175', ls=':', lw=1.4)
    ax.axvspan(switch_s, finish_s, color='#bad4ed', alpha=.14)
    ax.axvline(finish_s, color='#25825a', ls=':', lw=1.4)
    peak = max(max(case['initial_kgf'], case['final_kgf']) for case in cases)
    if not direct:
        ax.text(switch_s+.07, peak*1.08, 'Новая цель: 2,2 кгс' if fast_unwind else 'Новая цель: Pull', color='#526175', fontsize=11)
    ax.text(finish_s-.08, peak*1.08, f'Цель достигнута: {finish_s:g} с', color='#25825a', fontsize=11, ha='right')
    for case, color in zip(cases, ['#2463c5', '#d97720']):
        nominal, rows = case['nominal_kgf'], case['rows']
        ax.step([r['time_ms']/1000 for r in rows], [r['force_kg'] for r in rows],
                where='post', color=color, lw=2.2, label=f'Номинал {nominal} кгс')
        ax.scatter([switch_s, finish_s], [case['at_retarget_kgf'], case['final_kgf']], color=color, s=32, zorder=4)
        if fast_unwind:
            xytext = (2.65, 80) if nominal == 100 else (.75, 27)
        else:
            xytext = (2.55, 37) if nominal == 100 else (1.1, 4)
        if not direct:
            ax.annotate(f"{case['at_retarget_kgf']:.2f} кгс", xy=(switch_s, case['at_retarget_kgf']),
                        xytext=xytext, color=color, fontsize=12,
                        arrowprops=dict(arrowstyle='->', color=color, lw=1))
        ax.text(-.12, case['initial_kgf']+2, f"{case['initial_kgf']:g}", color=color, fontsize=11)
    if fast_unwind:
        ax.axhline(settings['unwind_kg'], color='#7d8998', ls='--', lw=1)
        ax.text(finish_s+.15, settings['unwind_kg']+3, '2,2 кгс', color='#536376', fontsize=10)
    ax.set(xlim=(-.2, finish_s+.5), ylim=(0, peak*1.15), xlabel='Время после команды Unwinding, с', ylabel='Уставка, кгс')
    ax.grid(True)
    fig.legend(*ax.get_legend_handles_labels(), loc='upper center', bbox_to_anchor=(.5, .82), ncol=2, frameon=False)
    note1 = ('Цель smooth_motor сразу задана равной 2,2 кгс и не меняется. Усиленный Unwinding в этом тесте не запрашивается.' if direct else
             'Скорость принята −2 м/с: после первых 2 с условие усиленного Unwinding по скорости (> −1 м/с) не действует.' if fast_unwind
             else 'Первые 2 с учтён усиленный Unwinding: цель 5,2 кгс. В t = 2 с снижение прерывается командой Pull.')
    note2 = ('Порог acceleration_off_mss = −3 м/с² проверяется и при этом снижении. При ускорении 0 м/с² пауз нет.' if direct else
             'В t = 2 с цель меняется на 2,2 кгс: новая кривая начинается от текущей уставки и длится ещё полные 4 с.' if fast_unwind
             else 'Рост начинается от последней уставки (шаг в t = 1,95 с), без скачка к PrePull. Новая кривая длится полные 4 с.')
    fig.text(.085, .15, note1,
             fontsize=11, color='#536376')
    fig.text(.085, .10, note2,
             fontsize=10.5, color='#536376')
    fig.text(.085, .05, 'Данные из C-кода smooth_motor. Ускорение принято равным нулю: пауз нет. Показана уставка, без динамики троса.',
             fontsize=10.5, color='#536376')
    filename = ('fast_pull_unwinding_direct' if direct else
                'fast_pull_unwinding' if fast_unwind else 'pre_pull_unwinding_pull')
    for ext in ('png', 'svg'):
        fig.savefig(output/f'{filename}.{ext}', dpi=170, facecolor='white')
    plt.close(fig)
    report = dict(settings=settings,
        production_source_sha256=hashlib.sha256((ROOT/'applications/skypuff/app_skypuff.c').read_bytes()).hexdigest(),
        cases=[{k: v for k, v in case.items() if k != 'rows'} for case in cases],
        checks='all C assertions and independent per-sample curve checks passed')
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f"Plot: {output/(filename+'.png')}")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compare physical and measured acceleration for the offline tuning trials."""
import argparse
import csv
import json
import os
from pathlib import Path

import numpy as np


def read_rows(path):
    with path.open() as stream:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def plot_single_transition(output, kg, plt, cases=None):
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.subplots_adjust(left=.07, right=.98, bottom=.15, top=.79, hspace=.4, wspace=.22)
    panels = [('command_kgf', 'Уставка тяги', 'кгс'),
              ('speed_mps', 'Скорость поверхности барабана', 'м/с; «+» — намотка, «−» — отдача'),
              ('acceleration_mps2', 'Физическое ускорение', 'м/с²'),
              ('filtered_acceleration_mps2', 'После фильтра — сигнал для порога', 'м/с²')]
    table = []
    cases = cases or [(100, 25), (20, 50), (10, 50)]
    single = len(cases) == 1
    colors = {(100, 25): '#b86150', (20, 50): '#2563a6', (10, 50): '#21836a'}
    for step, filt in cases:
        color = colors.get((step, filt), '#2563a6')
        style = '--' if (step, filt) == (10, 50) and not single else '-'
        folder = output/f'{kg:g}kg_s{step}_f{filt}_h0_encoder'
        report = json.loads((folder/'results.json').read_text())
        settings, p = report['settings'], report['parameters']
        assert settings['pull_kg'] == kg and settings['target_state'] == 'unwinding'
        completed = report['result']['transition_complete_s']
        rows = [r for r in read_rows(folder/'trial.csv') if r['time_s'] <= completed]
        t = np.array([r['time_s'] for r in rows])
        label = f'Шаг {step} мс / фильтр {filt} мс · {completed:.2f} с'
        for ax, (key, title, unit) in zip(axes.flat, panels):
            values = [r[key] for r in rows]
            kwargs = dict(color=color, ls=style, lw=1.2 if step == 100 else 1.5,
                          alpha=.8 if step == 100 else 1, label=label)
            if single:
                kwargs['label'] = {'command_kgf': f'Уставка · цель в {completed:.2f} с',
                                   'speed_mps': 'Барабан',
                                   'acceleration_mps2': 'Физическое ускорение',
                                   'filtered_acceleration_mps2': 'Измеренное ускорение'}[key]
                kwargs['color'] = {'acceleration_mps2': '#c34b45',
                                   'filtered_acceleration_mps2': '#8053aa'}.get(key, color)
                if key == 'command_kgf':
                    title = 'Тяга и натяжение троса'
            if key == 'command_kgf':
                ax.step(t, values, where='post', **kwargs)
            else:
                ax.plot(t, values, **kwargs)
            ax.set_title(title, loc='left', fontsize=13, weight='bold')
            ax.set_ylabel(unit)
            ax.set_xlabel('Время после команды, с')
            ax.grid(alpha=.2)
        table.append(dict(nominal_kgf=kg, step_ms=step, filter_ms=filt,
                          complete_s=completed, **report['score']))
        if single:
            axes[0, 0].plot(t, [r['rope_force_kgf'] for r in rows], color='#21836a',
                            lw=1.3, label='Натяжение троса у барабана')
            axes[0, 0].fill_between(t, 0, 1, where=np.array([r['paused'] for r in rows], dtype=bool),
                                   step='post', transform=axes[0, 0].get_xaxis_transform(),
                                   color='#c9b8a0', alpha=.3, label='Пауза снижения тока')
    axes[0, 1].axhline(p['glider_speed_mps'], color='#66727f', ls=':', lw=1.2,
                      label=f'Параплан: {p["glider_speed_mps"]:g} м/с')
    for ax in axes[1]:
        ax.axhline(settings['acceleration_off_mss'], color='#303a47', ls=':', lw=1.2,
                   label=f'Порог {settings["acceleration_off_mss"]:g} м/с²')
    for ax in axes.flat:
        ax.legend(fontsize=9, loc='best', framealpha=.9)
    start = {'pull': 'Pull', 'fast_pull': 'FastPull', 'pre_pull': 'PrePull'}[settings['start_state']]
    fig.suptitle(f'{start} → Unwinding: {p["initial_pull_kgf"]:g} → {settings["unwind_kg"]:g} кгс',
                 y=.97, fontsize=22, weight='bold')
    fig.text(.5, .925, f'Номинал {kg:g} кгс  ·  порог {settings["acceleration_off_mss"]:g} м/с²  ·  '
             f'p = {settings["smooth_motor_p"]:g}  ·  {settings["pull_ms"]/1000:g} с активного снижения + паузы',
             ha='center', fontsize=12)
    fig.text(.5, .88, f'Трос {p["airborne_length_m"]:g} м, {p["angle_deg"]:g}°  ·  '
             f'барабан {p["drum_mass_kg"]:g} кг, Ø{p["radius_m"]*2000:g} мм  ·  параплан {p["glider_speed_mps"]:g} м/с',
             ha='center', fontsize=11, color='#546477')
    case_caption = f'Шаг {step} мс  ·  фильтр {filt} мс  ·  ' if single else ''
    fig.text(.5, .84, case_caption + f'Измерения каждые {settings["measurement_period_ms"]:g} мс  ·  '
             f'дискретность тахометра {settings["tachometer_step_m"]*1000:.2f} мм по skypuff.ini',
             ha='center', fontsize=11, color='#546477')
    fig.text(.07, .08, 'Каждая кривая показана до достижения цели; время включает паузы. Отдельная коррекция тока antisex отключена.',
             fontsize=10, color='#546477')
    fig.text(.07, .04, 'Это расчёт модели: упругость и затухание троса заданы; шум электроники и ошибка калибровки тахометра не учитываются.',
             fontsize=10, color='#546477')
    for ext in ('png', 'svg'):
        fig.savefig(output/f'acceleration_tuning.{ext}', dpi=160, facecolor='white')
    plt.close(fig)
    (output/'comparison.json').write_text(json.dumps(table, indent=2)+'\n')
    print(json.dumps(table, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--nominal-kg', type=float, help='Plot one nominal pull, with force and speed panels')
    parser.add_argument('--case', type=int, nargs=2, action='append', metavar=('STEP_MS', 'FILTER_MS'),
                        help='Choose one or more step/filter pairs for --nominal-kg')
    args = parser.parse_args()
    output = args.directory.resolve()
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    if args.nominal_kg is not None:
        plot_single_transition(output, args.nominal_kg, plt, args.case)
        return
    fig, axes = plt.subplots(2, 2, figsize=(15, 9.5))
    fig.subplots_adjust(left=.07, right=.98, bottom=.16, top=.79, hspace=.42, wspace=.22)
    fig.suptitle('Как шаг уставки и фильтр влияют на ускорение', y=.97, fontsize=21, weight='bold')
    fig.text(.5, .925, 'FastPull → Unwinding  ·  порог −1 м/с²  ·  p = 1  ·  4 с активного снижения + паузы',
             ha='center', fontsize=12)
    fig.text(.5, .885, 'Трос 1500 м, 15°  ·  барабан 15 кг, Ø250 мм  ·  параплан 7 м/с  ·  измерения каждые 5 мс',
             ha='center', fontsize=11, color='#546477')
    fig.text(.5, .845, 'Добавлено квантование тахометра ≈8 мм по skypuff.ini; отдельная коррекция тока antisex отключена.',
             ha='center', fontsize=11, color='#546477')
    table = []
    for row_i, kg in enumerate((100, 60)):
        for step, filt, color, label, style in [
            (100, 25, '#b86150', 'Шаг 100 мс, фильтр 25 мс', '-'),
            (20, 50, '#2563a6', 'Шаг 20 мс, фильтр 50 мс', '-'),
            (10, 50, '#21836a', 'Шаг 10 мс, фильтр 50 мс', '--'),
        ]:
            folder = output/f'{kg}kg_s{step}_f{filt}_h0_encoder'
            report = json.loads((folder/'results.json').read_text())
            rows = read_rows(folder/'trial.csv')
            completed = report['result']['transition_complete_s']
            rows = [r for r in rows if r['time_s'] <= completed]
            t = np.array([r['time_s'] for r in rows])
            for col, key in enumerate(('acceleration_mps2', 'filtered_acceleration_mps2')):
                axes[row_i, col].plot(t, [r[key] for r in rows], color=color,
                    lw=1.15 if step == 100 else 1.45, ls=style, alpha=.8 if step == 100 else 1,
                    label=label)
            table.append(dict(nominal_kgf=kg, step_ms=step, filter_ms=filt,
                complete_s=completed, **report['score']))
        for col, title in enumerate(('Физическое ускорение', 'После фильтра — сигнал для порога')):
            ax = axes[row_i, col]
            ax.axhline(-1, color='#303a47', ls=':', lw=1.2)
            ax.set_title(f'Номинал {kg} кгс · {title}', loc='left', fontsize=12, weight='bold')
            ax.set_xlabel('Время после команды, с')
            ax.set_ylabel('м/с²')
            ax.grid(alpha=.2)
            ax.legend(fontsize=8, loc='best', framealpha=.9)
    fig.text(.07, .09, 'Сравниваются расчёты с одинаковым квантованием. Разброс в таблице — σ физического ускорения на участке 10–90% снижения тока.',
             fontsize=10, color='#546477')
    fig.text(.07, .05, 'Параметры оценочные: упругость и затухание троса заданы моделью; шум электроники и ошибка калибровки тахометра не моделируются.',
             fontsize=10, color='#546477')
    for ext in ('png', 'svg'):
        fig.savefig(output/f'acceleration_tuning.{ext}', dpi=160, facecolor='white')
    plt.close(fig)
    (output/'comparison.json').write_text(json.dumps(table, indent=2)+'\n')
    print(json.dumps(table, indent=2))


if __name__ == '__main__':
    main()

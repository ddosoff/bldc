"""Figures for acceleration-dependent pauses of the production controller."""
import os
import numpy as np


def plot_feedback(output, rows, baseline, report):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.titleweight': 'bold', 'axes.titlecolor': '#1d2d40',
        'axes.edgecolor': '#b7c3cf', 'grid.color': '#e0e6ed',
        'xtick.color': '#4d5b6c', 'ytick.color': '#4d5b6c', 'svg.fonttype': 'none'})
    blue, green, red, purple, gray = '#2563b8', '#208461', '#c34b45', '#8053aa', '#9aa2ac'
    def data(rr, key):
        return np.array([r[key] for r in rr])
    t, tb = data(rows, 'time_s'), data(baseline, 'time_s')
    threshold = report['settings']['acceleration_off_mss']
    p = report['parameters']
    settings = report['settings']
    start_state = settings.get('start_state', 'pull')
    start_name = {'pre_pull': 'PrePull', 'fast_pull': 'FastPull', 'pull': 'Pull'}[start_state]
    target_state = settings.get('target_state', 'manual_braking')
    braking_target = target_state == 'manual_braking'
    target_name = 'Manual_braking' if braking_target else 'Unwinding'
    target_kg = settings['brake_kg'] if braking_target else settings['unwind_kg']
    period_ms = settings['brake_ms'] if braking_target else settings['pull_ms']
    result = report['with_pause']
    completed = result.get('transition_complete_s', result['brake_switch_s'])
    if result['brake_switch_s'] == 0.0:
        plot_instant_brake(output, rows, report, plt)
        return
    for name, xmax, title in [
        (f'{start_state}_{target_state}_acceleration_pause', t[-1]+.1,
         f'{start_name} → {target_name}: один переход с паузами'),
        ('acceleration_pause_first_seconds', min(3, t[-1]), 'Начало перехода: удержание тока и возобновление снижения'),
    ]:
        fig, axes = plt.subplots(1, 3, figsize=(18.5, 7.2))
        fig.subplots_adjust(left=.055, right=.98, bottom=.25, top=.73, wspace=.26)
        fig.suptitle(title, fontsize=20, fontweight='bold', y=.96, color='#162a40')
        settings = report['settings']
        fig.text(.5, .89, f'acceleration_off_mss = {threshold:g} м/с²  ·  {p["initial_pull_kgf"]:g} → {target_kg:g} кгс напрямую  ·  '
                 f'p = {settings["smooth_motor_p"]:g}  ·  шаг {settings["smooth_max_step_delay"]:g} мс  ·  '
                 f'{period_ms/1000:g} с активного снижения + паузы',
                 ha='center', fontsize=13, color='#3e5269')
        fig.text(.5, .835, f'Номинал {settings["pull_kg"]:g} кгс  ·  {p["drum_mass_kg"]:g} кг на Ø{2*p["radius_m"]*1000:g} мм  ·  '
                 f'Dyneema Ø{p["diameter_m"]*1000:g} мм, {p["airborne_length_m"]:g} м под {p["angle_deg"]:g}°  ·  '
                 f'параплан: {p["glider_speed_mps"]:g} м/с',
                 ha='center', fontsize=11, color='#536376')
        fig.text(.5, .79, f'Фильтр скорости и ускорения: {settings.get("measurement_speed_filter_period_secs", .050)*1000:g} мс  ·  '
                 f'измерения каждые {settings.get("measurement_period_ms", 5):g} мс',
                 ha='center', fontsize=11, color='#536376')
        axes[0].step(tb, data(baseline, 'command_kgf'), where='post', color=gray, ls='--', lw=1.5, label='Уставка без пауз')
        axes[0].step(t, data(rows, 'command_kgf'), where='post', color=blue, lw=2, label='Уставка с паузами')
        axes[0].plot(t, data(rows, 'rope_force_kgf'), color=green, lw=1.7, label='Натяжение у барабана')
        axes[0].fill_between(t, 0, 1, where=data(rows, 'paused').astype(bool), step='post',
                            transform=axes[0].get_xaxis_transform(), color='#c9b8a0', alpha=.3, label='Пауза снижения')
        peak_force = max(p['initial_pull_kgf'], float(data(rows, 'rope_force_kgf').max()))
        axes[0].set_ylim(0, 1.1 * peak_force)
        axes[0].set_ylabel('кгс')
        axes[0].set_title('Сила', loc='left', pad=12)
        axes[1].plot(tb, data(baseline, 'speed_mps'), color=gray, ls='--', lw=1.5, label='Без пауз')
        axes[1].plot(t, data(rows, 'speed_mps'), color=blue, lw=2, label='С паузами')
        axes[1].axhline(7, color=green, ls=':', lw=1.5, label='Параплан: 7 м/с')
        axes[1].axhline(0, color='#8793a2', lw=.7)
        axes[1].set_ylabel('м/с; «+» — намотка, «−» — отдача')
        axes[1].set_title('Скорость барабана', loc='left', pad=12)
        axes[2].plot(tb, data(baseline, 'filtered_acceleration_mps2'), color=gray, ls='--', lw=1.4, label='Без пауз, фильтр')
        axes[2].plot(t, data(rows, 'acceleration_mps2'), color=red, lw=1.1, alpha=.75, label='Механическое ускорение')
        axes[2].plot(t, data(rows, 'filtered_acceleration_mps2'), color=purple, lw=1.7, label='Фильтр: сигнал для порога')
        axes[2].axhline(threshold, color='#242e39', ls=':', lw=1.5, label=f'acceleration_off_mss = {threshold:g}')
        axes[2].set_ylabel('м/с² на поверхности барабана')
        axes[2].set_title('Ускорение барабана', loc='left', pad=12)
        for ax in axes:
            ax.set_xlim(0, xmax)
            ax.set_xlabel('Время после команды, с')
            ax.grid(True)
            ax.legend(fontsize=9, framealpha=.9, edgecolor='none', loc='best')
            if completed is not None and completed <= xmax:
                ax.axvline(completed, color='#d57724', ls=':', lw=1)
        complete_label = 'Тормоз включён' if braking_target else 'Цель достигнута'
        switch = (f'{complete_label} в {completed:.2f} с.' if completed is not None
                  else 'До остановки расчёта целевая уставка не достигнута.')
        fig.text(.055, .15, f'Ниже порога удерживаются ток и время стадии. Выход из паузы — строго выше порога. {switch}',
                 fontsize=10, color='#536376')
        fig.text(.055, .10, f'Проверка порога при обновлении уставки (до {settings["smooth_max_step_delay"]:g} мс); '
                 'ускорение измеряется через фильтры antisex. Его отдельная коррекция тока отключена.',
                 fontsize=10, color='#536376')
        stop = (f'Модель с паузами: первое касание земли в {t[-1]:.2f} с.' if result['stop_reason']=='first_ground_contact'
                else f'Модель с паузами: показано {t[-1]:g} с.')
        baseline_stop = (f'Без пауз: касание земли в {tb[-1]:.2f} с.' if report['without_pause']['stop_reason']=='first_ground_contact'
                         else f'Без пауз: показано {tb[-1]:g} с.')
        fig.text(.055, .05, f'{stop} {baseline_stop} Упругая разгрузка и затухание троса — допущения.',
                 fontsize=10, color='#536376')
        for ext in ['png', 'svg']:
            fig.savefig(output/f'{name}.{ext}', dpi=170, facecolor='white')
        plt.close(fig)
        print(f'Plot: {output/(name+".png")}', flush=True)


def plot_instant_brake(output, rows, report, plt):
    """Show the pre-command setpoint and physical response to an instant brake."""
    p, settings, result = report['parameters'], report['settings'], report['with_pause']
    start_name = 'PrePull' if settings.get('start_state') == 'pre_pull' else 'Pull'
    initial, brake = p['initial_pull_kgf'], settings['brake_kg']
    t = np.array([r['time_s'] for r in rows])
    def data(key):
        return np.array([r[key] for r in rows])
    blue, orange, green, red, purple = '#2563b8', '#d57724', '#208461', '#c34b45', '#8053aa'
    fig, axes = plt.subplots(1, 3, figsize=(18.5, 7.2))
    fig.subplots_adjust(left=.055, right=.98, bottom=.25, top=.73, wspace=.26)
    fig.suptitle(f'{start_name} → Manual_braking: мгновенное переключение',
                 fontsize=20, fontweight='bold', y=.96, color='#162a40')
    fig.text(.5, .89, f'Номинал {settings["pull_kg"]:g} кгс  ·  {start_name} {initial:g} кгс → тормоз {brake:g} кгс  ·  команда в t = 0',
             ha='center', fontsize=13, color='#3e5269')
    fig.text(.5, .835, f'Барабан: {p["drum_mass_kg"]:g} кг на Ø{2*p["radius_m"]*1000:g} мм  ·  '
             f'Dyneema Ø{p["diameter_m"]*1000:g} мм: {p["airborne_length_m"]:g} м под {p["angle_deg"]:g}°  ·  '
             f'параплан: {p["glider_speed_mps"]:g} м/с', ha='center', fontsize=11, color='#536376')
    axes[0].plot([-.2, 0], [initial, initial], color=blue, lw=2, label='Уставка тяги до команды')
    axes[0].plot([0, 0], [initial, brake], color=orange, lw=2)
    axes[0].step(t, data('command_kgf'), where='post', color=orange, lw=2, label='Уставка тормоза, модуль')
    axes[0].plot(np.r_[-.2, t], np.r_[rows[0]['rope_force_kgf'], data('rope_force_kgf')],
                 color=green, lw=1.8, label='Натяжение у барабана')
    axes[0].set_ylim(0, 1.15 * initial)
    axes[0].set_title('Сила', loc='left', pad=12)
    axes[0].set_ylabel('кгс')
    axes[1].plot(np.r_[-.2, t], np.r_[p['initial_drum_speed_mps'], data('speed_mps')],
                 color=blue, lw=2, label='Поверхность барабана')
    axes[1].axhline(p['glider_speed_mps'], color=green, ls=':', lw=1.5, label='Скорость параплана')
    axes[1].axhline(0, color='#8793a2', lw=.7)
    axes[1].set_title('Скорость барабана', loc='left', pad=12)
    axes[1].set_ylabel('м/с; «+» — намотка, «−» — отдача')
    axes[2].plot(np.r_[-.2, 0, t], np.r_[0, 0, data('acceleration_mps2')],
                 color=red, lw=1.5, label='Механическое ускорение')
    axes[2].plot(np.r_[-.2, t], np.r_[0, data('filtered_acceleration_mps2')],
                 color=purple, lw=1.8, label='После фильтров antisex')
    axes[2].axhline(0, color='#8793a2', lw=.7)
    axes[2].set_title('Ускорение барабана', loc='left', pad=12)
    axes[2].set_ylabel('м/с² на поверхности барабана')
    for ax in axes:
        ax.set_xlim(-.2, t[-1]+.05)
        ax.set_xlabel('Время после команды, с')
        ax.grid(True)
        ax.axvline(0, color=orange, ls=':', lw=1)
        ax.legend(fontsize=9, framealpha=.9, edgecolor='none', loc='best')
    fig.text(.055, .15, 'C-код включает тормоз сразу. Уставка меняется скачком; натяжение и скорость следуют динамике троса и барабана.',
             fontsize=10, color='#536376')
    fig.text(.055, .10, f'Пауза при acceleration_off_mss = {settings["acceleration_off_mss"]:g} м/с² и шаг {settings["smooth_max_step_delay"]:g} мс к этой мгновенной команде не применяются. Коррекция тока antisex отключена.',
             fontsize=10, color='#536376')
    stop = (f'Расчёт до первого касания земли: {t[-1]:.2f} с.' if result['stop_reason']=='first_ground_contact'
            else f'Показано {t[-1]:g} с.')
    fig.text(.055, .05, f'{stop} Тормозной момент направлен против вращения. Упругая разгрузка и затухание троса — допущения модели.',
             fontsize=10, color='#536376')
    name = settings.get('start_state', 'pull') + '_manual_braking_dynamics'
    for ext in ['png', 'svg']:
        fig.savefig(output/f'{name}.{ext}', dpi=170, facecolor='white')
    plt.close(fig)
    print(f'Plot: {output/(name+".png")}', flush=True)

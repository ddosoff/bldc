"""Plot actual smooth_motor setpoints for the requested current/mode transitions."""
import os


def plot_growth_comparison(cases, output):
    """Compare increasing-current traces from the host C controller tests."""
    os.environ.setdefault("MPLCONFIGDIR", str(output / "matplotlib-cache"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    import numpy as np

    assert len(cases) == 2
    settings = cases[0][1]
    amps_per_kg = settings["amps_per_kg"]
    for _, other in cases:
        for key in ("amps_per_kg", "pull_ms", "smooth_motor_p", "smooth_max_step_delay", "unwind_kg", "strong_kg"):
            assert other[key] == settings[key], f"Different {key} in comparison"
    scenarios = [
        ("unwinding_pre_pull", "Unwinding → PrePull", True),
        ("strong_unwinding_pre_pull", "Strong_unwinding → PrePull", True),
        ("pre_pull_takeoff_pull", "PrePull → TakeoffPull", False),
        ("takeoff_pull_pull", "TakeoffPull → Pull", False),
        ("pull_fast_pull", "Pull → FastPull", False),
    ]
    colors = ["#2463c5", "#d97720"]
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 11,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.edgecolor": "#bdc5cf", "axes.labelcolor": "#3b4859",
        "xtick.color": "#546172", "ytick.color": "#546172",
        "axes.titlecolor": "#1c2a3b", "axes.titleweight": "bold",
        "grid.color": "#e5e9ee", "svg.fonttype": "none",
    })
    fig, axes = plt.subplots(3, 2, figsize=(16, 13))
    fig.subplots_adjust(left=.07, right=.93, bottom=.09, top=.83, hspace=.63, wspace=.33)
    fig.suptitle("SkyPUFF: рост тока", fontsize=25, fontweight="bold", color="#142338", y=.975)
    fig.text(.5, .935, f"p = {settings['smooth_motor_p']:g}  ·  период роста {settings['pull_ms']/1000:g} с"
             f"  ·  шаг {settings['smooth_max_step_delay']:g} мс  ·  {amps_per_kg:g} А/кгс",
             ha="center", fontsize=14, color="#43516a")
    fig.legend(handles=[Line2D([0], [0], color=color, lw=2.4,
               label=f"Номинал {s['pull_kg']:g} кгс") for (_, s), color in zip(cases, colors)],
               loc="upper center", bbox_to_anchor=(.5, .91), ncol=2, frameon=False, fontsize=13)
    for ax, (name, title, instant) in zip(axes.flat, scenarios):
        peak = 0
        for (traces, s), color in zip(cases, colors):
            rows = traces[name]
            t = np.array([r["time_ms"] for r in rows]) / 1000
            current = np.array([r["current_a"] for r in rows])
            assert all(r["mode"] == 1 for r in rows)
            ax.step(t, current, where="post", color=color, lw=2.1)
            peak = max(peak, float(current.max()))
            ax.scatter([0 if instant else s["pull_ms"]/1000], [current[-1]], color=color, s=28, zorder=4)
        duration = 0 if instant else settings["pull_ms"]/1000
        ax.axvline(duration, color="#8591a2", ls=":", lw=1)
        ax.set_title(title, loc="left", fontsize=15, pad=28)
        curve_name = "Линейный рост" if settings["smooth_motor_p"] == 1 else "Плавный рост"
        ax.text(0, 1.04, "Мгновенный рост до PrePull" if instant else f"{curve_name} за {duration:g} с",
                transform=ax.transAxes, fontsize=11, color="#536175")
        ax.set_xlim(-.2, settings["pull_ms"]/1000 + .4)
        ax.set_ylim(0, peak*1.14)
        ax.set_xlabel("Время после команды, с")
        ax.set_ylabel("Уставка тока, А")
        ax.grid(True)
        secondary = ax.secondary_yaxis("right", functions=(
            lambda current: current / amps_per_kg,
            lambda kgf: kgf * amps_per_kg))
        secondary.set_ylabel("Эквивалент уставки, кгс")
    notes = axes.flat[-1]
    notes.axis("off")
    notes.text(0, .98, "Уставки для двух номиналов", fontsize=16,
               fontweight="bold", color="#1c2a3b", va="top")
    cells = [[name] + [f"{s['pull_kg']*s[key]:g}" for _, s in cases]
             for name, key in [("PrePull", "pre_k"), ("TakeoffPull", "takeoff_k"), ("FastPull", "fast_k")]]
    cells.insert(2, ["Pull"] + [f"{s['pull_kg']:g}" for _, s in cases])
    table = notes.table(cellText=cells,
        colLabels=["Режим"] + [f"{s['pull_kg']:g} кгс" for _, s in cases],
        cellLoc="left", colLoc="left", bbox=[0, .35, 1, .53])
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    for (row, _), cell in table.get_celld().items():
        cell.set_edgecolor("#e0e6ed")
        cell.set_text_props(color="#334155")
        if row == 0:
            cell.set_facecolor("#edf2f7")
            cell.set_text_props(weight="bold")
    notes.text(0, .25, f"Значения в таблице — кгс.\nUnwinding: {settings['unwind_kg']:g} кгс; Strong_unwinding: {settings['strong_kg']:g} кгс.",
               fontsize=11, color="#536175", va="top", linespacing=1.6)
    fig.text(.07, .045, "Каждый график — отдельный переход из установившейся уставки. acceleration_off_mss при росте тока не вызывает пауз.",
             fontsize=11, color="#536175")
    fig.text(.07, .02, "Данные из C-кода smooth_motor. Показаны заданный ток и его пересчёт в кгс; динамика барабана и троса здесь не моделируется.",
             fontsize=10, color="#536175")
    for extension in ("png", "svg"):
        fig.savefig(output / f"growth_current_comparison.{extension}", dpi=170, facecolor="white")
    plt.close(fig)
    print(f"Plot: {output / 'growth_current_comparison.png'}")


def plot_results(traces, settings, summary, output):
    os.environ.setdefault("MPLCONFIGDIR", str(output / "matplotlib-cache"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    import numpy as np
    from run_tests import reference, SCENARIOS

    s = settings
    blue, orange, grey, green = "#2463c5", "#d97720", "#9aa5b4", "#25825a"
    colors = {1: blue, 2: orange}
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 11,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.edgecolor": "#bdc5cf", "axes.labelcolor": "#3b4859",
        "xtick.color": "#546172", "ytick.color": "#546172",
        "axes.titleweight": "bold", "axes.titlecolor": "#1c2a3b",
        "grid.color": "#e5e9ee", "grid.linewidth": 0.8,
        "savefig.facecolor": "white", "svg.fonttype": "none",
    })
    pre = s["pull_kg"] * s["pre_k"]
    takeoff = s["pull_kg"] * s["takeoff_k"]
    braking_scenarios = (
        "fast_pull_manual_braking", "pull_manual_braking", "takeoff_pull_manual_braking",
        "pre_pull_manual_braking", "strong_unwinding_manual_braking", "unwinding_manual_braking",
    )
    ramped_braking = braking_scenarios[:4]
    labels = {
        "unwinding_pre_pull": ("Unwinding → Pre_Pull", f"{s['unwind_kg']:g} → {pre:g} кгс мгновенно: рост до PrePull"),
        "strong_unwinding_pre_pull": ("Strong_unwinding → Pre_Pull", f"{s['strong_kg']:g} → {pre:g} кгс мгновенно: рост до PrePull"),
        "pull_unwinding": ("Pull → Unwinding", f"{s['pull_kg']:g} → {s['unwind_kg']:g} кгс напрямую за {s['pull_ms'] / 1000:g} с"),
        "pull_strong_unwinding": ("Pull → Strong_unwinding", f"{s['pull_kg']:g} → {s['strong_kg']:g} кгс напрямую за {s['pull_ms'] / 1000:g} с"),
        "pull_manual_braking": ("Pull → Manual_braking",
            f"{s['pull_kg']:g} → {s['brake_kg']:g} кгс напрямую за {s['brake_ms'] / 1000:g} с; затем тормоз"),
        "unwinding_manual_braking": ("Unwinding → Manual_braking",
            f"Тяга {s['unwind_kg']:g} → тормоз {s['brake_kg']:g} кгс мгновенно: ниже PrePull"),
        "manual_braking_unwinding": ("Manual_braking → Unwinding",
            f"Тормоз {s['brake_kg']:g} кгс → тяга {s['unwind_kg']:g} кгс мгновенно"),
        "pre_pull_takeoff_pull": ("Pre_Pull → Takeoff_Pull",
            f"{pre:g} → {takeoff:g} кгс плавно за {s['pull_ms'] / 1000:g} с"),
        "takeoff_pull_pull": ("Takeoff_Pull → Pull",
            f"{takeoff:g} → {s['pull_kg']:g} кгс плавно за {s['pull_ms'] / 1000:g} с"),
        "pre_pull_unwinding": ("Pre_Pull → Unwinding",
            f"{pre:g} → {s['unwind_kg']:g} кгс: одна плавная ступень за {s['pull_ms'] / 1000:g} с"),
        "takeoff_pull_unwinding": ("Takeoff_Pull → Unwinding",
            f"{takeoff:g} → {s['unwind_kg']:g} кгс напрямую за {s['pull_ms'] / 1000:g} с"),
        "pull_fast_pull": ("Pull → Fast_pull",
            f"{s['pull_kg']:g} → {s['pull_kg'] * s['fast_k']:g} кгс за {s['pull_ms'] / 1000:g} с; Fast_pull = {s['fast_k'] * 100:g}% номинала"),
        "fast_pull_manual_braking": ("Fast_pull → Manual_braking",
            f"{s['pull_kg'] * s['fast_k']:g} → {s['brake_kg']:g} кгс напрямую за {s['brake_ms'] / 1000:g} с; затем тормоз"),
        "takeoff_pull_manual_braking": ("Takeoff_pull → Manual_braking",
            f"{takeoff:g} → {s['brake_kg']:g} кгс напрямую за {s['brake_ms'] / 1000:g} с; затем тормоз"),
        "pre_pull_manual_braking": ("Pre_pull → Manual_braking",
            f"{pre:g} → {s['brake_kg']:g} кгс за {s['brake_ms'] / 1000:g} с; затем тормоз"),
        "strong_unwinding_manual_braking": ("Strong_unwinding → Manual_braking",
            f"Тяга {s['strong_kg']:g} → тормоз {s['brake_kg']:g} кгс мгновенно: ниже PrePull"),
    }
    data = {}
    for scenario, rows in traces.items():
        data[scenario] = (
            np.array([row["time_ms"] for row in rows]) / 1000,
            np.array([row["force_kg"] for row in rows]),
            np.array([row["mode"] for row in rows]),
            np.array([reference(scenario, row["time_ms"], s, sampled=False)[0] for row in rows]),
        )

    def draw_lines(ax, scenario, width=2.2):
        times, forces, modes, formula = data[scenario]
        ax.plot(times, formula, "--", color=grey, lw=1.4, zorder=2)
        for mode, color in colors.items():
            ax.step(times, np.where(modes == mode, forces, np.nan), where="post",
                    color=color, lw=width, zorder=3)
        # Show the instantaneous jump even though the two modes have different colors.
        for index in np.flatnonzero(modes[1:] != modes[:-1]) + 1:
            ax.plot([times[index], times[index]], [forces[index - 1], forces[index]],
                    color=colors[modes[index]], lw=width, zorder=3)

    def draw_panel(ax, scenario, end_seconds=None):
        times, forces, modes, formula = data[scenario]
        draw_lines(ax, scenario)
        title, caption = labels[scenario]
        ax.set_title(title, loc="left", fontsize=14, pad=31)
        ax.text(0, 1.04, caption, transform=ax.transAxes, fontsize=10, color="#536175")
        ax.set_ylabel("Уставка, кгс")
        ax.set_xlabel("Время, с")
        ax.tick_params(labelbottom=True)
        end = times[-1] if end_seconds is None else end_seconds
        ax.set_xlim(-0.2, end)
        ax.set_xticks(np.arange(0, end + 0.01, 1.0))
        peak = s["pull_kg"] if scenario in ("pre_pull_takeoff_pull", "takeoff_pull_pull") else max(forces)
        ax.set_ylim(0, peak * 1.15)
        ax.grid(True)
        if scenario in ("pull_unwinding", "pull_strong_unwinding", "takeoff_pull_unwinding") + ramped_braking:
            ax.axhline(pre, color="#7b698e", ls=":", lw=1.1, zorder=1)
            ax.text(0.98, pre + 1.0, f"PrePull = {pre:g} кгс", ha="right", fontsize=9,
                    color="#7b698e", transform=ax.get_yaxis_transform())
        deadline = summary[scenario]["completed_ms"] / 1000
        final_force = summary[scenario]["final_kg"]
        ax.axvline(deadline, color=green, ls=":", lw=1.2, zorder=1)
        ax.scatter([deadline], [final_force], color=green, s=35, zorder=4)
        ax.text(deadline + 0.08 if deadline == 0 else deadline - 0.08, 0.96,
                "сразу" if deadline == 0 else f"цель: {deadline:g} с",
                ha="left" if deadline == 0 else "right", transform=ax.get_xaxis_transform(),
                color=green, fontsize=10, bbox={"facecolor": "white", "edgecolor": "none", "pad": 2})

    def draw_rules(ax):
        ax.axis("off")
        ax.text(0.04, 0.97, "Правила на уровне smooth_motor", fontsize=15,
                fontweight="bold", color="#1c2a3b", transform=ax.transAxes)
        notes = [
            f"Порог PrePull: {pre:g} кгс ({pre * s['amps_per_kg']:g} А).",
            "Рост с целью ≤ PrePull — мгновенно.",
            "Снижение тока тяги — плавно, даже ниже PrePull.",
            "В тормоз: старт < PrePull, цель ≤ PrePull → сразу.",
            "Плавный переход — напрямую к цели.",
            f"Тяга: {s['pull_ms'] / 1000:g} с; перед тормозом: {s['brake_ms'] / 1000:g} с.",
            f"При a < {s['acceleration_off_mss']:g} м/с² снижение ждёт.",
            "Знак тяги до тормоза сохраняется.",
        ]
        for index, note in enumerate(notes):
            ax.text(0.04, 0.79 - index * 0.10, note, fontsize=11, color="#43516a", transform=ax.transAxes)

    def render_group(scenarios, filename, title, end_seconds=None):
        ncols = 1 if len(scenarios) == 1 else 2
        nrows = (len(scenarios) + ncols - 1) // ncols
        height = 6.0 + 4.4 * (nrows - 1)
        fig, axes = plt.subplots(nrows, ncols, figsize=(15, height), squeeze=False)
        fig.subplots_adjust(left=0.075, right=0.97, bottom=1.4 / height, top=1 - 2.2 / height,
                            hspace=0.54, wspace=0.23)
        fig.suptitle(title, fontsize=21, fontweight="bold", color="#142338", y=1 - 0.18 / height)
        fig.text(0.5, 1 - 0.72 / height,
                 f"Номинал {s['pull_kg']:g} кгс    ·    p = {s['smooth_motor_p']:g}"
                 f"    ·    pull = {s['pull_ms'] / 1000:g} с, braking = {s['brake_ms'] / 1000:g} с без пауз",
                 ha="center", fontsize=12, color="#43516a")
        fig.legend(handles=[
            Line2D([0], [0], color=blue, lw=2.3, label="MOTOR_CURRENT · ток тяги"),
            Line2D([0], [0], color=orange, lw=2.3, label="MOTOR_BRAKING · тормоз"),
            Line2D([0], [0], color=grey, lw=1.5, ls="--", label="Непрерывная расчётная кривая"),
        ], loc="upper center", bbox_to_anchor=(0.5, 1 - 1.05 / height), ncol=3, frameon=False, fontsize=10)
        for ax, scenario in zip(axes.flat, scenarios):
            draw_panel(ax, scenario, end_seconds)
        if len(scenarios) % ncols:
            draw_rules(axes.flat[-1])
        fig.text(0.075, 0.48 / height,
                 f"Уставки из C-кода, шаг до {s['smooth_max_step_delay']:g} мс; {s['amps_per_kg']:g} А/кгс. "
                 "Ускорение = 0: пауз нет. Вне зоны мгновенного торможения. Цель фиксирована.",
                 fontsize=10, color="#526177")
        fig.text(0.075, 0.19 / height,
                 "Графики показывают заданный ток, пересчитанный в кгс, а не измеренную силу.",
                 fontsize=10, color="#526177")
        for extension in ("png", "svg"):
            fig.savefig(output / f"{filename}.{extension}", dpi=180)
        plt.close(fig)
        print(f"Plot: {output / (filename + '.png')}")

    render_group(SCENARIOS[:4], "current_transitions", "SkyPUFF: переходы тока")
    render_group(SCENARIOS[4:7], "braking_transitions", "SkyPUFF: переходы к торможению и обратно")
    render_group(SCENARIOS[7:9], "takeoff_transitions", "SkyPUFF: PrePull → TakeoffPull → Pull")
    render_group(SCENARIOS[9:11], "reduction_transitions", "SkyPUFF: снижение от PrePull и TakeoffPull")
    render_group(["pull_fast_pull"], "fast_pull_transition", "SkyPUFF: Pull → Fast_pull")
    render_group(braking_scenarios, "manual_braking_transitions", "SkyPUFF: шесть переходов в Manual_braking",
                 end_seconds=s["brake_ms"] / 1000 + 0.5)
    render_group(SCENARIOS, "transitions", "SkyPUFF: переходы smooth_motor")

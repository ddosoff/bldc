#!/usr/bin/env python3
"""Compile the real SkyPUFF motor code on the host, test it, and optionally plot it."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENARIOS = (
    "unwinding_pre_pull", "strong_unwinding_pre_pull",
    "pull_unwinding", "pull_strong_unwinding", "pull_manual_braking",
    "unwinding_manual_braking", "manual_braking_unwinding",
    "pre_pull_takeoff_pull", "takeoff_pull_pull",
    "pre_pull_unwinding", "takeoff_pull_unwinding",
    "pull_fast_pull",
    "fast_pull_manual_braking", "takeoff_pull_manual_braking",
    "pre_pull_manual_braking", "strong_unwinding_manual_braking",
)


def prepare_production_code(output):
    source = (ROOT / "applications/skypuff/app_skypuff.c").read_text()
    header = (ROOT / "applications/skypuff/app_skypuff.h").read_text()
    clock_config = (ROOT / "chconf.h").read_text()
    clock_hz = int(re.search(r"#define CH_CFG_ST_FREQUENCY\s+(\d+)", clock_config)[1])
    assert clock_hz % 1000 == 0, "Harness expects whole clock ticks per millisecond"

    enums = []
    for first, name in [("UNINITIALIZED", "skypuff_state"),
                        ("MOTOR_RELEASED", "smooth_motor_mode")]:
        enums.append(re.search(r"typedef enum\s*\{\s*" + first +
                               r",.*?\} " + name + ";", header, re.DOTALL)[0])
    constants = []
    values = {}
    for name in ("smooth_max_step_delay", "smooth_motor_p", "strong_unwinding_period"):
        match = re.search(r"^const (?:int|float) " + name + r" = ([^;]+);", source, re.MULTILINE)
        constants.append(match[0])
        values[name] = float(match[1].rstrip("f"))
    values["clock_hz"] = clock_hz
    measurement_delay = re.search(r"^static systime_t measurement_delay = MS2ST\((\d+)\);", source, re.MULTILINE)
    filter_period = re.search(r"^static float measurement_speed_filter_period_secs = ([0-9.]+)f;", source, re.MULTILINE)
    values["measurement_period_ms"] = int(measurement_delay[1])
    values["measurement_speed_filter_period_secs"] = float(filter_period[1])
    measurement_struct = re.search(r"// Store tachometer based speed measurements here\s*(struct \{.*?^\} measurement;)",
                                   source, re.DOTALL | re.MULTILINE)[1]
    measurement_tick = re.search(r"static inline bool measurement_tick\(int cur_tac\) \{.*?^\}",
                                 source, re.DOTALL | re.MULTILINE)[0]
    utils_header = (ROOT / "util/utils_math.h").read_text()
    low_pass_macro = re.search(r"^#define UTILS_LP_FAST[^\n]+", utils_header, re.MULTILINE)[0]
    (output / "production_measurement.inc").write_text(
        f"#define CH_CFG_ST_FREQUENCY {clock_hz}U\n" + "\n".join(
            [low_pass_macro, measurement_delay[0], filter_period[0], measurement_struct, measurement_tick]) + "\n")
    declarations = source.split("// Smooth force apply and release\n", 1)[1].split("\n/*", 1)[0]
    (output / "production_types.inc").write_text(
        f"#define TEST_CLOCK_HZ {clock_hz}U\n" + "\n".join(enums + constants) + declarations)

    # These are production functions, not a second implementation of the ramp.
    functions = source.split("// Smooth motor functions\n", 1)[1].split("\nstatic void save_custom_msg", 1)[0]
    functions += source.split("// Process all posible motor mode transitions", 1)[1].split(
        "// Helper functions to check limits", 1)[0]
    functions += "inline static float unwinding_current(void) {" + source.split(
        "inline static float unwinding_current(void) {", 1)[1].split(
        "\ninline static void unwinding(", 1)[0]
    (output / "production_functions.inc").write_text(functions)
    return values


def ease(start, target, elapsed_ms, duration_ms, p):
    """Independent closed-form reference, evaluated in Python double precision."""
    u = max(0.0, min(1.0, elapsed_ms / duration_ms))
    return start + (target - start) * (1.0 - (1.0 - u) ** p)


def reference(scenario, time_ms, settings, sampled=True):
    s = settings
    pull, unwind, strong, brake = (s[k] for k in ("pull_kg", "unwind_kg", "strong_kg", "brake_kg"))
    pull_ms, brake_ms = s["pull_ms"], s["brake_ms"]
    pre = pull * s["pre_k"]
    takeoff = pull * s["takeoff_k"]
    step_ms = int(s["smooth_max_step_delay"])
    p = max(1.0, s["smooth_motor_p"])

    def curve(start, end, elapsed, duration):
        if elapsed >= duration:
            return end
        if sampled:
            elapsed = elapsed // step_ms * step_ms
        return ease(start, end, elapsed, duration, p)

    initial = {
        "unwinding_pre_pull": (unwind, 1),
        "strong_unwinding_pre_pull": (strong, 1),
        "pull_unwinding": (pull, 1),
        "pull_strong_unwinding": (pull, 1),
        "pull_manual_braking": (pull, 1),
        "unwinding_manual_braking": (unwind, 1),
        "manual_braking_unwinding": (brake, 2),
        "pre_pull_takeoff_pull": (pre, 1),
        "takeoff_pull_pull": (takeoff, 1),
        "pre_pull_unwinding": (pre, 1),
        "takeoff_pull_unwinding": (takeoff, 1),
        "pull_fast_pull": (pull, 1),
        "fast_pull_manual_braking": (pull * s["fast_k"], 1),
        "takeoff_pull_manual_braking": (takeoff, 1),
        "pre_pull_manual_braking": (pre, 1),
        "strong_unwinding_manual_braking": (strong, 1),
    }
    if time_ms < 0:
        return initial[scenario]
    if scenario in ("unwinding_pre_pull", "strong_unwinding_pre_pull"):
        return pre, 1
    if scenario in ("pull_unwinding", "pull_strong_unwinding", "takeoff_pull_unwinding"):
        target = unwind if scenario == "pull_unwinding" else strong
        if scenario == "takeoff_pull_unwinding":
            target = unwind
        start = initial[scenario][0]
        return curve(start, target, time_ms, pull_ms), 1
    if scenario in ("unwinding_manual_braking", "strong_unwinding_manual_braking"):
        return brake, 2
    if scenario == "manual_braking_unwinding":
        return unwind, 1
    if scenario == "pre_pull_takeoff_pull":
        return curve(pre, takeoff, time_ms, pull_ms), 1
    if scenario == "takeoff_pull_pull":
        return curve(takeoff, pull, time_ms, pull_ms), 1
    if scenario == "pre_pull_unwinding":
        return curve(pre, unwind, time_ms, pull_ms), 1
    if scenario == "pull_fast_pull":
        return curve(pull, pull * s["fast_k"], time_ms, pull_ms), 1
    if time_ms < brake_ms:
        return curve(initial[scenario][0], brake, time_ms, brake_ms), 1
    return brake, 2


def check_traces(traces, settings):
    s = settings
    deadlines = {
        "unwinding_pre_pull": 0,
        "strong_unwinding_pre_pull": 0,
        "pull_unwinding": s["pull_ms"],
        "pull_strong_unwinding": s["pull_ms"],
        "pull_manual_braking": s["brake_ms"],
        "unwinding_manual_braking": 0,
        "manual_braking_unwinding": 0,
        "pre_pull_takeoff_pull": s["pull_ms"],
        "takeoff_pull_pull": s["pull_ms"],
        "pre_pull_unwinding": s["pull_ms"],
        "takeoff_pull_unwinding": s["pull_ms"],
        "pull_fast_pull": s["pull_ms"],
        "fast_pull_manual_braking": s["brake_ms"],
        "takeoff_pull_manual_braking": s["brake_ms"],
        "pre_pull_manual_braking": s["brake_ms"],
        "strong_unwinding_manual_braking": 0,
    }
    summary = {}
    for scenario in SCENARIOS:
        rows = traces[scenario]
        by_time = {row["time_ms"]: row for row in rows}
        for row in rows:
            time_ms = row["time_ms"]
            expected, mode = reference(scenario, time_ms, s)
            assert math.isfinite(row["force_kg"])
            assert math.isclose(row["force_kg"], expected, abs_tol=2e-4), (
                f"{scenario} at {time_ms}ms: {row['force_kg']:.7f}kg, expected {expected:.7f}kg")
            assert row["mode"] == mode, f"{scenario} at {time_ms}ms: wrong motor mode"
            assert math.isclose(row["current_a"] / s["amps_per_kg"], row["force_kg"], abs_tol=1e-4)

        deadline = deadlines[scenario]
        if deadline > 0:
            assert by_time[deadline - 1]["stage_active"] == 1, f"{scenario}: completed early"
        assert by_time[deadline]["stage_active"] == 0, f"{scenario}: missed deadline"
        assert all(row["stage_active"] == 0 for row in rows if row["time_ms"] >= deadline)
        summary[scenario] = {"completed_ms": deadline,
                             "final_kg": by_time[deadline]["force_kg"],
                             "checked_samples": len(rows)}
        print(f"PASS {scenario}: {deadline / 1000:g}s, {len(rows)} checked samples")
    return summary


def read_traces(path):
    traces = {name: [] for name in SCENARIOS}
    integers = {"time_ms", "mode", "stage_active", "stage_start_ticks", "stage_duration_ms"}
    with path.open(newline="") as stream:
        for raw in csv.DictReader(stream):
            row = {key: int(value) if key in integers else float(value)
                   for key, value in raw.items() if key != "scenario"}
            traces[raw["scenario"]].append(row)
    return traces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    parser.add_argument("--output", type=Path, default=ROOT / "build/skypuff_smooth")
    parser.add_argument("--plot", action="store_true", help="Also save PNG and SVG plots (requires matplotlib)")
    parser.add_argument("--debug", action="store_true", help="Also compile DEBUG_SMOOTH_MOTOR code")
    parser.add_argument("--pull-kg", type=float, default=60.0)
    parser.add_argument("--pull-ms", type=int, default=4000)
    parser.add_argument("--brake-ms", type=int, default=2000)
    parser.add_argument("--amps-per-kg", type=float, default=3.0)
    parser.add_argument("--pre-k", type=float, default=0.3)
    parser.add_argument("--takeoff-k", type=float, default=0.65)
    parser.add_argument("--fast-k", type=float, default=1.25)
    parser.add_argument("--unwind-kg", type=float, default=0.4)
    parser.add_argument("--strong-kg", type=float, default=0.6)
    parser.add_argument("--brake-kg", type=float, default=1.0)
    parser.add_argument("--acceleration-off-mss", type=float, default=-3.0,
                        help="Antisex off / smooth pause threshold in host config (default: -3 m/s²)")
    args = parser.parse_args()
    assert args.pull_ms > 0 and args.brake_ms > 0 and args.amps_per_kg > 0
    assert 0 < args.unwind_kg < args.pull_kg
    assert 0 < args.brake_kg < args.pull_kg
    assert args.unwind_kg <= args.strong_kg < args.pull_kg
    assert args.strong_kg < args.pull_kg * args.pre_k < args.pull_kg
    assert args.brake_kg < args.pull_kg * args.pre_k
    assert args.pre_k < args.takeoff_k < 1.0
    assert args.fast_k > 1.0
    assert math.isfinite(args.acceleration_off_mss) and args.acceleration_off_mss < 0
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    settings = {key: value for key, value in vars(args).items() if key not in ("cc", "output", "plot", "debug")}
    settings.update(prepare_production_code(output))
    executable = output / "smooth_motor_test"
    flags = ["-DDEBUG_SMOOTH_MOTOR"] if args.debug else []
    subprocess.run(shlex.split(args.cc) + flags + ["-std=gnu99", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-Wno-old-style-declaration", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                   "-I", str(output), str(HERE / "harness.c"), "-lm", "-o", str(executable)], check=True)
    parameters = [args.pull_kg, args.pull_ms, args.brake_ms, args.amps_per_kg,
                  args.pre_k, args.unwind_kg, args.strong_kg, args.brake_kg, args.takeoff_k, args.fast_k,
                  args.acceleration_off_mss]
    csv_path = output / "transitions.csv"
    with csv_path.open("w") as stream:
        subprocess.run([str(executable), *map(str, parameters)], stdout=stream, check=True)
    traces = read_traces(csv_path)
    summary = check_traces(traces, settings)
    measurement_test = output / "measurement_filter_test"
    subprocess.run(shlex.split(args.cc) + ["-std=gnu99", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-fsanitize=undefined", "-fno-sanitize-recover=all", "-I", str(output),
                   str(HERE / "measurement_harness.c"), "-lm", "-o", str(measurement_test)], check=True)
    subprocess.run([str(measurement_test)], check=True)
    (output / "results.json").write_text(json.dumps(
        {"settings": settings, "tests": summary, "measurement_filter_checks": "pass"}, indent=2) + "\n")
    if args.plot:
        from plot_results import plot_results
        plot_results(traces, settings, summary, output)
    print(f"Results: {output}")


if __name__ == "__main__":
    main()

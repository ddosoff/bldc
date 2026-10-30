# SkyPUFF smooth-motor transition tests

## Current firmware after rollback

The firmware again uses the original `amps_per_sec` controller, with a 20-loop
maximum step interval and a 50 ms measurement filter. Current decreases now
pause below `config.antisex_acceleration_off_mss`; an existing pause continues
at equality. The first update above the threshold holds the setpoint and resets
the timing, so the next update takes an ordinary step without catching up.
Current increases and direct brake/release/speed overrides clear the pause.

Run the production C pause checks (both signs, prebrake, equality, retargeting,
late resume, live threshold changes, and overrides; normal and debug builds):

```sh
python applications/skypuff/tests/skypuff_smooth/run_rate_tests.py --cc gcc
```

For the current PrePull /
Unwinding plots and host checks, run:

```sh
python applications/skypuff/tests/skypuff_smooth/plot_rate_transitions.py --cc gcc
```

This compiles the current C functions and plots a fixed 42 ↔ 2.2 kgf transition
at nominal 140 kgf, assuming one loop per millisecond. Results go to
`build/skypuff_rate_pre_unwind_140kg/`. It does not simulate the rope or the FSM's
strong-Unwinding target selection.

The older scripts below (`run_tests.py`, `plot_retarget.py`, and their C harnesses)
are retained for the reverted stage-based controller. Their assertions and
interfaces do not match the restored firmware.

## Previous stage-based controller

The runner compiles the **current production C functions** from
`applications/skypuff/app_skypuff.c`. The harness sends current/brake commands
straight to `smooth_motor_current` and `smooth_motor_brake`. It does not use
application state entry functions to select smoothing behavior. Hardware
interfaces and the clock are replaced by host stubs; the application FSM state
is deliberately changed during each run to verify that it has no effect.

No firmware settings or motor hardware are changed by these tests.

## Run

Requires Python 3 and a host GCC with UndefinedBehaviorSanitizer. Plotting also
requires matplotlib (for example, install it in a virtual environment).

```sh
python applications/skypuff/tests/skypuff_smooth/run_tests.py --cc gcc --plot
```

Without `--plot`, only the Python standard library and GCC are required.
`--debug` additionally compiles the `DEBUG_SMOOTH_MOTOR` code. `--output PATH`
selects the output directory, defaulting to the ignored `build/skypuff_smooth/`.
Outputs include generated C includes, the executable, `transitions.csv`,
`results.json`, and PNG/SVG plots: `current_transitions`, `braking_transitions`,
`takeoff_transitions`, `reduction_transitions`, `fast_pull_transition`,
`manual_braking_transitions` (six starting states), and the combined `transitions` figure.

`plot_results.plot_growth_comparison(cases, output)` plots five increasing-current
transitions for two nominal pulls on one PNG/SVG figure. Each case is a
`(read_traces(csv_path), settings)` pair from a test run. The left axes show amperes;
the right axes show the equivalent setpoint in kgf. This is a setpoint comparison,
not the mechanical cable/drum simulation.

`python applications/skypuff/tests/skypuff_smooth/plot_retarget.py --cc gcc` compiles the same production
functions and plots PrePull → Unwinding, interrupted by a Pull command at 2 s,
for 100 and 60 kgf nominal pull. The initial strong-Unwinding timer is included.
Commands run before due setpoint updates, as in the application's main loop:
the new curve starts from the last applied setpoint and finishes at 6 s.
Acceleration is zero; the INI threshold is -3 m/s². C assertions and independent
per-sample checks verify continuity, current mode, curve values and completion.
Results are saved in `build/skypuff_pre_unwind_pull_retarget/` as PNG, SVG, CSV
and JSON, including the production-source hash.

Add `--scenario fast_unwind` to plot FastPull → Unwinding, starting at 125% of
nominal pull. The fixture holds speed at -2 m/s and acceleration at zero, so
the real `unwinding_current()` helper requests 5.2 kgf for the first 2 s, then
2.2 kgf. That target change restarts the ramp from the applied setpoint for a
full 4 s, completing at 6 s without acceleration pauses. At speeds above the
configured -1 m/s threshold, strong Unwinding can remain active past 2 s.
Outputs for this scenario go to `build/skypuff_fast_unwinding/`.

Use `--scenario fast_unwind_direct` for a single `smooth_motor` transition from
FastPull to a fixed 2.2 kgf Unwinding target. The fixture does not request the
strong-Unwinding target. With zero acceleration this takes exactly 4 s;
the normal acceleration-off hold rule still applies to the decreasing current.
Outputs go to `build/skypuff_fast_unwinding_direct/`.

## Default scenario

| Setting | Value |
| --- | --- |
| Nominal pull | 60 kgf |
| Current-control stage | 4000 ms |
| Each stage before braking | 2000 ms |
| Current/force conversion | 3 A/kgf |
| PrePull | 30% = 18 kgf |
| TakeoffPull | 65% = 39 kgf |
| FastPull | 125% = 75 kgf |
| Normal Unwinding | 0.4 kgf |
| Strong Unwinding | 0.6 kgf |
| Manual braking | 1 kgf |
| Antisex off / smooth pause threshold | -3 m/s² |

The exponent, update interval, clock frequency, measurement interval and shared
speed/acceleration filter period are read from production code.
`measurement_harness.c` also runs the actual `measurement_tick` function with
known tachometer motion: steady 7 m/s must stay steady, and a -2 m/s² acceleration
must cross the filtered -1 m/s² threshold sooner with the configured filter
period than with a doubled period (currently 50 vs 100 ms). Measurements still
update every 5 ms; smooth_motor setpoints update at intervals up to 20 ms.
`--acceleration-off-mss` sets `config.antisex_acceleration_off_mss` in the harness;
its default is -3 m/s², matching `set_example_config` and the INI.
Remaining defaults come from `set_example_config`, except for the requested
60 kgf pull and 4 s / 2 s periods, and the 65% takeoff and 125% fast-pull coefficients from
`skypuff.ini`. `--help` lists scenario overrides.

To use the values found in `Downloads/6.0-beta-51/skypuff.ini`, while keeping
the requested 60 kgf nominal pull (the INI contains 62.4 kgf), run:

```sh
python applications/skypuff/tests/skypuff_smooth/run_tests.py --cc gcc --plot \
  --pull-kg 60 --pull-ms 4000 --brake-ms 2000 --pre-k 0.3 --takeoff-k 0.65 --fast-k 1.25 \
  --amps-per-kg 2.45 --unwind-kg 2.2 --strong-kg 5.2 --brake-kg 1.8
```

These values give a PrePull threshold of 18 kgf / 44.1 A. The INI expresses
periods in seconds and `pre_pull_k` as a percentage; runner arguments use
milliseconds and a fractional coefficient.

The two takeoff plots start from settled setpoints: 18 → 39 kgf and 39 → 60 kgf.
Each ramp uses the 4 s `pull_applying_period`; `takeoff_period` controls the
application state timing and does not set the smooth_motor ramp duration.

Each plotted transition has a fixed requested target, so normal and strong
unwinding can be inspected separately. A separate regression test runs the real
`unwinding_current()` helper, including its initial two-second strong period.

Antisex is disabled and position is outside instant braking/slowing zones.
Graphs show commanded current converted to kgf, not measured rope tension or a
mechanical simulation. Braking is a different motor mode and is colored orange;
its torque direction depends on rotation.

## Rules and expected results

`PrePull = |pre_pull_k * pull_current|`. An increase in current magnitude whose
target is at or below PrePull is instantaneous. In current control, every decrease is smooth,
including decreases starting at or below PrePull. Larger increases start from
the actual current setpoint, without an initial jump to PrePull.

Every smooth transition goes **directly from the actual current to the target**.
PrePull is only an instant-command threshold, never an intermediate target.
One curve takes a single configured period: 4 seconds in current mode or
2 seconds before braking, plus any acceleration pauses.

With `smooth_motor_p = 1`, each stage is linear, with a constant rate of change.
With `smooth_motor_p = 2`, it is a quadratic ease-out parabola. The general formula is:

```text
u = clamp(elapsed / T, 0, 1)
I = I_start + (I_target - I_start) * (1 - (1 - u)^smooth_motor_p)
```

A decrease starting exactly at PrePull is smooth, including before braking.
A new brake request is instantaneous if the starting current magnitude is
strictly below PrePull and the requested brake magnitude is at or below PrePull. This check
is made only when starting a transition: reaching PrePull during an existing
brake ramp does not finish it early. Otherwise, the direct ramp retains the starting
drive-current sign. After reaching the requested brake magnitude, the motor
switches immediately to brake mode at that magnitude.

The reference traces below use measured acceleration = 0, so they contain no
acceleration pauses. A decrease is now held when filtered acceleration falls
below `config.antisex_acceleration_off_mss` (INI `acceleration_off_mss`, -3 m/s²
in the fixture), and resumes strictly above it.
The curve clock is held too, so those pauses extend the times below.
The runtime-config regression changes this threshold during a curve and verifies
that the next update holds or resumes without catching up, for both signs and modes.

For the INI values and the requested nominal pull of 60 kgf:

| Transition | Expected setpoints and timing |
| --- | --- |
| Unwinding → Pre_Pull | 2.2 → 18 kgf instantly |
| Strong_unwinding → Pre_Pull | 5.2 → 18 kgf instantly |
| Pull → Unwinding | 60 → 2.2 kgf in **4 s** |
| Pull → Strong_unwinding | 60 → 5.2 kgf in **4 s** |
| Pull → Manual_braking | 60 → 1.8 kgf in **2 s**, then brake |
| Unwinding → Manual_braking | Drive current 2.2 → brake 1.8 **instantly** |
| Manual_braking → Unwinding | Brake 1.8 → drive current 2.2 instantly |
| Pre_Pull → Takeoff_Pull | 18 → 39 in **4 s** |
| Takeoff_Pull → Pull | 39 → 60 in **4 s** |
| Pre_Pull → Unwinding | 18 → 2.2 in **4 s** |
| Takeoff_Pull → Unwinding | 39 → 2.2 kgf in **4 s** |
| Pull → Fast_Pull | 60 → 75 in **4 s**; at a 140 kgf nominal pull, 140 → 175 in **4 s** |
| Fast_Pull → Manual_braking | 75 → 1.8 kgf in **2 s**, then brake |
| Takeoff_Pull → Manual_braking | 39 → 1.8 kgf in **2 s**, then brake |
| Pre_Pull → Manual_braking | 18 → 1.8 kgf in **2 s**, then brake |
| Strong_unwinding → Manual_braking | Drive current 5.2 → brake 1.8 **instantly** |

At a nominal pull of 140 kgf, the six-transition braking figure starts from
FastPull 175, Pull 140, TakeoffPull 91, PrePull 42, Strong Unwinding 5.2, and
Unwinding 2.2 kgf. The first four ramp directly to 1.8 kgf and enter brake mode
at 2 s; the remaining two enter brake mode immediately.

Repeated commands preserve the curve and its progress. A changed target or period
starts a new transition from the current setpoint under the same rules, including
the immediate switch for a new small brake request.
Unchanged setpoints and nonpositive periods complete immediately. Release and
speed commands cancel smoothing; near-zero braking/slowing zones retain their
immediate-brake override.

## Assertions

Every millisecond sample is compared with an independent closed-form reference
and the actual setpoint-update schedule. Tests check motor mode, force/current
conversion, finite values, completion times, and the stable final state.

Additional C assertions cover both current signs, both sides of the PrePull
boundary (including the next representable float above it), small downward
transitions, large reversals, repeated commands across the PrePull threshold,
retargeting, brake magnitude changes, late updates, zero duration, release/speed
cancellation, and the near-zero braking override. The application FSM state
changes during simulation to verify that smoothing is independent of it.

The real `unwinding_current()` helper is tested with 1, 1.5, 2 and 3 ms control-loop
iterations and a simulated 5 ms speed-measurement interval. Its initial strong
period is checked against system time, including clock wraparound. Speed may
still request strong unwinding after that period. With the INI fixture and speed
below the strong threshold, the target changes from 5.2 to 2.2 kgf at about 2 s.
That new target starts a fresh 4-second curve from the current setpoint, reaching
2.2 kgf at about 6 s. A fixed Unwinding target takes only 4 s without pauses.

The same slow-loop profiles check current/braking ramps and the production update
scheduler: completion must occur on the first iteration at or after the system
clock deadline, including wraparound. A late update does not shift the deadline.
Update intervals must not exceed
`smooth_max_step_delay` plus one loop iteration.

Dedicated acceleration tests cover repeated holds, both current signs,
current-control and braking targets, equality at the threshold, repeated requests,
resumption without catch-up, higher retargets, instant overrides and pauses across
system-clock wraparound. The deadline cannot finish a held stage.

For graphs with mechanical acceleration fed back into the actual C controller,
see `applications/skypuff/tests/skypuff_dynamics/README.md` and `closed_loop.py` there. The simple
reference plots from this runner are not a simulation of the drum and rope.

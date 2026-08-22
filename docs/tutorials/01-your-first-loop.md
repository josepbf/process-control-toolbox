# Tutorial 1: your first loop

By the end of this page you will have simulated a level loop, scored it, and
drawn the standard figure — using the same objects and the same runner that
every experiment in the project uses.

## 1. The plant

`Tank` is the workhorse test process of classical control: a first-order lag
plus dead time. Level `h` [%] driven by an inlet valve `u` [%] whose effect
arrives `theta` seconds late, pushed around by an unmeasured load `d`.

```python
from src.plants.tank import Tank

plant = Tank(
    K=1.5,            # gain: % level per % valve
    tau=60.0,         # time constant [s]
    theta=15.0,       # transport delay valve -> level [s]
    Kd=1.0,           # disturbance gain
    h0=30.0,          # initial level [%]
    u_min=0.0, u_max=100.0,
    noise_std=0.15,   # measurement noise sigma [% level]
    seed=7,
)
```

Two properties are worth knowing straight away:

```pycon
>>> plant.fopdt
{'K': 1.5, 'tau': 60.0, 'theta': 15.0}
>>> plant.theta / plant.tau
0.25
>>> plant.steady_input(30.0)      # valve position that holds 30 %
20.0
```

`fopdt` is the **true** model. A controller is only entitled to it when we
deliberately grant a no-mismatch upper bound — which phase 1 does everywhere,
and says so. See [fairness rule 3](../concepts/fairness.md).

$\theta/\tau = 0.25$ puts this loop in the comfortable middle: a PI handles it
without difficulty. That is on purpose — [experiment 5](../articles/05-dead-time-sweep.md)
is where it gets hard.

## 2. The scenario

A scenario is what the world does to the loop: a sample time, a run length, a
setpoint programme, a disturbance programme, a seed, and named reporting
windows.

```python
from src.harness.scenarios import setpoint_and_load

scenario = setpoint_and_load(
    dt=1.0,
    y_start=30.0, y_step=50.0, t_step=100.0,   # setpoint 30 -> 50 % at t=100 s
    d_load=-12.0, t_load=700.0,                # load step at t=700 s
    t_final=1400.0,
    seed=7,
)
```

```pycon
>>> scenario.n_steps
1400
>>> scenario.windows
{'setpoint': (100.0, 700.0), 'disturbance': (700.0, 1400.0)}
```

Those two windows are the two jobs a regulatory loop actually has, and they
are in tension. Reporting them separately is what keeps the trade visible.

## 3. The tuning

No hand-tuning. Pick a published rule and let the number fall where it may.

```python
from src.tuning.rules import simc_pi

tuning = simc_pi(**plant.fopdt)
```

```pycon
>>> f"{tuning.rule}: Kc={tuning.Kc:.3f} Ti={tuning.Ti:.1f}"
'SIMC PI (Skogestad 2003), tau_c=15: Kc=1.333 Ti=60.0'
```

## 4. The controller

```python
from src.controllers.pid import PIDController

pi = PIDController(
    **tuning.as_kwargs(),          # Kc, Ti, Td
    dt=scenario.dt,
    u_min=plant.u_min[0],          # the controller must know the actuator range
    u_max=plant.u_max[0],          # for its anti-windup to be honest
    u0=plant.steady_input(30.0),   # bumpless start at the holding valve position
    name="PI (SIMC)",
    tuning_note=tuning.rule,       # fairness rule 1: the citation travels with the run
)
```

!!! tip "Why `u0` matters"
    Without it, the controller starts with a zero integral and steps the valve
    at t = 0. `u0` chooses the initial integral so the first output *is* the
    nominal valve position — a bumpless start, exactly as a real controller
    does on transfer from manual to automatic.

    `u_min` / `u_max` do **not** give the controller special powers: the plant
    saturates regardless. They are what the anti-windup back-calculation needs
    in order to know how much of its requested move was refused.

## 5. Run it

```python
from src.harness.simulate import simulate

df = simulate(plant, pi, scenario)
```

One tidy row per sample:

```text
     t          y    sp          u    d  d_meas  violation  violated  solve_time
0  0.0  30.000185  30.0  20.000000  0.0     0.0        0.0     False    0.000013
1  1.0  30.044812  30.0  19.940493  0.0     0.0        0.0     False    0.000007
2  2.0  29.958879  30.0  20.054074  0.0     0.0        0.0     False    0.000005
```

The row for time `t` holds **the measurement the controller acted on and the
move it made in response**. See the
[timing convention](../concepts/architecture.md#the-timing-convention).

`simulate()` deep-copies the plant by default, so you can run the same plant
object against a dozen controllers without any of them contaminating the next.

## 6. Score it

```python
from src.harness.metrics import compute_metrics

compute_metrics(df, scenario.windows["disturbance"])
```

```python
{'IAE': 410.898, 'ISE': 957.776, 'ITAE': 52277.142,
 'settling_2pct_s': 153.0, 'overshoot_pct': nan, 'peak_dev': 4.487,
 'ss_offset': -0.0, 'TV_u': 157.498, 'max_du': 1.066, 'reversals': 34,
 'n_violations': 0, 'violation_integral': 0.0,
 'solve_ms_mean': 0.003, 'solve_ms_p95': 0.003}
```

Three things to notice.

- `overshoot_pct` is `NaN`. There is no setpoint change in this window, and
  overshoot of a step that never happened is not a number worth printing —
  read `peak_dev` instead. This is
  [deliberate](../concepts/metrics.md#the-three-that-need-care).
- `ss_offset` is zero to three decimals. That is the integral action doing its
  job.
- `TV_u` and `reversals` are there whether you asked for them or not. Tracking
  bought with violent valve movement is not a win.

For the whole run at once, across every window:

```python
from src.harness.metrics import summarize, format_table

table = summarize({"PI (SIMC)": df}, windows=scenario.windows)
print(format_table(table))
```

```text
                             IAE       ITAE  settling_2pct_s  overshoot_pct  peak_dev  ss_offset      TV_u  reversals
controller window
PI (SIMC)  full         1147.124 446325.935          845.000          6.010    20.161     -0.003   343.488         52
           setpoint      715.414  33334.256           92.000          5.996    20.161      0.008   139.904         15
           disturbance   410.898  52277.142          153.000            NaN     4.487     -0.000   157.498         34
```

## 7. Draw it

```python
from src.harness.plotting import plot_runs

plot_runs(
    {"PI (SIMC)": df},
    title="A first closed loop",
    scenario=scenario,
    path="results/tutorial01.png",
)
```

Three panels on a shared time axis: controlled variable and setpoint,
manipulated variable with the actuator limits drawn in, and the disturbance
that was applied. Panel 2 is not optional — see
[Getting started](../getting-started.md#run-your-first-experiment).

## 8. Compare two controllers

The whole point of the harness. `run_all` gives every controller the same
plant, the same scenario and the same seed, so the only difference is the
control law.

```python
from src.controllers.onoff import OnOffController
from src.harness.simulate import run_all

onoff = OnOffController(u_on=100.0, u_off=0.0, hysteresis=1.0)

runs = run_all(plant, {"ON/OFF": onoff, "PI (SIMC)": pi}, scenario)
print(format_table(summarize(runs, windows=scenario.windows)))
```

That is [experiment 1](../articles/01-onoff-vs-pid.md), and the answer is that
ON/OFF never settles — its `settling_2pct_s` comes back `NaN`, and its
reversal count gives away why.

## The complete script

```python
--8<-- "docs/snippets/tutorial01.py"
```

## Next

[Tutorial 2: tuning a loop](02-tuning-a-loop.md) — where the `simc_pi` call
above came from, and how to check what it cost you in robustness.

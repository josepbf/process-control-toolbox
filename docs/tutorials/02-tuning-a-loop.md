# Tutorial 2: tuning a loop

[Tutorial 1](01-your-first-loop.md) handed `simc_pi()` the plant's true
parameters, because they were sitting right there in `plant.fopdt`. On a real
plant they are not. This tutorial does it the way it is actually done: bump
the valve, fit a model to the reaction curve, apply a rule, and then check
what the rule cost you in robustness.

## 1. The step test

Hold the valve steady, step it, and watch. This is the *reaction curve*, and
it is the oldest identification experiment there is.

```python
import numpy as np
from process_control.plants.tank import Tank

def open_loop_step(plant, u_before, u_after, t_step=50.0, dt=1.0, t_final=600.0):
    """Hold the valve steady, step it, and record the reaction curve."""
    plant.reset()
    t, y = [0.0], [float(plant.measure()[0])]
    for k in range(int(t_final / dt)):
        u = u_before if k * dt < t_step else u_after
        y.append(float(plant.step([u], dt)[0]))
        t.append((k + 1) * dt)
    return np.array(t), np.array(y)
```

Note that no `Controller` appears. `Plant.step()` is a plain function of the
move you hand it, so open-loop work needs no harness at all.

## 2. Fit FOPDT

The classic two-point fit: read the times at which the response has completed
35.3 % and 85.3 % of its final change, and solve for $\tau$ and $\theta$.

```python
def fit_fopdt(t, y, u_step, t_step):
    y0 = float(np.mean(y[(t > t_step - 20.0) & (t <= t_step)]))
    y_inf = float(np.mean(y[t >= 0.9 * t[-1]]))
    K = (y_inf - y0) / u_step

    def time_at(frac):
        target = y0 + frac * (y_inf - y0)
        i = int(np.argmax(np.sign(y_inf - y0) * (y - target) >= 0))
        return float(np.interp(target, [y[i - 1], y[i]], [t[i - 1], t[i]]))

    t1, t2 = time_at(0.353), time_at(0.853)
    return {"K": K, "tau": 0.67 * (t2 - t1), "theta": 1.3 * t1 - 0.29 * t2 - t_step}
```

Averaging over a window for `y0` and `y_inf` rather than taking single samples
is what keeps the fit usable in noise — and every real reaction curve has
noise.

Run it on a clean plant and then on a noisy one:

```text
noise=0.0 : {'K': 1.5, 'tau': 59.52, 'theta': 16.26}
noise=0.15: {'K': 1.5, 'tau': 56.13, 'theta': 17.79}
true      : {'K': 1.5, 'tau': 60.0,  'theta': 15.0}
```

!!! note "The bias is not random"
    Even noise-free, the two-point fit reports θ = 16.3 s against a true 15 s.
    A FOPDT fit absorbs the initial curvature of the response into the dead
    time — the same instinct as [the half rule](../concepts/dead-time.md#everything-a-pid-cannot-represent-becomes-dead-time).
    Whatever the two-parameter model cannot represent becomes θ.

    It errs in the safe direction: an over-estimated θ produces a *lower*
    gain, hence a more conservative loop.

## 3. Apply a rule

```python
from process_control.tuning.rules import simc_pi
tuning = simc_pi(**model)
```

Which rule? That depends on what the loop is for, and the honest answer is in
[experiment 3](../articles/03-tuning-shootout.md). Short version:

| you want | use |
|---|---|
| the project's standard baseline | `simc_pi(K, tau, theta)` — τ<sub>c</sub> = θ |
| a robust loop that will not be re-tuned | `amigo_pi(K, tau, theta)` |
| a deliberately aggressive reference | `ziegler_nichols_open_loop(...)` |
| an integrating process (level, drum, silo) | `simc_integrating(k_prime, theta)` |
| a surge tank whose job is absorbing upsets | `averaging_level_pi(...)` |

## 4. Check what it cost

This is the step most people skip, and it is the one that separates "we tuned
the PID" from "we tuned the PID well". Compute the
[maximum sensitivity](../concepts/robustness.md) of the resulting loop.

```python
from process_control.tuning.analysis import pid_on_fopdt

for label, tuning in rules.items():
    on_model = pid_on_fopdt(**model,        Kc=tuning.Kc, Ti=tuning.Ti)
    on_truth = pid_on_fopdt(**plant.fopdt,  Kc=tuning.Kc, Ti=tuning.Ti)
    print(f"{label:14s} Kc={tuning.Kc:5.2f} Ti={tuning.Ti:6.1f}  "
          f"Ms(identified)={on_model['Ms']:.2f}  Ms(true plant)={on_truth['Ms']:.2f}")
```

```text
SIMC           Kc= 1.05 Ti=  56.1  Ms(identified)=1.59  Ms(true plant)=1.44
AMIGO          Kc= 0.45 Ti=  48.2  Ms(identified)=1.23  Ms(true plant)=1.18
Lambda         Kc= 0.51 Ti=  56.1  Ms(identified)=1.24  Ms(true plant)=1.19
ZN open loop   Kc= 1.89 Ti=  59.2  Ms(identified)=2.60  Ms(true plant)=2.03
```

Two readings, and the second is the more useful one.

**Each rule lands where it is supposed to.** SIMC at Ms 1.59, AMIGO at 1.23
against its design target of ≤ 1.4, ZN at 2.60 — a loop that will oscillate
the first time the process gain moves. The rules are behaving exactly as
advertised.

**The identification error made every loop safer, not riskier.** Ms on the
true plant is *lower* than Ms on the identified model in every row, because
the fit over-estimated θ and the rules therefore chose lower gains than the
real process could have carried. That direction is luck rather than design —
a fit that under-estimated θ would have moved every number the other way — but
it is why the two-point method survives in the field.

!!! warning "This is not a substitute for a mismatch study"
    A single-step identification against a plant with the exact FOPDT
    structure being fitted is the friendliest possible case. Real model
    mismatch — wrong structure, drifting gain, operating-point dependence —
    is what a deliberate mismatch sweep exists to price.

## 5. Now close the loop

```python
from process_control.controllers.pid import PIDController
from process_control.harness.simulate import run_all
from process_control.harness.metrics import summarize, format_table
from process_control.harness.scenarios import setpoint_and_load

scenario = setpoint_and_load(dt=1.0, y_start=30.0, y_step=50.0, seed=7)
controllers = {
    label: PIDController(
        **t.as_kwargs(), dt=scenario.dt,
        u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=plant.steady_input(30.0), name=label, tuning_note=t.rule,
    )
    for label, t in rules.items()
}
print(format_table(summarize(run_all(plant, controllers, scenario),
                             windows=scenario.windows)))
```

You now have every ingredient of an experiment script. The pattern —
plant, scenario, a dict of named tunings, `run_all`, `summarize` — is
literally what every file in `experiments/` does.

## The complete script

```python
--8<-- "docs/snippets/tutorial02.py"
```

## Next

[Tutorial 3: relay auto-tuning](03-relay-autotuning.md) — getting the same two
numbers with no model and no step test, the way the autotune button does.

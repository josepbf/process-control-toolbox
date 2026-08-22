# Tutorial 5: writing a controller

A `Controller` subclass supplies two methods and three declarations. That is
the whole interface — it is deliberately small, because from phase 2 on it has
to hold an MPC as comfortably as it holds an ON/OFF switch.

## The contract

```python
class Controller(ABC):
    name: str = "controller"                     # label in plots and tables
    uses_measured_disturbance: bool = False      # do you read an instrumented d?
    tuning_note: str = "unspecified"             # how you were tuned, with citation

    def compute(self, y, setpoint, t) -> np.ndarray:   # required
    def reset(self) -> None:                           # required
    def describe(self) -> dict:                        # optional, override to add fields
```

!!! warning "`uses_measured_disturbance` is a modelling claim, not a switch"
    Declaring it `True` asserts that the disturbance is instrumented on the
    real plant. The harness then passes `d=` to `compute()`. A feedback-only
    controller must leave it `False`, so that no controller can quietly
    benefit from information the others do not have. This is
    [fairness rule 2](../concepts/fairness.md) in mechanism form.

`tuning_note` defaults to the deliberately embarrassing `"unspecified"`. If
that string appears in a results table, the run is not admissible.

## Worked example: a valve slew-rate limiter

Large valves, dampers and variable-speed drives cannot move arbitrarily fast.
A motorised control valve is typically 30–60 s from fully shut to fully open —
on a 1 s sample time, a limit of 1.7–3.3 % per second. A control law that
assumes it can step the actuator anywhere it likes is modelling something the
plant cannot do.

We will build it as a **composition** controller: it holds another controller
and modifies its output, in the same style as `TimeProportioningController`.

### 1. Hold the inner controller and its state

```python
class RateLimited(Controller):
    def __init__(self, inner, max_rate, dt, u0=0.0, name=None):
        self.inner = inner
        self.max_rate = float(max_rate)      # units of u per second
        self.dt = float(dt)
        self.u0 = float(u0)
        self.name = name or f"{inner.name} + rate limit"
        self.tuning_note = f"slew limit {self.max_rate:g}/s; inner: {inner.tuning_note}"
        self.uses_measured_disturbance = inner.uses_measured_disturbance
        self.reset()
```

That last line is easy to miss and important: pass the declaration through, or
wrapping a feedforward controller would silently stop it being handed the
disturbance measurement, and the run would be scored against a controller that
is not the one you thought you built.

### 2. `reset()` must reach all the way down

```python
    def reset(self) -> None:
        self.inner.reset()
        self._u = self.u0
```

`reset()` is called at the top of every `simulate()`. If it does not clear
*every* piece of state — integrators, filters, histories, warm starts, and any
nested controller — then run B is contaminated by run A and the comparison is
void.

### 3. `compute()`

```python
    def compute(self, y, setpoint, t, **kwargs) -> np.ndarray:
        demand = float(np.atleast_1d(self.inner.compute(y, setpoint, t, **kwargs))[0])
        step = self.max_rate * self.dt
        self._u = float(np.clip(demand, self._u - step, self._u + step))
        return np.array([self._u])
```

Always return an array, even for a single input — the harness treats `u` as a
vector and suffixes the columns when it has more than one channel.

The `**kwargs` passthrough is what lets this wrapper sit on top of a
feedforward controller, whose `compute()` takes an extra `d=` argument.

### 4. `describe()` — put your parameters in the record

```python
    def describe(self) -> dict:
        return {
            "controller": self.name, "tuning": self.tuning_note,
            "max_rate": self.max_rate, "inner": self.inner.describe(),
        }
```

`describe()` lands in `df.attrs["controller_info"]`, so a saved run carries its
own full configuration. Nesting the inner controller's record means a cascade
or a wrapper stays self-describing all the way down:

```python
{'controller': 'PI + rate limit',
 'tuning': 'slew limit 0.5/s; inner: SIMC PI (Skogestad 2003), tau_c=15',
 'max_rate': 0.5,
 'inner': {'controller': 'PI', 'tuning': 'SIMC PI (Skogestad 2003), tau_c=15',
           'Kc': 1.333, 'Ti': 60.0, 'Td': 0.0}}
```

## What it does

Same plant, same tuning, same seed — the only difference is how fast the valve
is allowed to move.

```text
                               IAE  settling_2pct_s  overshoot_pct  peak_dev      TV_u    max_du  reversals
controller   window
PI           setpoint      715.414           92.000          5.996    20.161   139.904     1.185         15
PI + 2.0 %/s setpoint      905.270          135.000         13.073    20.161   166.097     2.000         15
PI + 0.5 %/s setpoint     1696.328          248.000         37.943    20.161   152.707     0.500          0
PI + 0.2 %/s setpoint     3624.229          402.000         72.206    20.158   104.768     0.200          0
```

**Overshoot goes from 6 % to 72 %.** That is not the rate limit being harsh —
it is windup, one level down from
[experiment 2](../articles/02-integral-windup.md). The PI's back-calculation
knows about `u_min` and `u_max`, so it unwinds correctly when the *valve* hits
a travel limit. It knows nothing about the rate limiter sitting downstream of
it, so when the limiter refuses part of a move, the integrator carries on
accumulating against a demand that is never delivered.

!!! tip "Two correct fixes, both already in the codebase"
    **Tell the inner controller what it can actually have.** This is exactly
    what `FeedforwardPID` does — it narrows `feedback.u_min` / `u_max` every
    sample by the amount feedforward has already claimed, so the anti-windup
    stays honest. The same trick works here: set the inner controller's limits
    to `[self._u - step, self._u + step]` before calling it.

    **Or use the velocity form.** `VelocityPIDController` has no integral
    state to run away — the integral lives in `u_{k-1}`, which is clipped
    every sample. Anti-windup is a property of the form rather than a bolt-on,
    which is a large part of why DCS and PLC function blocks implement it.

    Leaving the bug in is the more instructive tutorial. Fixing it is a good
    exercise, and the metric to watch is `overshoot_pct` on the setpoint
    window.

!!! bug "And a metric artefact, reported rather than hidden"
    `reversals` reads **0** for the two tightest rate limits. That is not the
    valve holding still — it is the reversal threshold. `compute_metrics` sets
    `eps` to 0.5 % of the actuator span (0.5 on a 0–100 valve) to stop
    measurement-noise dither counting as wear, and the rate-limited moves are
    exactly 0.5 and 0.2, so every one of them is filtered out. Read `TV_u` in
    that row instead.

    The lesson generalises: a metric with a threshold in it has a regime where
    the threshold, not the process, is what you are measuring. See
    [Metrics](../concepts/metrics.md#control-effort).

## The six controllers already in the project

| controller | what it demonstrates |
|---|---|
| [`OnOffController`](../reference/controllers.md#onoffcontroller) | the baseline everything else has to beat |
| [`TimeProportioningController`](../reference/controllers.md#timeproportioningcontroller) | PWM — composition over a continuous law |
| [`PIDController`](../reference/controllers.md#pidcontroller) | positional form, back-calculation, filtered derivative |
| [`VelocityPIDController`](../reference/controllers.md#velocitypidcontroller) | incremental form — structural anti-windup |
| [`CascadeController`](../reference/controllers.md#cascadecontroller) | a controller whose output is another controller's setpoint |
| [`FeedforwardPID`](../reference/controllers.md#feedforwardpid) | the only one that declares `uses_measured_disturbance` |

## Checklist for a new controller

- [ ] `name` and `tuning_note` set, with a citation in the note
- [ ] `uses_measured_disturbance` declared honestly, and passed through if you wrap
- [ ] `reset()` clears **all** state, including nested controllers
- [ ] `compute()` returns an `np.ndarray`
- [ ] `describe()` overridden to record your parameters
- [ ] Anti-windup thought about, if there is an integrator anywhere
- [ ] A test: does the loop reach setpoint without offset, and does `reset()`
      actually make two consecutive runs identical?

## The complete script

```python
--8<-- "docs/snippets/tutorial05.py"
```

## Next

[Tutorial 6: designing an experiment](06-designing-an-experiment.md).

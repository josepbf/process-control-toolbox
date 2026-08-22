# Tutorial 3: relay auto-tuning

The closed-loop tuning rules need two numbers: the **ultimate gain** $K_u$, at
which the loop oscillates with constant amplitude, and the **ultimate period**
$P_u$ of that oscillation. Ziegler and Nichols' original way to get them was
to raise the proportional gain until the plant oscillated — deliberately
walking a real process to the edge of instability, with no guarantee of
stopping there.

Åström and Hägglund's 1984 relay experiment replaced it, and is what sits
behind the "autotune" button on essentially every industrial controller.

## How it works

Replace the controller with a **relay** of amplitude $h$. The loop settles
into a limit cycle whose frequency is, by construction, the frequency at which
the process has −180° of phase — the ultimate frequency. The oscillation is
*bounded*: its amplitude is set by the relay amplitude the engineer chose, and
it can be made as small as the measurement noise allows.

Describing-function analysis then gives the gain from the amplitude $a$ of the
resulting oscillation:

\[
K_u = \frac{4h}{\pi a} \quad\text{(ideal relay)}, \qquad
K_u = \frac{4h}{\pi\sqrt{a^2 - \varepsilon^2}} \quad\text{(hysteresis } \varepsilon)
\]

## Run one

```python
from src.plants.tank import Tank
from src.tuning.relay import relay_autotune

plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=50.0, noise_std=0.15, seed=7)

relay = relay_autotune(
    plant,
    setpoint=50.0,                        # the operating point to oscillate about
    u_bias=plant.steady_input(50.0),      # the valve position that holds it
    h=10.0,                               # relay amplitude: 10 % of valve travel
    hysteresis=0.5,                       # ~3 sigma of measurement noise
    dt=1.0, t_final=900.0, seed=7,
)
```

```pycon
>>> relay.summary()
'Ku = 3.514, Pu = 58.7 s from 14 cycles (amplitude 3.66, period spread 1.2 s)'
```

The `RelayResult` carries more than the two headline numbers: `periods` and
`amplitudes` per cycle (the spread is your confidence interval), and `df`, the
full run, so you can plot the experiment itself.

!!! tip "Choosing `h` is the real decision"
    The oscillation in `y` scales with the relay amplitude, so `h` trades
    identification quality against how much the experiment upsets production.
    That is the central practical decision of an autotune, and the reason
    operators are asked before the button is pressed. 10 % of valve travel,
    producing a ±3.7 % level swing here, is a reasonable industrial choice.

## How good is it?

This plant's ultimate values are known analytically, so the error can be
measured rather than guessed:

```python
from src.tuning.analysis import ultimate_gain_period
Ku_true, Pu_true = ultimate_gain_period(K=1.5, tau=60.0, theta=15.0)
# 4.623, 54.9
```

| | ultimate gain Ku | ultimate period Pu |
|---|---:|---:|
| relay experiment | 3.51 | 58.7 s |
| analytic truth | 4.62 | 54.9 s |
| **error** | **−24 %** | **+7 %** |

**The period comes out nearly right and the gain conservatively wrong**, and
both are explainable rather than incidental.

- $P_u$ is close *by construction* — the limit cycle sits at the −180° phase
  frequency, which is the definition of the ultimate frequency.
- $K_u$ comes from a describing-function approximation that keeps only the
  fundamental of the relay's square wave. With θ/τ = 0.25 the process does not
  filter the harmonics well, so the estimate suffers. **The error direction is
  safe: it under-gains.**

## What the hysteresis buys and costs

Run the same experiment four ways:

```text
noise=0.0   hysteresis=0.0  Ku= 3.703 ( -19.9 %)  Pu= 56.0 ( +1.9 %)  cycles= 14
noise=0.0   hysteresis=0.5  Ku= 3.498 ( -24.3 %)  Pu= 60.0 ( +9.2 %)  cycles= 13
noise=0.15  hysteresis=0.5  Ku= 3.514 ( -24.0 %)  Pu= 58.7 ( +6.9 %)  cycles= 14
noise=0.15  hysteresis=0.0  Ku= 5.033 ( +8.9 %)   Pu= 41.6 ( -24.2 %) cycles= 21
```

Read the four rows as a 2×2:

**Row 1 — clean plant, no hysteresis.** The best case, and the residual −20 %
on $K_u$ is the describing-function error alone.

**Row 2 — clean plant, with hysteresis.** The period stretches by 9 %. That is
the phase lag the hysteresis adds: the relay waits before switching, so the
limit cycle is a little slower than the true ultimate frequency. This is the
price of the deadband.

**Row 3 — noisy plant, with hysteresis.** Essentially identical to row 2. The
hysteresis has done its job: the measurement noise has been rejected entirely,
and the estimate is as good as it was on the clean plant.

**Row 4 — noisy plant, no hysteresis.** The experiment falls apart. 21 cycles
instead of 14, a period 24 % *short*, and a $K_u$ 9 % **too high** — the
dangerous direction. Without a deadband the relay chatters on measurement
noise and the experiment measures the noise instead of the process.

!!! danger "The hysteresis is not optional"
    Rule of thumb: 2–3 times the peak-to-peak measurement noise. Its cost is
    a small conservative bias in $K_u$; its absence costs you the experiment.

Internally, the period is measured from the **relay's own output** rather than
from zero crossings of the measurement — the relay output is clean by
construction, while the measurement crosses its setpoint several times per
transition when the noise is comparable to the local slope. A real autotuner
reads its own output for exactly this reason.

## Turning it into a controller

```python
from src.tuning.rules import tyreus_luyben, ziegler_nichols_closed_loop

zn = ziegler_nichols_closed_loop(relay.Ku, relay.Pu, kind="PI")
tl = tyreus_luyben(relay.Ku, relay.Pu, kind="PI")
```

```text
ZN-CL (from relay)           Kc= 1.58  Ti=  48.9  Ms=1.82  GM=2.57
Tyreus-Luyben (from relay)   Kc= 1.10  Ti= 129.2  Ms=1.39  GM=4.04
SIMC (from the true model)   Kc= 1.33  Ti=  60.0  Ms=1.59  GM=3.14
```

**An autotune gets you into the right neighbourhood with no model at all**,
which is a real result and the reason the button exists. Tyreus–Luyben is what
most autotuners actually ship, and the Ms column shows why: it lands at 1.39,
comfortably inside process practice, where ZN's 1.82 does not.

The closed-loop cost of that safety, and the comparison against the
model-based rule, is [experiment 4](../articles/04-relay-autotune.md).

## Failure modes

`relay_autotune` raises rather than returning a bad number:

| error | cause | fix |
|---|---|---|
| *"produced only N cycles"* | run too short, or relay amplitude too small to overcome noise | raise `t_final` or `h` |
| *"hysteresis is as large as the oscillation it produced"* | deadband ≥ oscillation amplitude, so $\sqrt{a^2-\varepsilon^2}$ is imaginary | lower `hysteresis` or raise `h` |

`settle_cycles` (default 1) discards leading cycles so the estimate uses the
established limit cycle rather than the approach to it.

## The complete script

```python
--8<-- "docs/snippets/tutorial03.py"
```

## Next

[Tutorial 4: writing a plant](04-writing-a-plant.md) — putting your own
process into the harness.

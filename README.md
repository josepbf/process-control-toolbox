# Process Control Toolbox

A toolbox for **classical process control** — plants, controllers, tuning
rules, frequency-domain robustness analysis and a shared simulation harness —
aimed at the features that actually make industrial control hard: **long dead
time, loop interaction, recycle, and hard actuator limits**.

Everything is written to be read: explicit implementations in preference to
clever ones, with the control concept behind each one explained where it
appears. Every component is exercised by a reproducible experiment that
demonstrates what it does and what it costs.

Comparing control strategies is something the toolbox is *used for* — see
[FINDINGS.md](FINDINGS.md) — not what it is for.

**Status.** Five plants, eight controllers, thirteen PID tuning rules plus a
cited MPC rule, relay auto-tuning, frequency-domain robustness analysis, and
eleven reproducible experiments, with 234 tests. Dead-time compensation and
linear MPC have landed — the latter dependency-free, as one more `Controller`.
The classical structural repertoire (selector, ratio, split-range) is next.

## Documentation

Full documentation — tutorials, concepts, articles on every experiment, and a
generated API reference — is at
**[josepbf.github.io/process-control-toolbox](https://josepbf.github.io/process-control-toolbox/)**,
and as plain Markdown in [`docs/`](docs/).

| | |
|---|---|
| [Getting started](docs/getting-started.md) | install, run an experiment, read the output |
| [Tutorials](docs/tutorials/index.md) | build a loop, tune it, write your own plant or controller |
| [Concepts](docs/concepts/index.md) | architecture, fairness rules, dead time, metrics, robustness |
| [Articles](docs/articles/index.md) | one narrative write-up per experiment |
| [Glossary](docs/glossary.md) | the process-control vocabulary used here |

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

To use the toolbox outside the repository — a notebook, another project —
install it in editable mode as well:

```bash
.venv/bin/pip install -e . --config-settings editable_mode=compat
```

Then `from process_control import Tank, PIDController, simulate, simc_pi`
works from anywhere. (The `editable_mode=compat` setting writes a plain path
entry rather than setuptools' import-hook variant, which is not honoured in
every environment.) Running the bundled experiments and tests needs no
install at all.

## Reproduce the results

Run any experiment as a module; each writes its figures and metric CSVs to
`results/` (gitignored) and prints its table.

```bash
.venv/bin/python -m experiments.exp01_onoff_vs_pid
```

Run them all, then the test suite:

```bash
for f in experiments/exp*.py; do .venv/bin/python -m "experiments.$(basename "$f" .py)"; done
```

```bash
.venv/bin/python -m pytest -q
```

Findings are recorded in [FINDINGS.md](FINDINGS.md), and written up at
length in the [articles](docs/articles/index.md).

| script | headline result | article |
|---|---|---|
| `exp01_onoff_vs_pid.py` | ON/OFF limit-cycles at ±18 % with a period of ~4θ; SIMC PI settles without offset; SIMC vs Ziegler–Nichols is a tracking-vs-effort trade | [1](docs/articles/01-onoff-vs-pid.md) |
| `exp02_antiwindup.py` | integral windup on an infeasible setpoint: back-calculation is worth **16×** on recovery IAE, from a structural fix rather than tuning | [2](docs/articles/02-integral-windup.md) |
| `exp03_tuning_shootout.py` | ten published PI rules scored on performance **and** maximum sensitivity Ms; picks the fair PID baseline on stated grounds (SIMC at the knee, Ms 1.59) | [3](docs/articles/03-tuning-shootout.md) |
| `exp04_relay_autotune.py` | the industrial autotune: the relay experiment recovers the ultimate period to +7 % and under-gains by 24 % — and errs safe | [4](docs/articles/04-relay-autotune.md) |
| `exp05_deadtime_sweep.py` | PI degradation vs θ/τ against the physical floor; usable loop gain falls 80× while the peak deviation approaches a limit no controller can beat | [5](docs/articles/05-dead-time-sweep.md) |
| `exp06_averaging_level.py` | surge tank: tight level control and averaging level control rank **opposite** depending on which metric is the objective | [6](docs/articles/06-averaging-level.md) |
| `exp07_cascade.py` | cascade is **7.7×** better on the disturbance it was designed for, 0.9× (worse) on the one it was not, at 2.1× the valve travel | [7](docs/articles/07-cascade.md) |
| `exp08_feedforward.py` | dynamic feedforward from a measured disturbance: **5.4×** on IAE, 16× on peak — and what a 30 % gain error or an unrealisable delay costs | [8](docs/articles/08-feedforward.md) |
| `exp09_inverse_response.py` | a right-half-plane zero: more gain deepens the wrong-way dip 8.6×, and costs the same as dead time | [9](docs/articles/09-inverse-response.md) |

## Layout

```
process_control/
  plants/                 ground truth; owns all saturation, dead time and noise
    base.py                 Plant ABC: RK4, three independent delay paths
    tank.py                 SISO level, FOPDT, separate disturbance path
    integrating_tank.py     surge tank: integrating, no self-regulation
    series_tanks.py         N lags in series; the half rule's honest test
    inverse_response.py     drum level: shrink and swell, RHP zero
    cascade_process.py      fast inner stage feeding a slow outer stage
  controllers/            control laws, swappable inside the harness
    base.py                 Controller ABC
    onoff.py                two-position with hysteresis; time-proportioning (PWM)
    pid.py                  positional PID (anti-windup, filtered derivative)
                            and the velocity/incremental form
    cascade.py              primary/secondary PID pair
    feedforward.py          lead-lag feedforward from a measured disturbance
  harness/
    simulate.py             the single closed-loop runner used by everything
    scenarios.py            setpoint / disturbance programmes
    metrics.py              tracking, effort, constraint and computation metrics
    plotting.py             three-panel comparison, trade-off scatter, sweeps
  tuning/
    rules.py                SIMC, IMC, Lambda, Cohen-Coon, ZN (open and closed
                            loop), Tyreus-Luyben, AMIGO, integrating and
                            averaging-level rules, Skogestad's half rule
    analysis.py             Ms, gain and phase margin, analytic ultimate gain
    relay.py                Astrom-Hagglund relay auto-tuning
experiments/              one script per reported result
tests/                    pytest suite (234 tests)
results/                  generated figures and CSVs (gitignored)
docs/                     documentation site (MkDocs Material)
tools/                    documentation asset build
```

### The three delay paths

Dead time is the central difficulty of this problem domain, so the plant models
it in the three places it actually occurs, independently:

| path | attribute | what it is |
|---|---|---|
| manipulated | `dead_time` | transport lag between valve and process |
| disturbance | `d_dead_time` | when the upset reaches the output |
| measurement | `y_dead_time` | analyser or thermowell reporting lag |

Their *relationship* decides what is possible: feedforward is realisable only
when the disturbance path is slower than the manipulated one, and cascade pays
for itself largely because the secondary measurement is the undelayed one.

## Core interfaces

```python
class Plant(ABC):
    """Ground truth. The controller never sees inside it."""
    def step(self, u, dt, d=None) -> np.ndarray: ...   # saturate -> delay -> integrate -> measure
    def reset(self, x0=None, seed=None) -> np.ndarray: ...

class Controller(ABC):
    def compute(self, y, setpoint, t) -> np.ndarray: ...
    def reset(self) -> None: ...
```

Two rules follow from this split and are enforced throughout:

* **All saturation and all transport delay live in the plant**, so every
  controller faces identical physical limits. A control law cannot exempt
  itself from a constraint, only anticipate one.
* **`reset()` re-seeds the plant's noise generator**, so two controllers
  compared on the same scenario see a bit-identical noise realisation.
* **A controller only receives a measured disturbance if it declares
  `uses_measured_disturbance`**, so no controller can quietly benefit from
  information the others do not have.
* **Controllers publish their internal signals** through `diagnostics()` —
  a cascade's inner setpoint, a feedforward's contribution — logged under a
  `diag_` prefix and plottable.

[ARCHITECTURE.md](ARCHITECTURE.md) documents how the pieces fit and the
contract for adding a controller that needs more than the current measurement.

## Design principles

Four properties the toolbox is built to guarantee. They started as rules for
keeping a controller comparison honest, and they are more useful as library
guarantees:

1. **Every tuning is a named, cited rule.** SIMC, IMC, Lambda, Cohen-Coon,
   Ziegler-Nichols, Tyreus-Luyben, AMIGO — each returns the citation alongside
   the gains, and the harness records it with the run. No silent hand-tuning
   anywhere in the repository.
2. **The plant owns every physical limit.** Actuator saturation, all three
   dead-time paths and measurement noise live in the `Plant`, so no control law
   can exempt itself from a constraint.
3. **Runs are reproducible and comparable.** `reset()` re-seeds the plant's
   noise generator, so two controllers on the same scenario see a bit-identical
   noise realisation. Extra information — a measured disturbance — is only
   supplied to a controller that declares it needs it.
4. **Tracking is always reported with control effort.** Every metric table
   carries a tracking metric *and* an effort metric (`TV_u`, `reversals`,
   `max_du`), plus computation cost (`solve_ms_mean`, `solve_ms_p95`).
   Tracking bought with violent actuator movement is not a win — in a real
   plant it destroys valves.

## Roadmap

Organised by capability. Each item lands with the tuning method, the tests and
the experiment that demonstrate it.

| capability | content | status |
|---|---|---|
| harness and metrics | closed-loop runner, scenarios, tracking/effort/constraint/computation metrics, plotting | done |
| plants | FOPDT tank, integrating surge tank, series lags, inverse response, two-stage cascade | done |
| controllers | ON/OFF, time-proportioning, PID (positional and velocity), cascade, feedforward | done |
| dead-time compensation | Smith predictor, discrete internal model, Kalman observer | done |
| model predictive control | linear MPC as a condensed QP, soft output constraints, preview | done |
| tuning and analysis | thirteen named rules, half-rule reduction, Ms/GM/PM analysis, relay auto-tuning | done |
| **dead-time compensation** | Smith predictor and variants; brings the discrete internal-model and observer utilities with it | next |
| selector, ratio, split-range | the classical structural repertoire beyond cascade and feedforward | |
| MIMO and interaction | quadruple tank, RGA pairing, decentralised control | |
| model predictive control | *conditional* — added if it fits the `Controller` abstraction using the extension points in [ARCHITECTURE.md](ARCHITECTURE.md), not as a headline goal | |

System identification is deliberately out of scope for now; the package will
come back when there is something to put in it.

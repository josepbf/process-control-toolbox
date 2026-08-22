# MPC vs classical process control

Model Predictive Control implemented from scratch and benchmarked, fairly and
quantitatively, against the classical ladder — ON/OFF, PID, cascade, Smith
predictor, LQR — on processes with the features that actually make industrial
control hard: **long dead time, loop interaction, recycle, and hard actuator
limits**.

The goal is understanding and a defensible comparison, not a production
library. Implementations are explicit and readable in preference to clever.

**Status: phase 1 complete and extended.** Harness, five plants, six
controllers, thirteen tuning rules with frequency-domain robustness analysis,
relay auto-tuning, and nine reproducible experiments. Phase 2 (linear MPC by hand) not started.

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

## Reproduce the results

Run any experiment directly; each writes its figures and metric CSVs to
`results/` (gitignored) and prints its table.

```bash
.venv/bin/python experiments/exp01_onoff_vs_pid.py
```

Run them all, then the test suite:

```bash
for f in experiments/exp*.py; do .venv/bin/python "$f"; done
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
src/
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
tests/                    pytest suite (119 tests)
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
  controller faces identical physical limits. MPC's advantage has to come from
  *anticipating* a constraint, never from being exempt from one.
* **`reset()` re-seeds the plant's noise generator**, so two controllers
  compared on the same scenario see a bit-identical noise realisation.
* **A controller only receives a measured disturbance if it declares
  `uses_measured_disturbance`**, so no controller can quietly benefit from
  information the others do not have.

## Fairness rules

It is trivial to make MPC look brilliant by tuning the PID badly, and much of
the published literature does exactly that. This project enforces:

1. Every baseline is tuned by a **named, cited rule** (SIMC, IMC, ZN, and RGA
   pairing from phase 3), and the rule is recorded in the results table with
   the run. No silent hand-tuning.
2. Identical plant, sample time, noise seed, disturbance profile and actuator
   limits across every controller in a comparison.
3. MPC's internal model is never the exact plant, unless explicitly labelled as
   an idealised upper bound.
4. Scenarios where MPC does **not** win get reported. Fast SISO loops with no
   active constraint are expected to favour PID.
5. MPC's computational cost appears in the comparison table (`solve_ms_mean`,
   `solve_ms_p95` are logged for every controller, including the trivial ones).

Every reported comparison carries a tracking metric **and** a control-effort
metric. Tracking bought with violent actuator movement is not a win — in a real
plant it destroys valves.

## Roadmap

| phase | content | status |
|---|---|---|
| 1 | harness, FOPDT tank, ON/OFF, PID, SIMC | done |
| 1+ | integrating / high-order / inverse-response / cascade plants; cascade, feedforward, velocity-form and PWM controllers; thirteen tuning rules; Ms robustness analysis; relay auto-tuning | done |
| 2 | linear MPC built by hand (prediction matrices → QP in CVXPY); unconstrained MPC must converge to LQR, as a unit test | next |
| 3 | quadruple tank, RGA pairing, decentralised PID vs multivariable MPC, minimum and non-minimum phase | |
| 4 | system identification (PRBS, ARX/subspace) and deliberate model mismatch sweeps | |
| 5 | nonlinear: CSTR with NMPC, grinding circuit with recycle and long dead time, Smith predictor baseline | |
| 6 | optional: MHE, offset-free MPC, RL contender | |

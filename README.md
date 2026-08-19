# MPC vs classical process control

Model Predictive Control implemented from scratch and benchmarked, fairly and
quantitatively, against the classical ladder — ON/OFF, PID, cascade, Smith
predictor, LQR — on processes with the features that actually make industrial
control hard: **long dead time, loop interaction, recycle, and hard actuator
limits**.

The goal is understanding and a defensible comparison, not a production
library. Implementations are explicit and readable in preference to clever.

**Status: phase 1 complete** (harness, FOPDT tank, ON/OFF, PID, SIMC tuning).

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

## Reproduce the results

```bash
.venv/bin/python experiments/exp01_onoff_vs_pid.py
```

```bash
.venv/bin/python experiments/exp02_antiwindup.py
```

```bash
.venv/bin/python -m pytest -q
```

Each experiment writes its figure and its metric CSV to `results/`
(gitignored). Findings are recorded in [FINDINGS.md](FINDINGS.md).

| script | figure | what it shows |
|---|---|---|
| `exp01_onoff_vs_pid.py` | `results/exp01_onoff_vs_pid.png`, `results/exp01_pi_detail.png` | ON/OFF limit-cycles; SIMC PI settles without offset; SIMC vs Ziegler–Nichols is a tracking-vs-effort trade |
| `exp02_antiwindup.py` | `results/exp02_antiwindup.png` | integral windup on an infeasible setpoint, and what back-calculation is worth (16× recovery IAE) |

## Layout

```
src/
  plants/       ground-truth simulators; own all saturation, dead time and noise
    base.py       Plant ABC: RK4 integration, input delay line, measurement noise
    tank.py       SISO level, first order plus dead time
  controllers/  control laws, swappable inside the harness
    base.py       Controller ABC
    onoff.py      two-position with hysteresis
    pid.py        PID with back-calculation anti-windup and filtered derivative
  harness/
    simulate.py   the single closed-loop runner used by everything
    scenarios.py  setpoint / disturbance programmes
    metrics.py    tracking, effort, constraint and computation metrics
    plotting.py   the standard three-panel comparison figure
  tuning/
    rules.py      SIMC, IMC, Ziegler-Nichols — named and cited
experiments/    one script per reported result
tests/          pytest suite
results/        generated figures and CSVs (gitignored)
```

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
| 2 | linear MPC built by hand (prediction matrices → QP in CVXPY); unconstrained MPC must converge to LQR, as a unit test | next |
| 3 | quadruple tank, RGA pairing, decentralised PID vs multivariable MPC, minimum and non-minimum phase | |
| 4 | system identification (PRBS, ARX/subspace) and deliberate model mismatch sweeps | |
| 5 | nonlinear: CSTR with NMPC, grinding circuit with recycle and long dead time, Smith predictor baseline | |
| 6 | optional: MHE, offset-free MPC, RL contender | |

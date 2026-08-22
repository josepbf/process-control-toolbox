# Getting started

## Requirements

Python 3.11 or newer. Phase 1 depends on nothing but the scientific stack —
no solver, no control-systems library. That is deliberate: the MPC of phase 2
is built by hand, and cross-checks against reference implementations are added
only when there is something to cross-check.

```
numpy  scipy  matplotlib  pandas  pytest
```

## Install

```bash
git clone https://github.com/josepbf/process-control-toolbox.git
```

```bash
cd process-control-toolbox && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

## Run your first experiment

Every experiment is a standalone script. It prints its table, writes its
figures and metric CSVs to `results/`, and needs no arguments.

```bash
.venv/bin/python experiments/exp01_onoff_vs_pid.py
```

You should see something close to this (the numbers are deterministic — the
plant re-seeds its noise generator on every reset, so the run is bit-identical
across machines with the same NumPy version):

```text
                                   IAE      ITAE settling_2pct_s overshoot_pct  peak_dev ss_offset      TV_u reversals
controller       window
ON/OFF           full         13010.791 9028480.698           NaN       119.296    27.569    -4.443  4600.000        45
                 setpoint      5902.749 1746539.544           NaN        95.147    27.120    -4.626  1900.000        18
                 disturbance   6293.802 2240535.732           NaN       130.605    20.751    -4.488  2400.000        23
PI (SIMC)        full          1147.124  446325.935       845.000         6.010    20.161    -0.003   343.488        52
                 setpoint       715.414   33334.256        92.000         5.996    20.161     0.008   139.904        15
                 disturbance    410.898   52277.142       153.000           NaN     4.487    -0.000   157.498        34
```

and two files in `results/`:

```text
results/exp01_onoff_vs_pid.png   three-panel comparison figure
results/exp01_metrics.csv        full metric table (per controller, per window)
```

!!! tip "Reading the three-panel figure"
    Every comparison figure in the project has the same layout: controlled
    variable on top, **manipulated variable in the middle**, disturbance at
    the bottom. Panel 2 is not decoration — a tracking plot on its own
    systematically flatters aggressive controllers, and showing the valve
    underneath is what makes the cost visible at a glance.

## Run everything

```bash
for f in experiments/exp*.py; do .venv/bin/python "$f"; done
```

Then the test suite (119 tests, a few seconds):

```bash
.venv/bin/python -m pytest -q
```

## Use it as a library

The three objects you need are a `Plant`, a `Controller` and a `Scenario`.
`simulate()` puts them together and hands back a tidy DataFrame.

```python
from src.plants.tank import Tank
from src.controllers.pid import PIDController
from src.harness.scenarios import setpoint_and_load
from src.harness.simulate import simulate
from src.harness.metrics import compute_metrics
from src.tuning.rules import simc_pi

plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.15, seed=7)
scenario = setpoint_and_load(dt=1.0, y_start=30.0, y_step=50.0, seed=7)

tuning = simc_pi(**plant.fopdt)          # SIMC PI from the true model
pi = PIDController(
    **tuning.as_kwargs(), dt=scenario.dt,
    u_min=plant.u_min[0], u_max=plant.u_max[0],
    u0=plant.steady_input(30.0),
    name="PI (SIMC)", tuning_note=tuning.rule,
)

df = simulate(plant, pi, scenario)
print(compute_metrics(df, scenario.windows["disturbance"])["IAE"])
```

The full walkthrough of that snippet, line by line, is
[Tutorial 1: your first loop](tutorials/01-your-first-loop.md).

## Build the documentation locally

The docs are a MkDocs Material site. They live in `docs/` and are plain
Markdown, so they are readable straight from the repository too.

```bash
python3 -m venv .venv-docs && .venv-docs/bin/pip install -r requirements-docs.txt
```

```bash
.venv-docs/bin/mkdocs serve
```

The figures the articles refer to are committed under
`docs/assets/figures/`, so the site builds without running a simulation. To
regenerate them after changing an experiment:

```bash
.venv/bin/python tools/build_docs_assets.py
```

## Repository layout

```text
src/
  plants/                 ground truth; owns all saturation, dead time and noise
  controllers/            control laws, swappable inside the harness
  harness/                the single closed-loop runner, scenarios, metrics, plots
  tuning/                 tuning rules, robustness analysis, relay auto-tuning
experiments/              one script per reported result
tests/                    pytest suite
results/                  generated figures and CSVs (gitignored)
docs/                     this documentation
tools/                    documentation asset build
```

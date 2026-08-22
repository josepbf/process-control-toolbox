# Tutorial 6: designing an experiment

The nine scripts in `experiments/` all follow the same shape. This page is the
convention, so a tenth one looks like the other nine and so anyone reading the
results knows what they are looking at.

## The anatomy of an experiment script

```python
"""Phase N: the one-sentence claim this script establishes.

Reproduce with:

    python experiments/expNN_name.py

Why this experiment exists, what is held fixed, what is varied, and what to
look for in the output. Long. This docstring is the article.

Outputs
-------
results/expNN_figure.png
results/expNN_metrics.csv
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

# ... imports from src ...

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def build():
    """Plant, scenario and controllers. Separated so tests can import it."""


def main():
    """Run, score, print, save. Returns the table."""


if __name__ == "__main__":
    main()
```

Four conventions worth stating explicitly:

**The docstring carries the argument.** Someone reading the script should not
have to run it to know what it claims and what would falsify the claim. Every
experiment in the repository has a docstring longer than its code.

**`build()` is separate from `main()`** where the setup is worth reusing
(experiments 1, 2, 7 and 8 do this). It lets a test import the constructed
objects and assert on them without running a 1400-sample simulation.

**Everything goes to `results/`**, which is gitignored. Figures for the
documentation are copied into `docs/assets/figures/` by
`tools/build_docs_assets.py`.

**`main()` returns the table**, so an experiment can be composed into another
script or a notebook.

## Pattern 1: a controller comparison

Several control laws, one plant, one scenario, one seed.

```python
controllers = {
    "single-loop PI": single_pi,
    "cascade PI/PI": cascade,
}
runs = run_all(plant, controllers, scenario)
table = summarize(runs, windows=scenario.windows)
print(format_table(table))

save_table(table, RESULTS / "expNN_metrics.csv")
plot_runs(runs, title="...", scenario=scenario, path=RESULTS / "expNN.png")
```

The one thing to get right is the **windows**. Declare them on the `Scenario`,
name them after the physical event, and let each window answer one question:

```python
windows={
    "setpoint":    (200.0, 800.0),
    "inner upset": (800.0, 1400.0),
    "outer upset": (1400.0, 2000.0),
}
```

[Experiment 7](../articles/07-cascade.md) is the model here — the three
windows are what turn "cascade is better" into "cascade is 7.7× better on one
disturbance and 0.9× on another".

## Pattern 2: a parameter sweep

One controller, one scenario shape, one parameter varied over a range. The
*shape of the degradation curve* is the result; a single operating point is
not.

```python
rows, runs = [], {}
for ratio in RATIOS:
    df, row = run_one(ratio)
    rows.append(row)
    if ratio in HIGHLIGHT:
        runs[f"theta/tau = {ratio:g}"] = df       # keep a few traces to plot
table = pd.DataFrame(rows).set_index("theta_over_tau")

plot_sweep(
    table.reset_index(),
    x="theta_over_tau",
    panels=[
        {"series": ["peak_dev", "peak_floor"], "ylabel": "load peak deviation [%]"},
        {"series": ["peak_ratio"], "ylabel": "peak / physical floor", "hlines": [1.0]},
        {"series": ["Kc_K"], "ylabel": "loop gain Kc*K"},
    ],
    title="...", x_label="theta / tau", logx=True,
    path=RESULTS / "expNN_sweep.png",
)
```

!!! tip "Scale the scenario with the process"
    In [experiment 5](../articles/05-dead-time-sweep.md) every time in the
    scenario is a multiple of `tau + theta`:

    ```python
    scale = TAU + theta
    scenario = setpoint_and_load(
        t_step=2 * scale, t_load=12 * scale, t_final=30 * scale, ...
    )
    ```

    Otherwise a slow case is scored on a run that ended before it settled, and
    the "degradation" you measure is partly an artefact of the run length. For
    the same reason, IAE is divided by `scale` before cases are compared.

!!! tip "Vary one thing"
    Experiment 9 sweeps a right-half-plane zero while **holding the
    steady-state gain fixed**, by construction:

    ```python
    def make_plant(K_fast, **kw):
        return InverseResponseTank(K_slow=K_NET - K_fast, K_fast=K_fast, ...)
    ```

    Without that, the sweep would confound the zero with the gain, and the
    resulting curve would mean nothing.

## Pattern 3: a trade-off frontier

Two metrics, one point per design. This is how you show that there is no
single best answer — only a frontier, and a choice about where to sit on it.

```python
plot_tradeoff(
    table, x="Ms", y="IAE_load",
    title="load rejection vs robustness",
    x_label="maximum sensitivity Ms",
    y_label="IAE, load disturbance window",
    good_x=(1.2, 1.6),          # shades the comfortable band
    path=RESULTS / "expNN_frontier.png",
)
```

[Experiment 3](../articles/03-tuning-shootout.md) is the canonical use: ten
tuning rules, performance on one axis and robustness on the other, and the
baseline for the entire project chosen from where the knee falls.

## Adding a fundamental limit to the plot

The most useful thing an experiment can do is show how much room is left. Any
time there is a bound no controller can beat, compute it and draw it.

```python
# Best any controller could do: the deviation accumulated during the dead
# time, before its first move can possibly arrive.
floor = abs(plant.Kd * D_LOAD) * (1.0 - np.exp(-theta / TAU))
```

The gap between the achieved curve and that line is the headroom a better
controller could in principle recover. Where they meet, the loss is physics.
This is what turns "MPC handles dead time better" from a slogan into a
falsifiable claim — and it is why experiment 5 exists *before* phase 2 rather
than after it.

## Reporting

Write the result into
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md)
with:

- the exact plant parameters, scenario and seed;
- the table, with a tracking column **and** an effort column;
- the interpretation, including the cases where the new thing loses;
- what it implies for the phases that have not happened yet.

Then add an [article](../articles/index.md) if it deserves the narrative
treatment.

!!! quote "The standard the project holds itself to"
    From experiment 7, on a result that could have been reported as a clean
    7.7× win:

    > The negative result is as important as the positive one: the outer-stage
    > upset never passes through the inner loop, so the extra measurement
    > carries no information about it, and the cascade is marginally *worse*
    > there. That is the correct shape for a structural change — it buys a
    > specific thing, and reporting only the case where it wins would be the
    > same sleight of hand this project is trying to avoid for MPC.

## Testing an experiment

The current suite tests the library rather than the experiment scripts, which
is the right default — experiment numbers legitimately move when a metric is
improved, and pinning them makes the suite fight the work.

Where an experiment's *premise* is worth pinning, import `build()` and assert
on the constructed objects rather than on the simulation:

```python
def test_exp02_valve_limit_makes_the_setpoint_infeasible():
    plant, scenario, controllers, tuning = build()
    assert plant.u_max[0] * plant.K < 60.0     # the setpoint really is unreachable
    assert set(controllers) == {"PI, no anti-windup", "PI + back-calculation"}
```

That keeps the suite fast and pins the *claim* the experiment is making. If
the valve limit were ever loosened, experiment 2 would quietly stop being
about windup, and this is the test that would say so.

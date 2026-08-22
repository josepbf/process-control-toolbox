"""Phase 1 supporting result: integral windup, and what back-calculation does.

Reproduce with:

    python experiments/exp02_antiwindup.py

Setup: the same tank, but the inlet valve is mechanically limited to 30 %%,
which holds at most 45 %% level. The setpoint is then driven to 60 %% -- an
*infeasible* target -- held there, and dropped back to 35 %%.

While the setpoint is unreachable the loop is effectively open: the error never
goes away and the valve cannot answer it. A naive integrator keeps accumulating
through that whole period, so when the setpoint finally becomes reachable the
controller must first unwind a large bogus integral before it starts closing
the valve -- and the level sails past the new setpoint.

Both controllers here are the *same* SIMC tuning. The only difference is
whether the integrator is told about the clipping (back-calculation with
tracking constant Tt) or not (Tt = infinity). Anti-windup is not tuning; it is
a structural fix, and this is what it is worth.
"""

from __future__ import annotations

import pathlib

import numpy as np


from process_control.controllers.pid import PIDController
from process_control.harness.metrics import format_table, summarize
from process_control.harness.plotting import plot_runs, save_table
from process_control.harness.scenarios import Scenario, constant, staircase
from process_control.harness.simulate import run_all
from process_control.plants.tank import Tank
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def build():
    plant = Tank(
        K=1.5, tau=60.0, theta=15.0, h0=30.0,
        u_min=0.0, u_max=30.0,          # valve mechanically limited: y_max_feasible = 45 %
        noise_std=0.15, seed=7,
    )
    scenario = Scenario(
        name="infeasible_setpoint",
        dt=1.0,
        t_final=2400.0,
        setpoint=staircase([(0.0, 30.0), (100.0, 60.0), (600.0, 35.0)]),
        disturbance=constant(0.0),
        seed=7,
        description="setpoint 30 -> 60 %% (infeasible, valve limited to 30 %%) -> 35 %%",
        windows={"saturated": (100.0, 600.0), "recovery": (600.0, 2400.0)},
    )

    tuning = simc_pi(**plant.fopdt)
    common = dict(
        **tuning.as_kwargs(), dt=scenario.dt,
        u_min=plant.u_min[0], u_max=plant.u_max[0], u0=plant.steady_input(30.0),
    )
    controllers = {
        "PI, no anti-windup": PIDController(
            **common, Tt=np.inf, name="PI, no anti-windup",
            tuning_note=tuning.rule + "; integrator blind to saturation",
        ),
        "PI + back-calculation": PIDController(
            **common, name="PI + back-calculation",
            tuning_note=tuning.rule + "; anti-windup Tt = Ti (Astrom & Hagglund)",
        ),
    }
    return plant, scenario, controllers, tuning


def main():
    plant, scenario, controllers, tuning = build()
    print(f"scenario: {scenario.description}")
    print(f"tuning  : {tuning.rule:<45} Kc={tuning.Kc:6.3f}  Ti={tuning.Ti:7.2f}")
    print(f"          identical in both runs; only the anti-windup path differs\n")

    runs = run_all(plant, controllers, scenario)
    table = summarize(runs, windows=scenario.windows)
    print(format_table(table))
    print()

    save_table(table, RESULTS / "exp02_metrics.csv")
    plot_runs(
        runs,
        title="Phase 1 -- integral windup on an infeasible setpoint (same PI tuning)",
        scenario=scenario,
        path=RESULTS / "exp02_antiwindup.png",
        figsize=(11.0, 7.5),
    )
    print(f"wrote {RESULTS / 'exp02_antiwindup.png'}")
    print(f"wrote {RESULTS / 'exp02_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

"""Phase 1: the surge tank, where tight control is the wrong objective.

Reproduce with:

    python experiments/exp06_averaging_level.py

An integrating level process (a surge tank between two units), with the tank
*outflow* as the manipulated variable and the inflow as the disturbance. Four
controllers, three of them tuned by named rules for this class of process and
one deliberately wrong.

The point of the experiment is that the metric decides the winner:

* Judged on level deviation, tight SIMC tuning wins easily.
* Judged on **outflow variability** -- what the downstream unit actually
  experiences -- it is the worst of the four, because every inflow wobble is
  passed straight through the tank instead of being absorbed by it.

A surge tank is installed to buy the downstream unit a steady feed. A level
controller that holds level perfectly has, quite literally, removed the reason
the tank was built. This is the clearest phase-1 example of a rule the whole
project runs on: a control comparison is meaningless until the objective is
stated, and "tracking error" is not automatically the objective.

The fourth controller shows the classic field mistake: applying a
self-regulating (FOPDT) tuning rule to an integrating process, which sets the
integral time from a lag that does not exist and produces the slow rolling
oscillation seen in level loops in every plant.

Outputs
-------
results/exp06_averaging_level.png
results/exp06_metrics.csv
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pandas as pd

from src.controllers.pid import PIDController
from src.harness.metrics import compute_metrics
from src.harness.plotting import plot_runs, save_table
from src.harness.scenarios import Scenario, constant, staircase
from src.harness.simulate import run_all
from src.plants.integrating_tank import IntegratingTank
from src.tuning.analysis import pid_on_integrator
from src.tuning.rules import averaging_level_pi, lambda_tuning, simc_integrating

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

ALARM_LOW, ALARM_HIGH = 25.0, 75.0


def main() -> pd.DataFrame:
    # u is the outflow valve: opening it lowers the level, so k' is negative and
    # every tuning below comes out reverse acting.
    plant = IntegratingTank(
        k_prime=-0.02, u_bias=50.0, theta=10.0, Kd=0.02, h0=50.0,
        noise_std=0.2, y_min=ALARM_LOW, y_max=ALARM_HIGH, seed=7,
    )
    scenario = Scenario(
        name="inflow_upsets",
        dt=1.0,
        t_final=3600.0,
        setpoint=constant(50.0),
        disturbance=staircase(
            [(0.0, 0.0), (300.0, 15.0), (1200.0, -10.0), (2100.0, 8.0), (3000.0, 0.0)]
        ),
        seed=7,
        description="inflow steps +15, -10, +8, 0 into a surge tank; level setpoint 50 %",
        windows={"upset 1": (300.0, 1200.0), "upset 2": (1200.0, 2100.0)},
    )

    p = plant.integrating
    tight = simc_integrating(**p, tau_c=p["theta"])
    loose = simc_integrating(**p, tau_c=5 * p["theta"])
    averaging = averaging_level_pi(k_prime=p["k_prime"], v_max=15.0, y_max_dev=20.0)
    # The mistake: pretend the integrator is a very slow self-regulating lag.
    tau_fake = 1000.0
    wrong = lambda_tuning(K=p["k_prime"] * tau_fake, tau=tau_fake, theta=p["theta"], lam=100.0)

    rules = {
        "SIMC tight (tau_c=theta)": tight,
        "SIMC loose (tau_c=5*theta)": loose,
        "averaging P-only": averaging,
        "lambda, integrator ignored": wrong,
    }
    controllers = {
        label: PIDController(
            **t.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=plant.u_bias, name=label, tuning_note=t.rule,
        )
        for label, t in rules.items()
    }
    runs = run_all(plant, controllers, scenario)

    rows = []
    for label, t in rules.items():
        m = compute_metrics(runs[label], (300.0, 3600.0))
        rob = pid_on_integrator(k_prime=p["k_prime"], theta=p["theta"], Kc=t.Kc, Ti=t.Ti)
        rows.append({
            "controller": label, "Kc": t.Kc, "Ti": t.Ti, "Ms": rob["Ms"],
            "peak_level_dev": m["peak_dev"], "IAE_level": m["IAE"],
            "TV_outflow": m["TV_u"], "max_flow_move": m["max_du"],
            "alarm_violations": m["n_violations"], "citation": t.rule,
        })
    table = pd.DataFrame(rows).set_index("controller")

    cols = ["Kc", "Ti", "Ms", "peak_level_dev", "IAE_level", "TV_outflow",
            "max_flow_move", "alarm_violations"]
    print(f"surge tank: k' = {p['k_prime']} %/s per % valve, theta = {p['theta']} s")
    print(f"alarm band {ALARM_LOW:g}-{ALARM_HIGH:g} %; {scenario.description}\n")
    print(table[cols].to_string(float_format=lambda v: f"{v:9.3f}"))
    print()
    print("peak_level_dev  what a level-tracking metric rewards")
    print("TV_outflow      what the downstream unit actually feels")
    print("The two rankings are opposite. That is the finding.\n")

    save_table(table, RESULTS / "exp06_metrics.csv")
    plot_runs(
        runs,
        title="Phase 1 -- surge tank: tight level control vs averaging level control",
        scenario=scenario,
        y_label="level y [%]",
        u_label="outflow u [%]",
        path=RESULTS / "exp06_averaging_level.png",
        figsize=(11.5, 8.0),
    )
    print(f"wrote {RESULTS / 'exp06_averaging_level.png'}")
    print(f"wrote {RESULTS / 'exp06_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

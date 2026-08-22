"""Phase 1 headline: ON/OFF vs PID on a FOPDT tank.

Reproduce with:

    python experiments/exp01_onoff_vs_pid.py

Outputs
-------
results/exp01_onoff_vs_pid.png   three-panel comparison figure
results/exp01_metrics.csv        full metric table (per controller, per window)

What to look for
----------------
* ON/OFF cannot settle. It limit-cycles forever, at an amplitude set by the
  deadband *and the dead time* -- the valve keeps driving for theta seconds
  after the level has already crossed the band. Its reversal count is the
  giveaway.
* SIMC PI settles with no steady-state offset (integral action) and rejects the
  load step without any help from feedforward.
* Ziegler-Nichols PI is included as the aggressive reference: better peak
  deviation on the load upset, paid for with overshoot and far more valve
  travel. This is the tracking-vs-effort trade the project insists on showing.

Note on fairness: both PI tunings are computed from the *true* FOPDT parameters
of the plant, which is the most generous possible setting for the baselines.
Model mismatch is phase 4's problem, and it is introduced there deliberately.
"""

from __future__ import annotations

import pathlib


import pandas as pd

from process_control.controllers.onoff import OnOffController
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import format_table, summarize
from process_control.harness.plotting import plot_runs, save_table
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import run_all
from process_control.plants.tank import Tank
from process_control.tuning.rules import simc_pi, ziegler_nichols_open_loop

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def build():
    plant = Tank(
        K=1.5,
        tau=60.0,
        theta=15.0,
        Kd=1.0,
        h0=30.0,
        u_min=0.0,
        u_max=100.0,
        noise_std=0.15,
        seed=7,
    )
    scenario = setpoint_and_load(dt=1.0, seed=7)

    p = plant.fopdt
    simc = simc_pi(**p)                                   # tau_c = theta (default)
    zn = ziegler_nichols_open_loop(**p, kind="PI")
    u0 = plant.steady_input(30.0)                         # bumpless start at 30 %

    controllers = {
        "ON/OFF": OnOffController(u_on=100.0, u_off=0.0, hysteresis=1.0),
        "PI (SIMC)": PIDController(
            **simc.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=u0, name="PI (SIMC)", tuning_note=simc.rule,
        ),
        "PI (ZN)": PIDController(
            **zn.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=u0, name="PI (ZN)", tuning_note=zn.rule,
        ),
    }
    return plant, scenario, controllers, {"SIMC": simc, "ZN": zn}


def main() -> pd.DataFrame:
    plant, scenario, controllers, tunings = build()

    print(f"plant   : {plant!r}")
    print(f"          K={plant.K}, tau={plant.tau}, theta={plant.theta}, "
          f"theta/tau={plant.theta / plant.tau:.2f}, noise_std={plant.noise_std[0]}")
    print(f"scenario: {scenario.description}")
    for key, tuning in tunings.items():
        print(f"tuning  : {tuning.rule:<45} Kc={tuning.Kc:6.3f}  Ti={tuning.Ti:7.2f}")
    print()

    runs = run_all(plant, controllers, scenario)
    table = summarize(runs, windows=scenario.windows)

    print(format_table(table))
    print()

    save_table(table, RESULTS / "exp01_metrics.csv")
    plot_runs(
        runs,
        title="Phase 1 -- ON/OFF vs PID on a FOPDT tank (setpoint step + load disturbance)",
        scenario=scenario,
        path=RESULTS / "exp01_onoff_vs_pid.png",
    )
    # The ON/OFF limit cycle spans the whole axis, so the PI traces are
    # unreadable on the combined plot. Second figure: the PI tunings alone.
    plot_runs(
        {k: v for k, v in runs.items() if k != "ON/OFF"},
        title="Phase 1 -- SIMC vs Ziegler-Nichols PI (detail)",
        scenario=scenario,
        path=RESULTS / "exp01_pi_detail.png",
        figsize=(11.0, 7.0),
    )
    print(f"wrote {RESULTS / 'exp01_onoff_vs_pid.png'}")
    print(f"wrote {RESULTS / 'exp01_pi_detail.png'}")
    print(f"wrote {RESULTS / 'exp01_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

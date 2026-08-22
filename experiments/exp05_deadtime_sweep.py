"""Phase 1: how far does PID degrade as dead time takes over?

Reproduce with:

    python experiments/exp05_deadtime_sweep.py

This is the setup experiment for everything the project is really about. The
ratio theta/tau -- dead time over lag -- is the single best predictor of how
hard a loop is. Below ~0.2 almost any tuning works. Above ~1 the loop is
dead-time-dominant: the controller is acting on information that is already
stale, gain has to be cut to stay stable, and feedback alone runs out of road.
That is exactly the regime a cement mill or a kiln lives in, and exactly where
dead-time compensation (Smith predictor, phase 5) and prediction (MPC, phase 2)
are supposed to earn their keep.

The sweep holds everything else fixed -- same plant gain, same lag, same
disturbance, same tuning rule (SIMC with tau_c = theta, tuned on the true
model), same seed -- and varies only theta.

A fundamental limit is plotted alongside. After a load step, *no* controller
can affect the output for theta seconds, because its move takes that long to
arrive. The unavoidable peak deviation is therefore at least

    |Kd * d| * (1 - exp(-theta/tau))

which is the dashed line. The gap between the PI curve and that line is what
better control could in principle recover; where the two meet, the loss is
physics and no controller -- MPC included -- can buy it back.

Outputs
-------
results/exp05_deadtime_sweep.png
results/exp05_deadtime_metrics.csv
"""

from __future__ import annotations

import pathlib


import numpy as np
import pandas as pd

from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.plotting import plot_runs, plot_sweep, save_table
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import simulate
from process_control.plants.tank import Tank
from process_control.tuning.analysis import pid_on_fopdt
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

TAU = 60.0
K = 1.5
D_LOAD = -12.0
RATIOS = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 4.0]
HIGHLIGHT = [0.1, 1.0, 4.0]


def run_one(ratio: float):
    theta = ratio * TAU
    plant = Tank(K=K, tau=TAU, theta=theta, Kd=1.0, h0=30.0, noise_std=0.15, seed=7)

    # Every time in the scenario scales with the process, so each case gets the
    # same number of process time constants to respond in.
    scale = TAU + theta
    scenario = setpoint_and_load(
        dt=1.0, y_start=30.0, y_step=50.0, t_step=2 * scale, d_load=D_LOAD,
        t_load=12 * scale, t_final=30 * scale, seed=7,
    )

    tuning = simc_pi(**plant.fopdt)
    ctrl = PIDController(
        **tuning.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=plant.steady_input(30.0), name=f"theta/tau={ratio:g}", tuning_note=tuning.rule,
    )
    df = simulate(plant, ctrl, scenario)

    sp = compute_metrics(df, scenario.windows["setpoint"])
    ld = compute_metrics(df, scenario.windows["disturbance"])
    rob = pid_on_fopdt(**plant.fopdt, Kc=tuning.Kc, Ti=tuning.Ti)

    # Best any controller could do: the deviation accumulated during the dead
    # time, before its first move can possibly arrive.
    floor = abs(plant.Kd * D_LOAD) * (1.0 - np.exp(-theta / TAU))

    return df, {
        "theta_over_tau": ratio,
        "theta_s": theta,
        "Kc_K": tuning.Kc * K,                       # dimensionless loop gain
        "Ti_over_tau": tuning.Ti / TAU,
        "Ms": rob["Ms"],
        "peak_dev": ld["peak_dev"],
        "peak_floor": floor,
        "peak_ratio": ld["peak_dev"] / max(floor, 1e-9),
        "IAE_load_scaled": ld["IAE"] / scale,
        "settling_scaled": ld["settling_2pct_s"] / scale,
        "overshoot_sp_pct": sp["overshoot_pct"],
        "TV_u": sp["TV_u"] + ld["TV_u"],
    }


def main() -> pd.DataFrame:
    print(f"tau = {TAU:g} s fixed, theta swept; SIMC PI (tau_c = theta) on the true model")
    print(f"load disturbance d = {D_LOAD:g} in every case\n")

    rows, runs = [], {}
    for ratio in RATIOS:
        df, row = run_one(ratio)
        rows.append(row)
        if ratio in HIGHLIGHT:
            runs[f"theta/tau = {ratio:g}"] = df
    table = pd.DataFrame(rows).set_index("theta_over_tau")

    print(table.to_string(float_format=lambda v: f"{v:9.3f}"))
    print()
    print("Kc_K            dimensionless loop gain the tuning rule dares to use")
    print("peak_ratio      achieved peak deviation / the physical floor set by dead time")
    print("IAE_load_scaled IAE divided by (tau+theta), so cases are comparable")
    print()

    save_table(table, RESULTS / "exp05_deadtime_metrics.csv")
    plot_sweep(
        table.reset_index(),
        x="theta_over_tau",
        panels=[
            {"series": ["peak_dev", "peak_floor"],
             "ylabel": "load peak deviation [%]"},
            {"series": ["peak_ratio"],
             "ylabel": "peak / physical floor", "hlines": [1.0]},
            {"series": ["IAE_load_scaled"], "ylabel": "IAE / (tau+theta)"},
            {"series": ["Kc_K"], "ylabel": "loop gain Kc*K"},
        ],
        title="Phase 1 -- PI performance vs dead-time dominance (SIMC, true model)",
        x_label="theta / tau",
        logx=True,
        path=RESULTS / "exp05_deadtime_sweep.png",
        figsize=(9.5, 11.0),
    )
    plot_runs(
        runs,
        title="Phase 1 -- the same loop and the same tuning rule at three dead-time ratios",
        path=RESULTS / "exp05_trajectories.png",
        figsize=(11.0, 7.5),
    )
    print(f"wrote {RESULTS / 'exp05_deadtime_sweep.png'}")
    print(f"wrote {RESULTS / 'exp05_trajectories.png'}")
    print(f"wrote {RESULTS / 'exp05_deadtime_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

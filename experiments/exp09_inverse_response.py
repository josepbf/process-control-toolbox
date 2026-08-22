"""Phase 1: inverse response -- when the measurement lies to you first.

Reproduce with:

    python experiments/exp09_inverse_response.py

Boiler drum level, and the same mathematics in any process with two opposing
paths: opening feedwater must eventually raise the level, but the cold water
collapses steam bubbles below the surface and the level *falls first*. The
transfer function has a right-half-plane zero.

Two results here.

**1. Pushing harder digs the hole deeper.** Four SIMC tunings of increasing
aggressiveness on the same plant. The signature of the RHP zero is the
*wrong-way dip*: on a step up in setpoint the level first falls, and the harder
the controller pushes to correct the error it sees, the further it falls --
because its own action is what produces the dip. On a minimum-phase plant a
higher gain always *reduces* the initial deviation; here it monotonically
increases it. That is the ceiling on achievable bandwidth that no tuning can
lift, and it is why more gain is not the answer.

(The rest of the picture stays ordinary, and is reported rather than hidden:
higher gain still improves load-disturbance IAE, at the usual price in
robustness and valve travel. The setpoint IAE is U-shaped with a minimum at the
SIMC default.)

**2. The cost scales with the zero.** The severity of the inverse response is
swept while holding the steady-state gain fixed, so the *only* thing changing
is the zero. Performance degrades the same way it degrades with dead time --
which is the point: an RHP zero and a transport delay cost you the same thing,
the ability to act on information you do not yet have.

This is the phase-3 non-minimum-phase story on a SISO loop, where it can be
seen on its own before loop interaction is added on top.

Outputs
-------
results/exp09_inverse_response.png, results/exp09_zero_sweep.png
results/exp09_metrics.csv, results/exp09_sweep.csv
"""

from __future__ import annotations

import pathlib


import numpy as np
import pandas as pd

from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.plotting import plot_runs, plot_sweep, save_table
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import run_all, simulate
from process_control.plants.inverse_response import InverseResponseTank
from process_control.tuning.analysis import pid_on_fopdt
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

TAU_SLOW, TAU_FAST, K_NET = 60.0, 5.0, 1.0


def make_plant(K_fast: float, **kw) -> InverseResponseTank:
    """Vary the inverse response while holding the steady-state gain at K_NET,
    so the sweep isolates the zero rather than confounding it with gain."""
    return InverseResponseTank(
        K_slow=K_NET - K_fast, tau_slow=TAU_SLOW,
        K_fast=K_fast, tau_fast=TAU_FAST, theta=2.0, **kw
    )


def scenario_for(plant):
    return setpoint_and_load(
        dt=1.0, y_start=30.0, y_step=50.0, t_step=200.0,
        d_load=-10.0, t_load=1200.0, t_final=2600.0, seed=7,
    )


def part1_tuning_aggressiveness() -> pd.DataFrame:
    plant = make_plant(K_fast=-0.5, noise_std=0.15, seed=7)
    model = plant.fopdt_half_rule()
    scenario = scenario_for(plant)

    print("part 1 -- four tunings, same plant")
    print(f"  true plant   : K = {plant.K:.2f}, RHP zero T_z = {plant.zero:.1f} s, "
          f"inverse response = {plant.has_inverse_response}")
    print(f"  half-rule model given to the tuning rule: "
          f"{ {k: round(v, 1) for k, v in model.items()} }")
    print(f"  (the zero has been folded into an effective dead time of "
          f"{model['theta']:.0f} s)\n")

    factors = {"aggressive (tau_c=0.3*theta)": 0.3, "SIMC default (tau_c=theta)": 1.0,
               "conservative (tau_c=2*theta)": 2.0, "very slow (tau_c=4*theta)": 4.0}
    rules = {
        label: simc_pi(**model, tau_c=f * model["theta"]) for label, f in factors.items()
    }
    controllers = {
        label: PIDController(
            **t.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=plant.steady_input(30.0), name=label, tuning_note=t.rule,
        )
        for label, t in rules.items()
    }
    runs = run_all(plant, controllers, scenario)

    rows = []
    for label, t in rules.items():
        df = runs[label]
        sp = compute_metrics(df, scenario.windows["setpoint"])
        ld = compute_metrics(df, scenario.windows["disturbance"])
        rob = pid_on_fopdt(**model, Kc=t.Kc, Ti=t.Ti)

        # The wrong-way dip: how far *below* the starting level the process goes
        # in the first three minutes after a setpoint step upward.
        t0 = scenario.windows["setpoint"][0]
        early = df[(df["t"] >= t0) & (df["t"] <= t0 + 180.0)]
        dip = float(30.0 - early["y"].min())

        rows.append({
            "tuning": label, "Kc": t.Kc, "Ti": t.Ti, "Ms_reduced_model": rob["Ms"],
            "wrong_way_dip": dip,
            "IAE_sp": sp["IAE"], "overshoot_pct": sp["overshoot_pct"],
            "IAE_load": ld["IAE"], "peak_dev_load": ld["peak_dev"],
            "TV_u": sp["TV_u"] + ld["TV_u"],
        })
    table = pd.DataFrame(rows).set_index("tuning")
    print(table.to_string(float_format=lambda v: f"{v:9.3f}"))
    print("\nwrong_way_dip: how far the level falls BELOW its starting value after a")
    print("step UP in setpoint. It grows monotonically with gain -- the controller")
    print("is digging its own hole. No minimum-phase plant behaves like this.")

    plot_runs(
        runs, scenario=scenario,
        title="Phase 1 -- inverse response: more gain makes it worse, not better",
        path=RESULTS / "exp09_inverse_response.png", figsize=(11.5, 8.0),
    )
    save_table(table, RESULTS / "exp09_metrics.csv")
    return table


def part2_zero_sweep() -> pd.DataFrame:
    print("\npart 2 -- sweeping the zero at constant steady-state gain")
    rows = []
    for K_fast in [0.0, -0.1, -0.2, -0.4, -0.6, -0.8]:
        plant = make_plant(K_fast=K_fast, noise_std=0.15, seed=7)
        model = plant.fopdt_half_rule()
        scenario = scenario_for(plant)
        t = simc_pi(**model)
        ctrl = PIDController(
            **t.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=plant.steady_input(30.0), name=f"K_fast={K_fast}", tuning_note=t.rule,
        )
        df = simulate(plant, ctrl, scenario)
        sp = compute_metrics(df, scenario.windows["setpoint"])
        ld = compute_metrics(df, scenario.windows["disturbance"])
        rows.append({
            "K_fast": K_fast,
            "T_z_s": plant.zero,
            "T_z_over_tau": plant.zero / TAU_SLOW,
            "inverse": plant.has_inverse_response,
            "theta_eff_s": model["theta"],
            "Kc": t.Kc,
            "IAE_sp": sp["IAE"],
            "settling_sp_s": sp["settling_2pct_s"],
            "IAE_load": ld["IAE"],
            "peak_dev_load": ld["peak_dev"],
        })
    table = pd.DataFrame(rows).set_index("K_fast")
    print(table.to_string(float_format=lambda v: f"{v:9.3f}"))

    plot_sweep(
        table.reset_index(),
        x="T_z_over_tau",
        panels=[
            {"series": ["IAE_sp", "IAE_load"], "ylabel": "IAE"},
            {"series": ["peak_dev_load"], "ylabel": "load peak deviation [%]"},
            {"series": ["Kc"], "ylabel": "gain the rule dares to use"},
        ],
        title="Phase 1 -- cost of a right-half-plane zero (steady-state gain held fixed)",
        x_label="T_z / tau   (negative = ordinary zero, positive = inverse response)",
        path=RESULTS / "exp09_zero_sweep.png",
        figsize=(9.5, 8.5),
    )
    save_table(table, RESULTS / "exp09_sweep.csv")
    return table


def main():
    t1 = part1_tuning_aggressiveness()
    t2 = part2_zero_sweep()
    print(f"\nwrote {RESULTS / 'exp09_inverse_response.png'}")
    print(f"wrote {RESULTS / 'exp09_zero_sweep.png'}")
    return t1, t2


if __name__ == "__main__":
    main()

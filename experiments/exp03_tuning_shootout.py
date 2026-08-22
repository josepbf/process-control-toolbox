"""Phase 1: which classical tuning rule is the *fair* PID baseline?

Reproduce with:

    python experiments/exp03_tuning_shootout.py

Nine published PI rules, one plant, one scenario, one seed. Each rule is
scored on time-domain performance *and* on maximum sensitivity Ms, the
frequency-domain robustness number (see process_control/tuning/analysis.py).

Why this experiment exists: the whole MPC-vs-PID literature turns on which PID
you compare against, and "tuned by a named rule" is not by itself enough --
the named rules disagree with each other by a factor of four in gain. This
picks the baseline on stated grounds, and records the grounds.

Outputs
-------
results/exp03_metrics.csv        performance + robustness, one row per rule
results/exp03_trajectories.png   the four representative responses
results/exp03_frontier.png       load-rejection IAE vs Ms -- the real trade
"""

from __future__ import annotations

import pathlib


import pandas as pd

from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.plotting import plot_runs, plot_tradeoff, save_table
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import run_all
from process_control.plants.tank import Tank
from process_control.tuning.analysis import pid_on_fopdt, ultimate_gain_period
from process_control.tuning.rules import (
    amigo_pi,
    cohen_coon,
    imc_pi,
    lambda_tuning,
    simc_pi,
    tyreus_luyben,
    ziegler_nichols_closed_loop,
    ziegler_nichols_open_loop,
)

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

#: Shown on the trajectory plot -- one from each corner of the trade.
HIGHLIGHT = ["SIMC", "AMIGO", "Cohen-Coon", "Lambda (l=tau)"]


def tunings(K: float, tau: float, theta: float) -> dict:
    """Every rule applied to the same three process parameters."""
    Ku, Pu = ultimate_gain_period(K, tau, theta)
    return {
        "SIMC": simc_pi(K, tau, theta),
        "SIMC (tau_c=2*theta)": simc_pi(K, tau, theta, tau_c=2 * theta),
        "IMC": imc_pi(K, tau, theta),
        "Lambda (l=tau)": lambda_tuning(K, tau, theta),
        "Lambda (l=3*theta)": lambda_tuning(K, tau, theta, lam=3 * theta),
        "AMIGO": amigo_pi(K, tau, theta),
        "Cohen-Coon": cohen_coon(K, tau, theta, kind="PI"),
        "ZN open-loop": ziegler_nichols_open_loop(K, tau, theta, kind="PI"),
        "ZN closed-loop": ziegler_nichols_closed_loop(Ku, Pu, kind="PI"),
        "Tyreus-Luyben": tyreus_luyben(Ku, Pu, kind="PI"),
    }


def main() -> pd.DataFrame:
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.15, seed=7)
    scenario = setpoint_and_load(dt=1.0, seed=7)
    p = plant.fopdt
    Ku, Pu = ultimate_gain_period(**p)

    print(f"plant : K={p['K']}, tau={p['tau']}, theta={p['theta']}, theta/tau={p['theta']/p['tau']:.2f}")
    print(f"        ultimate gain Ku={Ku:.3f}, ultimate period Pu={Pu:.1f} s\n")

    rules = tunings(**p)
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
        rob = pid_on_fopdt(**p, Kc=t.Kc, Ti=t.Ti, Td=t.Td)
        sp = compute_metrics(runs[label], scenario.windows["setpoint"])
        ld = compute_metrics(runs[label], scenario.windows["disturbance"])
        rows.append(
            {
                "rule": label,
                "Kc": t.Kc,
                "Ti": t.Ti,
                "Ms": rob["Ms"],
                "GM": rob["GM"],
                "PM_deg": rob["PM_deg"],
                "IAE_sp": sp["IAE"],
                "overshoot_pct": sp["overshoot_pct"],
                "settling_sp_s": sp["settling_2pct_s"],
                "IAE_load": ld["IAE"],
                "peak_dev_load": ld["peak_dev"],
                "TV_u": sp["TV_u"] + ld["TV_u"],
                "citation": t.rule,
            }
        )
    table = pd.DataFrame(rows).set_index("rule").sort_values("Ms")

    cols = ["Kc", "Ti", "Ms", "GM", "PM_deg", "IAE_sp", "overshoot_pct",
            "IAE_load", "peak_dev_load", "TV_u"]
    print(table[cols].to_string(float_format=lambda v: f"{v:8.2f}"))
    print()
    print("Ms  = maximum sensitivity (peak of 1/(1+L)); 1.2-1.6 comfortable, >2.0 fragile")
    print("IAE_sp / IAE_load are the same controller judged on the two jobs it has")
    print()

    save_table(table, RESULTS / "exp03_metrics.csv")
    plot_runs(
        {k: runs[k] for k in HIGHLIGHT},
        title="Phase 1 -- four classical PI tuning rules, same plant and seed",
        scenario=scenario,
        path=RESULTS / "exp03_trajectories.png",
        figsize=(11.0, 7.5),
    )
    plot_tradeoff(
        table.reset_index(),
        x="Ms",
        y="IAE_load",
        label_col="rule",
        title="Performance vs robustness: no rule wins both",
        x_label="maximum sensitivity Ms  (higher = closer to instability)",
        y_label="IAE, load disturbance window  (lower = better rejection)",
        good_x=(1.2, 1.6),
        path=RESULTS / "exp03_frontier.png",
    )
    print(f"wrote {RESULTS / 'exp03_trajectories.png'}")
    print(f"wrote {RESULTS / 'exp03_frontier.png'}")
    print(f"wrote {RESULTS / 'exp03_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

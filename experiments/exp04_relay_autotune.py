"""Phase 1: the industrial autotune -- relay feedback identification.

Reproduce with:

    python experiments/exp04_relay_autotune.py

Every DCS and every smart transmitter has an "autotune" button, and behind
almost all of them is Astrom & Hagglund's 1984 relay experiment: bang the loop
with a relay, measure the amplitude and period of the limit cycle it settles
into, and read the ultimate gain and period off a describing-function formula.
This runs that experiment on a plant whose true ultimate gain we can compute
analytically, so the *identification error itself* can be measured -- which is
the part a real autotune can never show you.

Outputs
-------
results/exp04_relay_experiment.png   the relay limit cycle the plant produced
results/exp04_closed_loop.png        the resulting tunings, in closed loop
results/exp04_metrics.csv
"""

from __future__ import annotations

import pathlib


import pandas as pd

from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.plotting import plot_runs, save_table
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import run_all
from process_control.plants.tank import Tank
from process_control.tuning.analysis import pid_on_fopdt, ultimate_gain_period
from process_control.tuning.relay import relay_autotune
from process_control.tuning.rules import simc_pi, tyreus_luyben, ziegler_nichols_closed_loop

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def main() -> pd.DataFrame:
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=50.0, noise_std=0.15, seed=7)
    p = plant.fopdt

    # --- the experiment ------------------------------------------------
    # Relay amplitude h = 10 %% of valve travel: enough to give an oscillation
    # well above the noise, small enough that a real plant would tolerate it.
    # Hysteresis = 0.5 %% of level, about 3x the measurement noise sigma, which
    # is what stops the relay chattering on noise.
    relay = relay_autotune(
        plant, setpoint=50.0, u_bias=plant.steady_input(50.0),
        h=10.0, hysteresis=0.5, dt=1.0, t_final=900.0, seed=7,
    )
    Ku_true, Pu_true = ultimate_gain_period(**p)

    print("relay experiment")
    print(f"  {relay.summary()}")
    print(f"  analytic truth        : Ku = {Ku_true:.3f}, Pu = {Pu_true:.1f} s")
    print(f"  identification error  : Ku {100 * (relay.Ku - Ku_true) / Ku_true:+.1f} %, "
          f"Pu {100 * (relay.Pu - Pu_true) / Pu_true:+.1f} %")
    print("  (the describing function assumes the process filters the relay's")
    print("   harmonics; with theta/tau = 0.25 it does so imperfectly, and the")
    print("   error lands on the *conservative* side of the true gain)\n")

    # --- what the plant would be tuned to, from that experiment --------
    rules = {
        "ZN-CL (from relay)": ziegler_nichols_closed_loop(relay.Ku, relay.Pu, kind="PI"),
        "Tyreus-Luyben (from relay)": tyreus_luyben(relay.Ku, relay.Pu, kind="PI"),
        "SIMC (from true model)": simc_pi(**p),
    }

    scenario = setpoint_and_load(dt=1.0, y_start=30.0, y_step=50.0, seed=7)
    controllers = {
        label: PIDController(
            **t.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=plant.steady_input(30.0), name=label, tuning_note=t.rule,
        )
        for label, t in rules.items()
    }
    runs = run_all(Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.15, seed=7),
                   controllers, scenario)

    rows = []
    for label, t in rules.items():
        rob = pid_on_fopdt(**p, Kc=t.Kc, Ti=t.Ti)
        sp = compute_metrics(runs[label], scenario.windows["setpoint"])
        ld = compute_metrics(runs[label], scenario.windows["disturbance"])
        rows.append({
            "tuning": label, "Kc": t.Kc, "Ti": t.Ti, "Ms": rob["Ms"], "GM": rob["GM"],
            "IAE_sp": sp["IAE"], "overshoot_pct": sp["overshoot_pct"],
            "IAE_load": ld["IAE"], "TV_u": sp["TV_u"] + ld["TV_u"],
            "needs_model": label.endswith("true model)"),
        })
    table = pd.DataFrame(rows).set_index("tuning")
    print(table.to_string(float_format=lambda v: f"{v:8.2f}"))
    print()

    save_table(table, RESULTS / "exp04_metrics.csv")
    plot_runs(
        {"relay experiment": relay.df},
        title=(f"Phase 1 -- relay autotune experiment  "
               f"(Ku = {relay.Ku:.2f} vs true {Ku_true:.2f}, "
               f"Pu = {relay.Pu:.0f} s vs true {Pu_true:.0f} s)"),
        path=RESULTS / "exp04_relay_experiment.png",
        figsize=(11.0, 7.0),
    )
    plot_runs(
        runs,
        title="Phase 1 -- tunings derived from the relay test vs from the true model",
        scenario=scenario,
        path=RESULTS / "exp04_closed_loop.png",
        figsize=(11.0, 7.5),
    )
    print(f"wrote {RESULTS / 'exp04_relay_experiment.png'}")
    print(f"wrote {RESULTS / 'exp04_closed_loop.png'}")
    print(f"wrote {RESULTS / 'exp04_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

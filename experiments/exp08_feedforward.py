"""Phase 1: feedforward from a measured disturbance.

Reproduce with:

    python experiments/exp08_feedforward.py

Feedback is reactive by construction: it cannot act until an error exists. If
the disturbance is instrumented, that limitation goes away for that
disturbance -- the valve can move the moment the upset is *measured*, long
before the controlled variable notices.

The plant here has genuinely different paths for the valve and the disturbance:

    valve       -> y :  K = 1.5, tau = 60 s, dead time 15 s
    disturbance -> y :  Kd = 1.0, tau =  15 s, dead time 45 s

so the ideal compensator is a real lead-lag with 30 s of usable advance:

    u_ff(s) = -(Kd/K) * (60s+1)/(15s+1) * exp(-30 s)

Four things are measured, and the last two matter more than the first two:

1. feedback only (SIMC PI);
2. + static feedforward (gain only, no dynamics);
3. + dynamic feedforward (lead-lag and delay) -- the textbook design;
4. + dynamic feedforward with a 30 %% error in the feedforward gain.

Case 4 is the honest one. Feedforward is *open loop*: nothing corrects it if
the ratio Kd/K is wrong, and that ratio drifts with throughput, moisture and
wear. Feedforward is never deployed alone; it is deployed for speed with
feedback underneath it for truth, and this experiment prices both halves.

A second plant repeats the comparison with the disturbance arriving *before*
the valve can act (theta_d = 0 < theta = 15 s). The ideal compensator is then
non-causal, so the best available design acts 15 s late -- it still helps, but
the perfect cancellation of case 3 is off the table. Saying which case you are
in is the useful engineering answer.

Outputs
-------
results/exp08_feedforward.png, results/exp08_unrealisable.png
results/exp08_metrics.csv
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pandas as pd

from src.controllers.feedforward import FeedforwardPID
from src.controllers.pid import PIDController
from src.harness.metrics import compute_metrics
from src.harness.plotting import plot_runs, save_table
from src.harness.scenarios import Scenario, constant, staircase
from src.harness.simulate import run_all
from src.plants.tank import Tank
from src.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def make_scenario() -> Scenario:
    return Scenario(
        name="measured_load_upsets",
        dt=1.0,
        t_final=1800.0,
        setpoint=constant(50.0),
        disturbance=staircase([(0.0, 0.0), (300.0, -12.0), (900.0, 6.0), (1400.0, 0.0)]),
        seed=7,
        d_noise_std=0.4,          # the disturbance transmitter has noise too
        description="measured load steps -12, +6, 0; level setpoint held at 50 %",
        windows={"upset 1": (300.0, 900.0), "upset 2": (900.0, 1400.0)},
    )


def build_controllers(plant: Tank, scenario: Scenario, ff_gain_error: float = 0.0) -> dict:
    p, pd_ = plant.fopdt, plant.fopdt_disturbance
    tuning = simc_pi(**p)
    common = dict(
        dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=plant.steady_input(50.0),
    )

    def pi(name):
        return PIDController(**tuning.as_kwargs(), **common, name=name, tuning_note=tuning.rule)

    def ff(name, static, gain_error=0.0):
        return FeedforwardPID(
            feedback=pi(name),
            K_p=p["K"] * (1 + gain_error), K_d=pd_["K"],
            tau_p=p["tau"], tau_d=pd_["tau"],
            theta_p=p["theta"], theta_d=pd_["theta"],
            dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
            static_only=static, name=name,
        )

    return {
        "PI only": pi("PI only"),
        "PI + static FF": ff("PI + static FF", static=True),
        "PI + dynamic FF": ff("PI + dynamic FF", static=False),
        "PI + dynamic FF, 30% gain error": ff(
            "PI + dynamic FF, 30% gain error", static=False, gain_error=0.30
        ),
    }


def score(runs: dict, scenario: Scenario, label_suffix: str = "") -> list[dict]:
    rows = []
    for label, df in runs.items():
        for wname, w in scenario.windows.items():
            m = compute_metrics(df, w)
            rows.append({
                "controller": label + label_suffix, "window": wname,
                "IAE": m["IAE"], "peak_dev": m["peak_dev"],
                "settling_s": m["settling_2pct_s"], "TV_u": m["TV_u"],
            })
    return rows


def main() -> pd.DataFrame:
    scenario = make_scenario()

    # --- realisable: the disturbance arrives later than the valve does ---
    plant = Tank(
        K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, theta_d=45.0,
        h0=50.0, noise_std=0.15, seed=7,
    )
    print(f"valve path       : {plant.fopdt}")
    print(f"disturbance path : {plant.fopdt_disturbance}")
    print(f"feedforward is realisable: theta_d={plant.theta_d:g} >= theta={plant.theta:g}, "
          f"{plant.theta_d - plant.theta:g} s of usable advance\n")

    runs = run_all(plant, build_controllers(plant, scenario), scenario)
    rows = score(runs, scenario)

    # --- unrealisable: the disturbance beats the valve to the output ---
    plant_bad = Tank(
        K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, theta_d=0.0,
        h0=50.0, noise_std=0.15, seed=7,
    )
    runs_bad = run_all(plant_bad, build_controllers(plant_bad, scenario), scenario)
    rows += score(runs_bad, scenario, label_suffix="  [theta_d=0]")

    table = pd.DataFrame(rows).set_index(["controller", "window"])
    print(table.to_string(float_format=lambda v: f"{v:9.3f}"))

    u1 = table.xs("upset 1", level="window")["IAE"]
    print(f"\nrealisable case, first upset:")
    print(f"  static FF   : {u1['PI only'] / u1['PI + static FF']:.1f}x better IAE than feedback alone")
    print(f"  dynamic FF  : {u1['PI only'] / u1['PI + dynamic FF']:.1f}x")
    print(f"  30% FF gain error: {u1['PI only'] / u1['PI + dynamic FF, 30% gain error']:.1f}x "
          f"-- degraded, but feedback still cleans up the residue")
    print(f"  same design with theta_d = 0 (non-causal ideal): "
          f"{u1['PI only  [theta_d=0]'] / u1['PI + dynamic FF  [theta_d=0]']:.1f}x")

    save_table(table, RESULTS / "exp08_metrics.csv")
    plot_runs(
        runs, scenario=scenario,
        title="Phase 1 -- feedforward from a measured disturbance (realisable: theta_d > theta)",
        path=RESULTS / "exp08_feedforward.png", figsize=(11.5, 8.0),
    )
    plot_runs(
        runs_bad, scenario=scenario,
        title="Phase 1 -- the same designs when the disturbance beats the valve (theta_d = 0)",
        path=RESULTS / "exp08_unrealisable.png", figsize=(11.5, 8.0),
    )
    print(f"\nwrote {RESULTS / 'exp08_feedforward.png'}")
    print(f"wrote {RESULTS / 'exp08_unrealisable.png'}")
    print(f"wrote {RESULTS / 'exp08_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

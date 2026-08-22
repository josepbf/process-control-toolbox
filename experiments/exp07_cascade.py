"""Phase 1: cascade control -- buying information instead of tuning harder.

Reproduce with:

    python experiments/exp07_cascade.py

A fast inner stage (valve -> flow, tau = 5 s) feeding a slow outer stage
(flow -> temperature, tau = 60 s). Two controllers, both tuned by SIMC:

* single loop: one PI, temperature -> valve, tuned on the half-rule reduction
  of the whole chain;
* cascade: an inner PI on the flow measurement, tuned tight, with an outer PI
  on temperature whose *output is the flow setpoint*.

The disturbance is deliberately placed in the inner stage -- a supply pressure
change, so the same valve position now delivers a different flow. This is the
case cascade was invented for. The single loop cannot know anything has
happened until the upset has worked its way through the slow lag; the inner
loop sees it within its own 5 s time constant and corrects it before the
primary variable moves much at all.

Note what is *not* being changed: the plant, the actuator limits, the sample
time, the seed, the tuning method. The only difference is one extra
measurement. That is the honest way to price a structural change -- and it is
the same accounting MPC will be put through in later phases.

Outputs
-------
results/exp07_cascade.png
results/exp07_metrics.csv
"""

from __future__ import annotations

import pathlib


import pandas as pd

from process_control.controllers.cascade import CascadeController
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.plotting import plot_runs, save_table
from process_control.harness.scenarios import Scenario, staircase
from process_control.harness.simulate import run_all
from process_control.plants.cascade_process import CascadeProcess
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"


def build():
    plant = CascadeProcess(
        K1=1.0, tau1=5.0, K2=1.2, tau2=60.0, theta=2.0,
        Kd_inner=1.0, Kd_outer=1.0, noise_std=(0.15, 0.25), seed=7,
    )
    scenario = Scenario(
        name="inner_then_outer_upset",
        dt=1.0,
        t_final=2000.0,
        setpoint=staircase([(0.0, 30.0), (200.0, 50.0)]),
        # d = [inner (supply pressure), outer (ambient loss)]
        disturbance=staircase(
            [(0.0, [0.0, 0.0]), (800.0, [-15.0, 0.0]), (1400.0, [-15.0, -8.0])]
        ),
        seed=7,
        description=(
            "primary setpoint 30 -> 50 at t=200 s; inner-stage upset -15 at t=800 s; "
            "outer-stage upset -8 at t=1400 s"
        ),
        windows={"setpoint": (200.0, 800.0), "inner upset": (800.0, 1400.0),
                 "outer upset": (1400.0, 2000.0)},
    )
    return plant, scenario


def main() -> pd.DataFrame:
    plant, scenario = build()
    u0 = plant.steady_input(30.0)

    # --- baseline: one PI on the whole chain --------------------------
    single = simc_pi(**plant.single_loop_fopdt)
    single_pi = PIDController(
        **single.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=u0, name="single-loop PI", tuning_note=single.rule,
    )

    # --- cascade: inner tuned first and tight, then the outer ---------
    inner = simc_pi(**plant.inner_fopdt)                       # tau_c = theta: fast
    outer = simc_pi(**plant.outer_fopdt)
    cascade = CascadeController(
        primary=PIDController(
            **outer.as_kwargs(), dt=scenario.dt,
            # The outer controller's "actuator" is the inner setpoint, so its
            # limits are the range of flow the inner loop can actually deliver.
            u_min=plant.K1 * plant.u_min[0], u_max=plant.K1 * plant.u_max[0],
            u0=plant.K1 * u0, name="outer PI", tuning_note=outer.rule,
        ),
        secondary=PIDController(
            **inner.as_kwargs(), dt=scenario.dt,
            u_min=plant.u_min[0], u_max=plant.u_max[0], u0=u0,
            name="inner PI", tuning_note=inner.rule,
        ),
    )

    print(f"plant   : inner {plant.inner_fopdt}, outer {plant.outer_fopdt}")
    print(f"          single-loop reduction (half rule): "
          f"{ {k: round(v, 2) for k, v in plant.single_loop_fopdt.items()} }")
    print(f"          time-constant ratio tau2/tau1 = {plant.tau2 / plant.tau1:.0f} "
          f"(cascade needs >= 3-5)")
    print(f"scenario: {scenario.description}\n")
    print(f"single-loop PI : Kc={single.Kc:6.3f}  Ti={single.Ti:7.2f}   [{single.rule}]")
    print(f"cascade  inner : Kc={inner.Kc:6.3f}  Ti={inner.Ti:7.2f}   [{inner.rule}]")
    print(f"cascade  outer : Kc={outer.Kc:6.3f}  Ti={outer.Ti:7.2f}   [{outer.rule}]\n")

    runs = run_all(plant, {"single-loop PI": single_pi, "cascade PI/PI": cascade}, scenario)

    rows = []
    for label, df in runs.items():
        for wname, w in scenario.windows.items():
            m = compute_metrics(df, w)
            rows.append({
                "controller": label, "window": wname, "IAE": m["IAE"],
                "peak_dev": m["peak_dev"], "settling_s": m["settling_2pct_s"],
                "overshoot_pct": m["overshoot_pct"], "TV_u": m["TV_u"],
            })
    table = pd.DataFrame(rows).set_index(["controller", "window"])
    print(table.to_string(float_format=lambda v: f"{v:9.3f}"))

    inner_iae = table.xs("inner upset", level="window")["IAE"]
    print(f"\ninner-stage upset: cascade IAE is "
          f"{inner_iae['single-loop PI'] / inner_iae['cascade PI/PI']:.1f}x better")
    outer_iae = table.xs("outer upset", level="window")["IAE"]
    print(f"outer-stage upset: cascade IAE is "
          f"{outer_iae['single-loop PI'] / outer_iae['cascade PI/PI']:.1f}x better "
          f"-- the extra measurement helps far less here, as it should: this "
          f"disturbance does not pass through the inner loop")

    save_table(table, RESULTS / "exp07_metrics.csv")
    plot_runs(
        runs,
        title="Phase 1 -- cascade vs single loop, upset entering the inner stage",
        scenario=scenario,
        y_label="primary y1 [temp]",
        u_label="valve u [%]",
        path=RESULTS / "exp07_cascade.png",
        figsize=(11.5, 8.0),
        # The outer loop's real output is the inner setpoint, not the valve.
        # Only the cascade publishes it, which is itself the point.
        diagnostic="inner_setpoint",
        diagnostic_label="inner SP\n(flow)",
    )
    print(f"\nwrote {RESULTS / 'exp07_cascade.png'}")
    print(f"wrote {RESULTS / 'exp07_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

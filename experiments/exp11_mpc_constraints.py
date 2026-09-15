"""Phase 2: what model predictive control actually buys.

Reproduce with:

    python experiments/exp11_mpc_constraints.py

The roadmap sets the bar for anything added after phase 1: a new strategy is
measured against a *well-structured* classical scheme with a cited tuning rule,
never against a bare PID; no comparison is reported without the effort column;
and a large claimed win on peak deviation at high theta/tau is a result to be
checked rather than celebrated. This experiment is built to meet that bar, which
means it is built to make MPC lose wherever MPC deserves to lose.

Three questions, in the order that keeps the answer honest.

**1. With nothing binding, does MPC beat a well-tuned PI?**

No -- and it should not. Tuned from the *same* FOPDT model by a named rule
(Shridhar & Cooper 1997 for the MPC, SIMC/Skogestad 2003 for the PI), the two
land within a few per cent of each other on tracking error. An unconstrained
MPC is a linear controller; `LinearMPC.linear_gain()` will even tell you which
one. There is nothing here for prediction to do.

The move-suppression weight R is an aggressiveness knob that *brackets* the PI:
small R buys tracking error with valve travel, large R gives it back. So "our
MPC beat the PI on IAE" is a statement about the R somebody picked. Panel 1
shows the whole frontier so that claim cannot be made by accident.

**2. With an output limit binding, does MPC respect it?**

Yes, and this is the structural difference. The band is placed at 50.5 %, which
is where the *well-tuned* SIMC PI's own overshoot crosses it -- not somewhere
only a deliberately bad baseline would go. The PI has no representation of a
limit it is not allowed to cross; the MPC has the limit in its constraint rows
and rides it.

**3. What does that cost?**

Valve travel, and computation. Both are in the table. Every structural
improvement in this project has cost effort -- anti-windup, cascade, feedforward
-- and this one is no exception.

What would falsify the constraint result
----------------------------------------
If MPC shows a large win, the three things to rule out, in order:

(a) it was given a better model than the PI's. It is not: both are built from
    `plant.fopdt`, the same three numbers, and the model is recorded in
    `df.attrs["controller_info"]["model"]`.
(b) it had setpoint preview the PI did not. It does not -- preview is off here,
    and `df.attrs["uses_preview"]` says so for every run.
(c) it knew about the disturbance. It does not: the load is unmeasured for
    every controller, and MPC rejects it through an estimated output-disturbance
    state, which is its integral action and is plotted as `d_hat`.

Outputs
-------
results/exp11_frontier.png      R sweep: MPC's operating range against the PI
results/exp11_constraint.png    the constrained comparison
results/exp11_horizon.png       the receding horizon itself
results/exp11_metrics.csv
"""

from __future__ import annotations

import pathlib

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from process_control.controllers.mpc import LinearMPC
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics, summarize
from process_control.harness.plotting import plot_horizon, plot_runs, save_table
from process_control.harness.scenarios import Scenario, setpoint_and_load, staircase
from process_control.harness.simulate import run_all, simulate
from process_control.models.discrete import fopdt_model
from process_control.plants.tank import Tank
from process_control.tuning.mpc_rules import shridhar_cooper
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

#: The band sits where the well-tuned PI's own overshoot crosses it. If this is
#: loosened the experiment quietly stops being about constraint handling.
Y_MAX = 50.5


def make_plant(y_max: float | None = None) -> Tank:
    return Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=30.0, noise_std=0.0, y_max=y_max)


def make_step_scenario() -> Scenario:
    return Scenario(
        name="setpoint_step_against_a_ceiling",
        dt=1.0,
        t_final=900.0,
        setpoint=staircase([(0.0, 30.0), (100.0, 50.0)]),
        description="30 -> 50 % step with a 50.5 % ceiling the PI overshoots through",
        windows={"step": (100.0, 900.0)},
    )


def build_pi(plant: Tank, scenario: Scenario) -> PIDController:
    tuning = simc_pi(**plant.fopdt)
    return PIDController(
        **tuning.as_kwargs(), dt=scenario.dt,
        u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=plant.steady_input(30.0),
        name="SIMC PI", tuning_note=tuning.rule,
    )


def build_mpc(plant: Tank, scenario: Scenario, *, y_max=None, R=None, name="MPC") -> LinearMPC:
    """Built from `plant.fopdt` -- the same three numbers the PI is tuned on.

    Deliberately not a `from_plant()` convenience: handing a model-based
    controller the true plant in one line is exactly what fairness rule 3 is
    there to prevent, and writing it out keeps the grant visible.
    """
    tuning = shridhar_cooper(**plant.fopdt, dt=scenario.dt, M=10)
    return LinearMPC(
        fopdt_model(**plant.fopdt, dt=scenario.dt),
        N=tuning.N, M=tuning.M, Q=tuning.Q,
        R=tuning.R if R is None else R,
        u_min=plant.u_min[0], u_max=plant.u_max[0],
        y_max=y_max,
        u0=plant.steady_input(30.0),
        name=name,
        tuning_note=tuning.rule if R is None else f"{tuning.rule}; R overridden to {R:g}",
    )


def sweep_move_weight(plant: Tank, scenario: Scenario, weights) -> pd.DataFrame:
    """MPC's operating range, so a tracking win cannot be passed off as structure."""
    rows = []
    for R in weights:
        metrics = compute_metrics(simulate(plant, build_mpc(plant, scenario, R=R), scenario))
        rows.append({"R": R, "IAE": metrics["IAE"], "TV_u": metrics["TV_u"],
                     "overshoot_pct": metrics["overshoot_pct"]})
    return pd.DataFrame(rows)


def main() -> pd.DataFrame:
    RESULTS.mkdir(parents=True, exist_ok=True)

    # ---- 1. unconstrained: MPC has nothing to do -------------------------
    plant, scenario = make_plant(), setpoint_and_load(dt=1.0)
    pi = build_pi(plant, scenario)
    mpc = build_mpc(plant, scenario)

    runs = run_all(plant, {"SIMC PI": pi, "MPC (Shridhar-Cooper)": mpc}, scenario)
    unconstrained = summarize(runs)

    print("1. Nothing binding -- two rules, one model, one answer")
    print("   (the tie is the result; an unconstrained MPC is a linear controller)")
    pi_iae = compute_metrics(runs["SIMC PI"])["IAE"]
    mpc_iae = compute_metrics(runs["MPC (Shridhar-Cooper)"])["IAE"]
    print(f"   SIMC PI  IAE = {pi_iae:7.1f}")
    print(f"   MPC      IAE = {mpc_iae:7.1f}   ({mpc_iae / pi_iae:.3f}x)")
    print(f"   QP iterations used by MPC, max: "
          f"{int(runs['MPC (Shridhar-Cooper)']['diag_qp_iters'].max())}")

    weights = [0.05, 0.2, 1.0, 3.0, shridhar_cooper(**plant.fopdt, dt=1.0, M=10).R, 20.0, 100.0]
    frontier = sweep_move_weight(plant, scenario, sorted(weights))

    # ---- 2. constrained: the structural difference ------------------------
    c_plant, c_scenario = make_plant(y_max=Y_MAX), make_step_scenario()
    c_pi = build_pi(c_plant, c_scenario)
    c_mpc = build_mpc(c_plant, c_scenario, y_max=Y_MAX, R=0.5, name="MPC (constrained)")

    c_runs = run_all(c_plant, {"SIMC PI": c_pi, "MPC + y_max": c_mpc}, c_scenario)
    constrained = summarize(c_runs)

    pi_m = compute_metrics(c_runs["SIMC PI"])
    mpc_m = compute_metrics(c_runs["MPC + y_max"])
    print(f"\n2. A {Y_MAX} % ceiling the well-tuned PI overshoots through")
    print(f"   SIMC PI      peak = {c_runs['SIMC PI']['y'].max():6.2f} %   "
          f"violations = {int(pi_m['n_violations']):3d}   integral = {pi_m['violation_integral']:6.2f}")
    print(f"   MPC + y_max  peak = {c_runs['MPC + y_max']['y'].max():6.2f} %   "
          f"violations = {int(mpc_m['n_violations']):3d}   integral = {mpc_m['violation_integral']:6.2f}")
    print(f"\n3. What it cost")
    print(f"   valve travel  {pi_m['TV_u']:.1f} -> {mpc_m['TV_u']:.1f}  "
          f"({mpc_m['TV_u'] / pi_m['TV_u']:.1f}x)")
    print(f"   solve time    {pi_m['solve_ms_mean']:.4f} -> {mpc_m['solve_ms_mean']:.4f} ms/sample "
          f"({mpc_m['solve_ms_mean'] / pi_m['solve_ms_mean']:.0f}x)")

    # The three things that would falsify the constraint result, checked.
    info = c_runs["MPC + y_max"].attrs
    assert info["uses_preview"] is False, "preview would be an undeclared advantage"
    assert info["uses_measured_disturbance"] is False, "MPC must not see the load"
    assert info["controller_info"]["model"]["theta_realised"] == [15.0]

    # ---- figures ----------------------------------------------------------
    _plot_frontier(frontier, pi_iae, compute_metrics(runs["SIMC PI"])["TV_u"])
    plot_runs(
        c_runs, scenario=c_scenario,
        title=f"MPC honours a {Y_MAX} % ceiling that a well-tuned SIMC PI overshoots through",
        path=RESULTS / "exp11_constraint.png", figsize=(11.5, 8.0),
        # QP iterations read directly as "how many limits the process forced on
        # me this sample" -- zero until the ceiling comes into view.
        diagnostic="qp_iters", diagnostic_label="active\nconstraints",
    )

    # A short window and a tight stride: the fan is only interesting while the
    # controller is still deciding something.
    horizon_scenario = Scenario(
        name="step_zoom", dt=1.0, t_final=320.0,
        setpoint=staircase([(0.0, 30.0), (100.0, 50.0)]),
        description="the same step, windowed on the transient",
    )
    horizon_run = simulate(
        c_plant, build_mpc(c_plant, horizon_scenario, y_max=Y_MAX, R=0.5),
        horizon_scenario, snapshot_stride=8,
    )
    plot_horizon(
        horizon_run,
        title="The receding horizon: each prediction drawn from the sample it was made at",
        path=RESULTS / "exp11_horizon.png",
        # The horizon covers the settling time (N = 316); only its first minute
        # is legible, and the rest is every curve flat on the setpoint.
        max_steps=60,
    )

    table = pd.concat(
        {"unconstrained": unconstrained, f"ceiling at {Y_MAX} %": constrained},
        names=["case"],
    )
    save_table(table, RESULTS / "exp11_metrics.csv")
    save_table(frontier, RESULTS / "exp11_frontier.csv")
    for figure in ("exp11_frontier.png", "exp11_constraint.png", "exp11_horizon.png"):
        print(f"wrote {RESULTS / figure}")
    print(f"wrote {RESULTS / 'exp11_metrics.csv'}")
    return table


def _plot_frontier(frontier: pd.DataFrame, pi_iae: float, pi_tv: float) -> None:
    """The move weight is a knob, and it brackets the classical baseline.

    Drawn because the alternative -- quoting one R -- makes a tuning choice look
    like a structural result.
    """
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(9.0, 7.5), gridspec_kw={"height_ratios": [3, 2]}
    )

    ax.plot(frontier["TV_u"], frontier["IAE"], "o-", lw=1.6, ms=5, label="MPC, R swept")
    for _, row in frontier.iterrows():
        ax.annotate(f"R={row['R']:.3g}", (row["TV_u"], row["IAE"]),
                    textcoords="offset points", xytext=(6, 4), fontsize=8)
    ax.scatter([pi_tv], [pi_iae], s=110, marker="*", color="crimson", zorder=5,
               label="SIMC PI (Skogestad 2003)")
    ax.set_xlabel("valve travel TV_u  (the cost)")
    ax.set_ylabel("IAE  (the benefit)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title("With nothing binding, the move weight brackets the classical baseline",
                 fontsize=11)

    ax2.axhline(1.0, color="crimson", ls="--", lw=1.2)
    ax2.plot(frontier["R"], frontier["IAE"] / pi_iae, "o-", lw=1.6, ms=5)
    ax2.set_xscale("log")
    ax2.set_xlabel("move suppression R  (log)")
    ax2.set_ylabel("IAE relative\nto the SIMC PI")
    ax2.grid(alpha=0.3)
    ax2.text(frontier["R"].iloc[-1], 1.0, " PI ", color="crimson", va="bottom",
             ha="right", fontsize=9)

    fig.tight_layout()
    fig.savefig(RESULTS / "exp11_frontier.png", dpi=150)


if __name__ == "__main__":
    main()

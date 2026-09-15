"""Phase 2: dead-time compensation, and what it costs when the model is wrong.

Reproduce with:

    python experiments/exp10_smith_predictor.py

Article 5 measured the damage dead time does: as theta/tau rises, a PI has to be
detuned until it is barely controlling anything, and peak deviation approaches a
physical floor no feedback controller reaches past. The Smith predictor (1957) is
the classical answer, and it is the first controller in this project to carry an
internal model -- which makes it the right place to put `models.discrete` under
load before an MPC leans on it.

The structure closes the PI around the model's *undelayed* prediction and uses
the measurement only to correct the model:

    y_feedback = y0_model + (y_measured - y_delayed_model)

If the model is right and nothing else is happening, the correction term is
identically zero and the PI is controlling a loop with no dead time in it -- so
it can be tuned as if there were none. That is the entire claim, and it is worth
exactly as much as the model is right.

Three cases, and the third is the one that matters:

1. SIMC PI on the full FOPDT model -- the honest classical baseline;
2. Smith predictor with a *perfect* model, inner PI tuned as though the only
   remaining dead time were the sample time;
3. the same predictor with theta underestimated by two thirds.

Case 3 is why this is not a free lunch. The aggressive inner tuning is protected
by a cancellation that holds only if the delay is right. Gain and time-constant
errors degrade the predictor gently; **delay errors do not**, and a Smith
predictor tuned as if it had no dead time, on a plant whose dead time it has
underestimated, can be worse than the PI it replaced.

A theta/tau sweep repeats the comparison across the regime article 5 mapped, so
the benefit can be read against the same physical floor.

Outputs
-------
results/exp10_smith_predictor.png
results/exp10_deadtime_sweep.png
results/exp10_metrics.csv
"""

from __future__ import annotations

import pathlib

import numpy as np
import pandas as pd

from process_control.controllers.pid import PIDController
from process_control.controllers.smith import SmithPredictor
from process_control.harness.metrics import compute_metrics, summarize
from process_control.harness.plotting import plot_runs, plot_sweep, save_table
from process_control.harness.scenarios import Scenario, setpoint_and_load
from process_control.harness.simulate import run_all, simulate
from process_control.models.discrete import fopdt_model
from process_control.plants.tank import Tank
from process_control.tuning.rules import simc_pi

RESULTS = pathlib.Path(__file__).resolve().parents[1] / "results"

K, TAU, THETA = 1.5, 60.0, 15.0


def make_plant(theta: float = THETA, **kw) -> Tank:
    return Tank(K=K, tau=TAU, theta=theta, Kd=1.0, h0=30.0, noise_std=0.0, **kw)


def build_pi(plant: Tank, scenario: Scenario, theta: float | None = None) -> PIDController:
    """SIMC PI. ``theta`` overrides the model dead time the rule is given."""
    model = dict(plant.fopdt)
    if theta is not None:
        model["theta"] = theta
    tuning = simc_pi(**model)
    return PIDController(
        **tuning.as_kwargs(), dt=scenario.dt,
        u_min=plant.u_min[0], u_max=plant.u_max[0], u0=plant.steady_input(30.0),
        name="SIMC PI", tuning_note=tuning.rule,
    )


def build_smith(plant: Tank, scenario: Scenario, model_theta: float | None = None,
                name: str = "Smith predictor") -> SmithPredictor:
    """Inner PI tuned as if the only dead time left were the sample time.

    That is the honest version of "as if there were none": the sample time is
    itself dead time, and a rule asked for theta = 0 would return infinite gain.
    """
    model_theta = plant.theta if model_theta is None else model_theta
    inner = simc_pi(K=plant.K, tau=plant.tau, theta=scenario.dt)
    return SmithPredictor(
        feedback=PIDController(
            **inner.as_kwargs(), dt=scenario.dt,
            u_min=plant.u_min[0], u_max=plant.u_max[0], u0=plant.steady_input(30.0),
            tuning_note=inner.rule,
        ),
        model=fopdt_model(K=plant.K, tau=plant.tau, dt=scenario.dt, delay_states=False),
        theta=model_theta,
        u_min=plant.u_min[0], u_max=plant.u_max[0],
        name=name,
    )


def sweep_dead_time(ratios, dt: float = 1.0) -> pd.DataFrame:
    """The article-5 regime, walked again with a predictor in the loop."""
    rows = []
    for ratio in ratios:
        theta = ratio * TAU
        plant = make_plant(theta=theta)
        scenario = setpoint_and_load(dt=dt, t_final=max(1400.0, 20.0 * theta))

        pi = compute_metrics(simulate(plant, build_pi(plant, scenario), scenario))
        smith = compute_metrics(simulate(plant, build_smith(plant, scenario), scenario))
        # The same predictor, believing theta is a third of what it is.
        wrong = compute_metrics(
            simulate(plant, build_smith(plant, scenario, model_theta=theta / 3.0), scenario)
        )

        rows.append({
            "theta_over_tau": ratio,
            "IAE_pi": pi["IAE"],
            "IAE_smith": smith["IAE"],
            "IAE_smith_wrong_theta": wrong["IAE"],
            "IAE_ratio": smith["IAE"] / pi["IAE"],
            "IAE_ratio_wrong_theta": wrong["IAE"] / pi["IAE"],
            "TV_pi": pi["TV_u"],
            "TV_smith": smith["TV_u"],
        })
    return pd.DataFrame(rows)


def main() -> pd.DataFrame:
    RESULTS.mkdir(parents=True, exist_ok=True)

    plant, scenario = make_plant(), setpoint_and_load(dt=1.0)
    controllers = {
        "SIMC PI": build_pi(plant, scenario),
        "Smith (perfect model)": build_smith(plant, scenario),
        "Smith (theta underestimated 3x)": build_smith(
            plant, scenario, model_theta=THETA / 3.0, name="Smith (wrong theta)"
        ),
    }
    runs = run_all(plant, controllers, scenario)
    table = summarize(runs)

    pi_m = compute_metrics(runs["SIMC PI"])
    good = compute_metrics(runs["Smith (perfect model)"])
    bad = compute_metrics(runs["Smith (theta underestimated 3x)"])

    print(f"theta/tau = {THETA / TAU:.2f}")
    print(f"  SIMC PI                       IAE = {pi_m['IAE']:7.1f}  TV_u = {pi_m['TV_u']:6.1f}")
    print(f"  Smith, perfect model          IAE = {good['IAE']:7.1f}  TV_u = {good['TV_u']:6.1f}"
          f"   ({pi_m['IAE'] / good['IAE']:.2f}x better, {good['TV_u'] / pi_m['TV_u']:.1f}x the travel)")
    print(f"  Smith, theta underestimated   IAE = {bad['IAE']:7.1f}  TV_u = {bad['TV_u']:6.1f}"
          f"   ({bad['IAE'] / good['IAE']:.2f}x worse than the honest predictor)")

    sweep = sweep_dead_time([0.1, 0.25, 0.5, 1.0, 2.0, 4.0])
    print("\ntheta/tau sweep -- IAE relative to the SIMC PI at the same ratio")
    print("  (below 1.0 the predictor helps; above 1.0 it is a liability)")
    for _, row in sweep.iterrows():
        print(f"  theta/tau = {row['theta_over_tau']:4.2f}   perfect model {row['IAE_ratio']:5.2f}x"
              f"   wrong theta {row['IAE_ratio_wrong_theta']:5.2f}x")

    plot_runs(
        runs, scenario=scenario,
        title="Smith predictor: the benefit, and what an underestimated dead time does to it",
        path=RESULTS / "exp10_smith_predictor.png", figsize=(11.5, 8.0),
        # The correction term is the whole structure: zero on a perfect model,
        # and the thing that misbehaves when the delay is wrong.
        diagnostic="model_error", diagnostic_label="model error\n(the correction)",
    )
    plot_sweep(
        sweep, x="theta_over_tau",
        panels=[
            {"series": ["IAE_pi", "IAE_smith", "IAE_smith_wrong_theta"],
             "ylabel": "IAE", "logy": True},
            {"series": ["IAE_ratio", "IAE_ratio_wrong_theta"],
             "ylabel": "IAE relative\nto the SIMC PI", "hlines": [1.0]},
            {"series": ["TV_pi", "TV_smith"], "ylabel": "valve travel"},
        ],
        title="Dead-time compensation across the regime article 5 mapped",
        x_label="theta / tau", logx=True,
        path=RESULTS / "exp10_deadtime_sweep.png",
    )

    save_table(table, RESULTS / "exp10_metrics.csv")
    save_table(sweep, RESULTS / "exp10_sweep.csv")
    print(f"\nwrote {RESULTS / 'exp10_smith_predictor.png'}")
    print(f"wrote {RESULTS / 'exp10_deadtime_sweep.png'}")
    print(f"wrote {RESULTS / 'exp10_metrics.csv'}")
    return table


if __name__ == "__main__":
    main()

"""The Smith predictor: what it buys, what it costs, and when it turns on you."""

import numpy as np
import pytest

from process_control.controllers.pid import PIDController
from process_control.controllers.smith import SmithPredictor
from process_control.harness.metrics import compute_metrics
from process_control.harness.scenarios import Scenario, constant, setpoint_and_load
from process_control.harness.simulate import simulate
from process_control.models.discrete import fopdt_model
from process_control.plants.tank import Tank
from process_control.tuning.rules import simc_pi

TRUE = dict(K=1.5, tau=60.0, theta=15.0)


def _pi(K, tau, theta, **kw):
    tuning = simc_pi(K=K, tau=tau, theta=theta)
    return PIDController(
        **tuning.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0, u0=20.0,
        tuning_note=tuning.rule, **kw
    )


def _smith(model_theta=None, model_K=None, model_tau=None, inner_theta=1.0):
    """A Smith predictor whose model can be deliberately wrong.

    The inner PI is tuned as if the only remaining dead time were the sample
    time -- which is the honest version of "as if there were none", since the
    sample time is itself dead time.
    """
    model_theta = TRUE["theta"] if model_theta is None else model_theta
    K = TRUE["K"] if model_K is None else model_K
    tau = TRUE["tau"] if model_tau is None else model_tau
    return SmithPredictor(
        feedback=_pi(K, tau, inner_theta),
        model=fopdt_model(K=K, tau=tau, dt=1.0, delay_states=False),
        theta=model_theta,
        u_min=0.0,
        u_max=100.0,
    )


def _plant(**kw):
    return Tank(**{**TRUE, "Kd": 1.0, "noise_std": 0.0, **kw})


# ----------------------------------------------------------------------
# the structure's defining property
# ----------------------------------------------------------------------
def test_a_perfect_model_leaves_nothing_for_the_feedback_path_to_say():
    """The correction term is the model error. With the right model and no
    disturbance it is zero, which is exactly the claim the structure rests on:
    the inner PI is then controlling a loop with no dead time in it."""
    scenario = Scenario("step", dt=1.0, t_final=600.0, setpoint=constant(50.0))
    df = simulate(_plant(h0=30.0), _smith(), scenario)

    # Allow the startup transient: the model is seeded from the first reading,
    # so the first theta seconds carry the seeding, not a modelling failure.
    settled = df["diag_model_error"].to_numpy(float)[30:]
    assert np.max(np.abs(settled)) < 1e-6


def test_the_model_error_is_exactly_the_disturbance_the_model_cannot_see():
    """An unmeasured load is, by construction, the part of reality the internal
    model has no representation of -- so it turns up undiminished in the
    correction term. That is the signal being published, and it is why watching
    it tells you whether the predictor is coping."""
    scenario = Scenario(
        "load", dt=1.0, t_final=2000.0, setpoint=constant(50.0),
        disturbance=constant(-12.0),
    )
    df = simulate(_plant(h0=30.0, Kd=1.0), _smith(), scenario)
    assert float(df["diag_model_error"].iloc[-1]) == pytest.approx(-12.0, abs=0.2)


def test_a_model_carrying_dead_time_states_is_refused():
    """The structure needs the undelayed and the delayed prediction as two
    separate signals, and an augmented model only ever gives the delayed one."""
    with pytest.raises(ValueError, match="dead-time-free"):
        SmithPredictor(
            feedback=_pi(**TRUE),
            model=fopdt_model(**TRUE, dt=1.0),      # delay states included
            theta=15.0,
        )


# ----------------------------------------------------------------------
# what it buys, and what it costs
# ----------------------------------------------------------------------
def test_the_smith_predictor_beats_a_simc_pi_on_tracking_error():
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    pi = compute_metrics(simulate(plant, _pi(**TRUE), scenario))
    smith = compute_metrics(simulate(plant, _smith(), scenario))
    assert smith["IAE"] < 0.75 * pi["IAE"]


def test_and_it_pays_for_that_in_valve_travel():
    """Every structural improvement in this project has cost effort, and this
    one is no exception. A comparison that reports the tracking win without the
    effort column is not reporting the result."""
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    pi = compute_metrics(simulate(plant, _pi(**TRUE), scenario))
    smith = compute_metrics(simulate(plant, _smith(), scenario))
    assert smith["TV_u"] > pi["TV_u"]


def test_it_still_removes_steady_state_offset():
    """The predictor supplies timing, not integral action -- the inner PI does
    that, and the controller is only ever as offset-free as its inner loop."""
    scenario = Scenario(
        "load", dt=1.0, t_final=3000.0, setpoint=constant(50.0),
        disturbance=constant(-12.0),
    )
    df = simulate(_plant(h0=30.0), _smith(), scenario)
    assert float(df["y"].iloc[-1]) == pytest.approx(50.0, abs=1e-2)


# ----------------------------------------------------------------------
# where it turns on you
# ----------------------------------------------------------------------
def test_underestimating_the_dead_time_is_what_actually_hurts():
    """The aggressive inner tuning is protected by a cancellation that only
    holds if theta is right. Get the gain or the time constant wrong and the
    predictor degrades; get theta wrong -- low, in particular -- and it can be
    worse than the PI it replaced. This is the caveat that belongs in the
    result, not in a footnote."""
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)

    honest = compute_metrics(simulate(plant, _smith(), scenario))
    optimistic = compute_metrics(simulate(plant, _smith(model_theta=5.0), scenario))

    assert optimistic["IAE"] > honest["IAE"]
    assert optimistic["TV_u"] > honest["TV_u"]       # and it thrashes the valve


def test_gain_mismatch_degrades_it_far_more_gently_than_delay_mismatch():
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    honest = compute_metrics(simulate(plant, _smith(), scenario))
    wrong_gain = compute_metrics(simulate(plant, _smith(model_K=1.5 * 1.3), scenario))
    wrong_delay = compute_metrics(simulate(plant, _smith(model_theta=5.0), scenario))

    gain_penalty = wrong_gain["IAE"] / honest["IAE"]
    delay_penalty = wrong_delay["IAE"] / honest["IAE"]
    assert delay_penalty > gain_penalty


# ----------------------------------------------------------------------
# the contract every controller signs
# ----------------------------------------------------------------------
def test_two_consecutive_runs_are_identical_after_reset():
    """Including the internal model's state and its delay line -- reset() has to
    reach all the way down, and a stale delay line is the easiest piece to
    forget."""
    plant, scenario = _plant(noise_std=0.2, seed=7), setpoint_and_load(dt=1.0)
    smith = _smith()
    a = simulate(plant, smith, scenario)
    b = simulate(plant, smith, scenario)
    assert np.array_equal(a["y"].to_numpy(), b["y"].to_numpy())
    assert np.array_equal(a["u"].to_numpy(), b["u"].to_numpy())


def test_it_starts_from_the_measurement_rather_than_kicking_the_valve():
    """Bumpless start: the model is seeded so the correction term begins at
    zero. Without it the first move is a step and every settling-time number in
    the run is measuring the startup, not the control."""
    scenario = Scenario("hold", dt=1.0, t_final=100.0, setpoint=constant(30.0))
    df = simulate(_plant(h0=30.0), _smith(), scenario)
    assert float(df["diag_model_error"].iloc[0]) == pytest.approx(0.0, abs=1e-9)
    assert float(df["u"].iloc[0]) == pytest.approx(20.0, abs=1e-6)   # the inner PI's u0


def test_it_never_asks_for_a_move_outside_the_actuator_range():
    scenario = Scenario("big step", dt=1.0, t_final=600.0, setpoint=constant(95.0))
    df = simulate(_plant(h0=10.0), _smith(), scenario)
    u = df["u"].to_numpy(float)
    assert u.min() >= 0.0 - 1e-9 and u.max() <= 100.0 + 1e-9


def test_the_run_record_names_the_model_it_trusted():
    """Fairness rule 1 and rule 3 together: a model-based controller has to
    declare both its tuning and the model it was given."""
    df = simulate(_plant(), _smith(), setpoint_and_load(dt=1.0))
    info = df.attrs["controller_info"]
    assert info["theta_model"] == 15.0
    assert info["model"]["dt"] == 1.0
    assert "Smith 1957" in info["tuning"]
    assert "unspecified" not in info["tuning"]

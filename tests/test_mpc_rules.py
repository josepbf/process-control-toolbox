"""MPC tuning from a named rule, and what it agrees with."""

import numpy as np
import pytest

from process_control.controllers.mpc import LinearMPC
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.scenarios import setpoint_and_load
from process_control.harness.simulate import simulate
from process_control.models.discrete import fopdt_model
from process_control.plants.tank import Tank
from process_control.tuning.mpc_rules import MPCTuning, settling_horizon, shridhar_cooper
from process_control.tuning.rules import simc_pi

TRUE = dict(K=1.5, tau=60.0, theta=15.0)


def test_the_rule_reproduces_its_published_formulas():
    """k = theta/T + 1, N = 5 tau/T + k, f = (M/500)(3.5 tau/T + 2 - (M-1)/2),
    lambda = f K^2 (Shridhar & Cooper 1997, Table 1)."""
    tuning = shridhar_cooper(K=1.5, tau=60.0, theta=15.0, dt=1.0, M=10)

    assert tuning.N == 316                       # 5*60 + (15 + 1)
    expected_f = (10 / 500.0) * (3.5 * 60.0 + 2.0 - 4.5)
    assert tuning.R == pytest.approx(expected_f * 1.5**2)
    assert tuning.Q == 1.0


def test_move_suppression_scales_with_the_square_of_the_process_gain():
    """Because the cost compares an output error against an input move, and
    those are in different units. A raw R copied between loops is meaningless
    without this, which is the whole reason a rule beats a knob."""
    base = shridhar_cooper(K=1.0, tau=60.0, theta=15.0, dt=1.0, M=5)
    doubled = shridhar_cooper(K=2.0, tau=60.0, theta=15.0, dt=1.0, M=5)
    assert doubled.R == pytest.approx(4.0 * base.R)


def test_the_horizon_covers_the_open_loop_settling_time():
    """LinearMPC has no terminal cost -- the horizon is the stability argument,
    so it has to be long enough to see the process settle."""
    assert shridhar_cooper(**TRUE, dt=1.0, M=5).N == settling_horizon(60.0, 15.0, 1.0)
    assert settling_horizon(tau=60.0, theta=15.0, dt=1.0) == 316
    assert settling_horizon(tau=60.0, theta=15.0, dt=5.0) == 64      # 60 + 3 + 1


def test_a_single_move_horizon_is_floored_and_says_so():
    """The published rule gives lambda = 0 at M = 1, which LinearMPC would
    refuse. The substitution is named rather than hidden."""
    tuning = shridhar_cooper(**TRUE, dt=1.0, M=1)
    assert tuning.R > 0.0
    assert "floored" in tuning.rule
    LinearMPC(fopdt_model(**TRUE, dt=1.0), **tuning.as_kwargs())     # must construct


def test_a_control_horizon_outside_the_rule_s_range_is_refused():
    """f goes negative once M swamps the 3.5 tau/T term. That is out of range,
    not a weight -- returning it would be inventing tuning the paper does not
    support."""
    with pytest.raises(ValueError, match="non-positive"):
        shridhar_cooper(K=1.0, tau=1.0, theta=0.0, dt=1.0, M=20)


def test_the_tuning_splats_into_the_controller_like_a_pid_tuning_does():
    tuning = shridhar_cooper(**TRUE, dt=1.0, M=10)
    mpc = LinearMPC(
        fopdt_model(**TRUE, dt=1.0), **tuning.as_kwargs(),
        u_min=0.0, u_max=100.0, u0=20.0, tuning_note=tuning.rule,
    )
    assert mpc.N == tuning.N and mpc.M == tuning.M
    assert "Shridhar" in mpc.tuning_note
    assert isinstance(tuning, MPCTuning)


def test_two_unrelated_classical_rules_land_on_the_same_closed_loop():
    """The cross-check worth having.

    SIMC (Skogestad 2003) tunes a PI from the FOPDT model; Shridhar-Cooper
    (1997) tunes a DMC from the *same* FOPDT model. Two different rules, two
    different control structures, one model -- and they arrive at the same
    tracking error. That is evidence for both the MPC implementation and the
    tuning rule, in a way that neither checking a rule against itself nor
    checking an MPC against a hand-picked weight could be.

    It is also the honest headline: with no constraint active there is nothing
    for MPC to do that a well-tuned PI is not already doing.
    """
    plant, scenario = Tank(**TRUE, Kd=1.0, noise_std=0.0), setpoint_and_load(dt=1.0)

    pi_tuning = simc_pi(**TRUE)
    pi = PIDController(
        **pi_tuning.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0, u0=20.0,
        tuning_note=pi_tuning.rule,
    )
    mpc_tuning = shridhar_cooper(**TRUE, dt=1.0, M=10)
    mpc = LinearMPC(
        fopdt_model(**TRUE, dt=1.0), **mpc_tuning.as_kwargs(),
        u_min=0.0, u_max=100.0, u0=20.0, tuning_note=mpc_tuning.rule,
    )

    pi_metrics = compute_metrics(simulate(plant, pi, scenario))
    mpc_metrics = compute_metrics(simulate(plant, mpc, scenario))

    assert mpc_metrics["IAE"] == pytest.approx(pi_metrics["IAE"], rel=0.05)
    # And MPC gets there with less overshoot, for somewhat more valve travel --
    # reported, because no comparison here is reported without the effort column.
    assert mpc_metrics["overshoot_pct"] < pi_metrics["overshoot_pct"]
    assert mpc_metrics["TV_u"] > pi_metrics["TV_u"]


def test_a_longer_control_horizon_is_tuned_more_aggressively():
    """More moves to spend means the rule trusts the model further, so the
    weight rises with M but the closed loop still gets faster."""
    plant, scenario = Tank(**TRUE, Kd=1.0, noise_std=0.0), setpoint_and_load(dt=1.0)
    results = []
    for M in (2, 5, 10):
        tuning = shridhar_cooper(**TRUE, dt=1.0, M=M)
        mpc = LinearMPC(
            fopdt_model(**TRUE, dt=1.0), **tuning.as_kwargs(),
            u_min=0.0, u_max=100.0, u0=20.0,
        )
        results.append(compute_metrics(simulate(plant, mpc, scenario))["IAE"])
    assert results == sorted(results, reverse=True)


def test_a_degenerate_model_is_refused():
    with pytest.raises(ValueError, match="tau must be positive"):
        shridhar_cooper(K=1.0, tau=0.0, theta=1.0, dt=1.0)
    with pytest.raises(ValueError, match="dt must be positive"):
        shridhar_cooper(K=1.0, tau=10.0, theta=1.0, dt=0.0)
    with pytest.raises(ValueError, match="at least one move"):
        shridhar_cooper(K=1.0, tau=10.0, theta=1.0, dt=1.0, M=0)

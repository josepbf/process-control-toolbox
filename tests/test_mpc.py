"""Linear MPC: the claims it makes, and the ones it is not allowed to make."""

import numpy as np
import pytest

from process_control.controllers.mpc import LinearMPC
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import compute_metrics
from process_control.harness.scenarios import Scenario, constant, setpoint_and_load, staircase
from process_control.harness.simulate import simulate
from process_control.models.discrete import fopdt_model
from process_control.plants.tank import Tank
from process_control.solvers import available_backends
from process_control.tuning.rules import simc_pi

TRUE = dict(K=1.5, tau=60.0, theta=15.0)
INSTALLED = [name for name, ok in available_backends().items() if ok]


def _mpc(**kw):
    """An MPC on a perfect model. Horizon covers theta + 4 tau, as the module says."""
    options = dict(N=60, M=10, Q=1.0, R=5.0, u_min=0.0, u_max=100.0, u0=20.0)
    options.update(kw)
    model = fopdt_model(**TRUE, dt=1.0)
    return LinearMPC(model, **options)


def _pi(**kw):
    tuning = simc_pi(**TRUE)
    return PIDController(
        **tuning.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0, u0=20.0,
        tuning_note=tuning.rule, **kw
    )


def _plant(**kw):
    return Tank(**{**TRUE, "Kd": 1.0, "noise_std": 0.0, **kw})


def _step(t_final=900.0, y_from=30.0, y_to=50.0, **kw):
    return Scenario(
        "step", dt=1.0, t_final=t_final,
        setpoint=staircase([(0.0, y_from), (100.0, y_to)]), **kw
    )


# ----------------------------------------------------------------------
# what an unconstrained MPC actually is
# ----------------------------------------------------------------------
def test_an_unconstrained_mpc_is_exactly_the_linear_law_it_reports():
    """With nothing active, MPC is a linear controller. Being able to say
    *which* linear controller is what makes it comparable to everything else in
    the project -- it gives an Ms through the existing analysis machinery, and
    it forecloses treating MPC as magic.

    Note the law is in the move increments and involves the previous move, so
    the comparison is against `u_prev + K_x x_hat + ...`, not against a
    position-form gain. That is the usual trap.
    """
    mpc = _mpc(R=20.0, u_min=-np.inf, u_max=np.inf)
    K_x, K_sp, K_u = mpc.linear_gain()

    rng = np.random.default_rng(0)
    y, worst = np.array([30.0]), 0.0
    for k in range(200):
        sp = np.array([50.0 if k > 20 else 30.0])
        u_prev = float(mpc._u_prev[0])
        if not mpc._initialised:
            mpc.observer.seed_from_measurement(y, u0=mpc.u0)
            mpc._initialised = True

        x_hat = mpc.observer.correct(y)
        expected = (
            (1.0 + float(K_u[0, 0])) * u_prev
            + float((K_x @ x_hat)[0])
            + float(K_sp[0, 0]) * float(sp[0])
        )
        # Undo the peek so compute() sees exactly the state it would have seen.
        mpc.observer._x = mpc.observer._x - mpc.observer.L @ mpc.observer.innovation

        u = float(mpc.compute(y, sp, float(k))[0])
        worst = max(worst, abs(u - expected))
        y = y + 0.02 * (1.5 * u - y) + rng.normal(0.0, 0.05, 1)

    assert worst < 1e-9


def test_a_quiet_loop_costs_the_solver_no_iterations_at_all():
    """A dual active-set method starts from the unconstrained minimiser, so the
    iteration count reads directly as 'how many limits the process forced on me
    this sample'. On a loop with nothing binding, that is zero -- which is the
    same claim as the test above, measured from the outside."""
    df = simulate(_plant(), _mpc(R=20.0), setpoint_and_load(dt=1.0))
    assert df["diag_qp_iters"].max() == 0.0


# ----------------------------------------------------------------------
# offset-free tracking
# ----------------------------------------------------------------------
def test_mpc_reaches_the_setpoint_without_offset():
    df = simulate(_plant(h0=30.0), _mpc(), _step(t_final=1500.0))
    assert float(df["y"].iloc[-1]) == pytest.approx(50.0, abs=1e-3)


def test_mpc_removes_offset_after_an_unmeasured_load_step():
    """Deliberately the same plant, scenario and assertion as
    `test_pi_removes_steady_state_offset`, so the two results are literally
    comparable rather than merely similar."""
    scenario = Scenario(
        "load", dt=1.0, t_final=3000.0, setpoint=constant(50.0),
        disturbance=constant(-12.0),
    )
    df = simulate(_plant(h0=30.0), _mpc(), scenario)
    assert float(df["y"].iloc[-1]) == pytest.approx(50.0, abs=1e-3)


def test_the_disturbance_estimate_is_the_integral_action():
    """It converges on the load it cannot measure. That signal *is* why the
    output comes back, and publishing it is what makes the mechanism visible
    rather than asserted."""
    scenario = Scenario(
        "load", dt=1.0, t_final=3000.0, setpoint=constant(50.0),
        disturbance=constant(-12.0),
    )
    df = simulate(_plant(h0=30.0), _mpc(), scenario)
    assert float(df["diag_d_hat"].iloc[-1]) == pytest.approx(-12.0, abs=0.05)


# ----------------------------------------------------------------------
# constraints -- the actual claim
# ----------------------------------------------------------------------
def test_mpc_respects_an_output_limit_that_a_well_tuned_pi_drives_through():
    """The headline result, pinned.

    The band is placed where the SIMC PI's own overshoot crosses it -- not
    somewhere only a deliberately bad baseline would violate. The roadmap's rule
    is that a new strategy is measured against a well-structured classical
    scheme, and a PI that had been detuned into violating would prove nothing.
    """
    y_max = 50.5
    plant, scenario = _plant(h0=30.0, y_max=y_max), _step()

    pi = compute_metrics(simulate(plant, _pi(), scenario))
    mpc = compute_metrics(simulate(plant, _mpc(R=0.5, y_max=y_max), scenario))

    assert pi["n_violations"] > 0                # the constraint genuinely binds
    assert mpc["n_violations"] == 0
    assert mpc["violation_integral"] == 0.0


def test_mpc_rides_the_output_limit_rather_than_backing_far_off_it():
    """Honouring a constraint by never going near it would be a different, worse
    controller. The peak should sit *on* the band."""
    y_max = 50.5
    df = simulate(_plant(h0=30.0, y_max=y_max), _mpc(R=0.5, y_max=y_max), _step())
    assert float(df["y"].max()) == pytest.approx(y_max, abs=1e-2)


def test_the_plant_never_has_to_clip_an_mpc_move():
    """Stronger than checking the logged u is in range: it asserts the
    constraint handling is the controller's, not the plant quietly covering for
    it. `plant.saturate` is the mechanism the controller must never need."""
    plant = _plant(h0=10.0, u_min=0.0, u_max=100.0)
    df = simulate(plant, _mpc(R=0.5), Scenario(
        "big step", dt=1.0, t_final=600.0, setpoint=constant(95.0)))
    u = df["u"].to_numpy(float)
    assert np.allclose(u, np.clip(u, 0.0, 100.0), atol=1e-12)


def test_the_move_rate_limit_is_honoured_at_every_sample():
    du_max = 0.8
    df = simulate(
        _plant(h0=30.0), _mpc(R=0.1, du_max=du_max),
        _step(t_final=1200.0),
    )
    assert compute_metrics(df)["max_du"] <= du_max + 1e-9


def test_a_soft_output_constraint_degrades_instead_of_going_infeasible():
    """The run starts outside its own band, which a hard output constraint would
    make infeasible at the very first sample -- a silent failure buried in a
    sweep. The soft version gives up some of the band, says how much, and
    finishes the run."""
    y_max = 40.0
    df = simulate(
        _plant(h0=60.0, y_max=y_max),               # starts 20 above the limit
        _mpc(R=0.5, y_max=y_max),
        Scenario("recover", dt=1.0, t_final=1200.0, setpoint=constant(35.0)),
    )
    assert len(df) == 1201                          # it completed
    assert float(df["diag_slack"].iloc[0]) > 0.0    # and admitted the band was broken
    assert float(df["diag_slack"].iloc[-1]) == pytest.approx(0.0, abs=1e-6)
    assert (df["diag_qp_status"] == 0.0).all()      # never fell back
    assert float(df["y"].iloc[-1]) == pytest.approx(35.0, abs=1e-2)


def test_no_constraint_means_no_slack_is_ever_spent():
    df = simulate(_plant(h0=30.0, y_max=80.0), _mpc(y_max=80.0), _step())
    assert float(df["diag_slack"].max()) == pytest.approx(0.0, abs=1e-9)


# ----------------------------------------------------------------------
# the claim MPC is NOT allowed to make
# ----------------------------------------------------------------------
def test_the_move_weight_spans_the_pi_so_a_tracking_win_alone_proves_nothing():
    """R is an aggressiveness knob, and it brackets the classical baseline: a
    small R buys tracking error with valve travel, a large R gives it back. So
    "MPC beat the PI on IAE" is a statement about the R somebody chose, not
    about MPC.

    This is the test that stops the headline result being written the wrong way
    round. What MPC has that the PI does not is constraint handling, which is a
    structural difference and is tested above -- not a lower IAE, which is a
    tuning difference and is available to both.
    """
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    pi_iae = compute_metrics(simulate(plant, _pi(), scenario))["IAE"]

    aggressive = compute_metrics(simulate(plant, _mpc(R=0.05), scenario))
    sluggish = compute_metrics(simulate(plant, _mpc(R=100.0), scenario))

    assert aggressive["IAE"] < pi_iae < sluggish["IAE"]
    assert aggressive["TV_u"] > sluggish["TV_u"]     # and it was paid for in travel


def test_heavier_move_suppression_always_costs_tracking_and_buys_quiet():
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    runs = [compute_metrics(simulate(plant, _mpc(R=R), scenario)) for R in (1.0, 5.0, 20.0, 100.0)]
    iae = [m["IAE"] for m in runs]
    travel = [m["TV_u"] for m in runs]
    assert iae == sorted(iae)                        # monotonically worse tracking
    assert travel == sorted(travel, reverse=True)    # monotonically quieter valve


# ----------------------------------------------------------------------
# the horizon
# ----------------------------------------------------------------------
def test_the_horizon_snapshot_predicts_what_actually_happens():
    """Perfect model, no noise, nothing active: the one-step prediction has to
    match the next measurement. This exercises the whole chain end to end --
    discretisation, dead-time states, observer, prediction matrices -- and a
    fault anywhere in it shows up here.

    The plant integrates with RK4 and the model with an exact matrix
    exponential, so they agree to integration error rather than exactly.
    """
    df = simulate(_plant(h0=30.0), _mpc(R=20.0), _step(t_final=600.0), snapshot_stride=10)
    snaps = df.attrs["snapshots"]
    assert snaps

    y = df["y"].to_numpy(float)
    for snap in snaps[1:]:                          # skip the seeded first sample
        assert snap["y_pred"][0, 0] == pytest.approx(y[snap["k"] + 1], abs=1e-3)


def test_the_planned_moves_are_held_past_the_control_horizon():
    """Increments past M are zero, so the last move stands to the end of the
    prediction horizon. That is what makes a short control horizon stabilising
    rather than short-sighted, and it is worth pinning."""
    df = simulate(_plant(h0=30.0), _mpc(M=5), _step(t_final=400.0), snapshot_stride=50)
    plan = df.attrs["snapshots"][1]["u_plan"][:, 0]
    assert np.allclose(plan[5:], plan[4], atol=1e-12)


def test_preview_is_declared_and_reaches_the_target_trajectory():
    mpc = _mpc(R=1.0, preview=True)
    assert mpc.uses_preview is True and mpc.preview_horizon == mpc.N

    df = simulate(_plant(h0=30.0), mpc, _step(t_final=600.0), snapshot_stride=10)
    assert df.attrs["uses_preview"] is True

    # A snapshot taken before the step must already show the step in its target.
    early = [s for s in df.attrs["snapshots"] if s["t"] == 60.0][0]
    assert early["sp_target"][0, 0] == 30.0
    assert early["sp_target"][-1, 0] == 50.0


def test_preview_lets_the_controller_move_before_the_setpoint_does():
    """The information advantage, made visible. It is also why the flag exists:
    a controller that moves early is not comparable to one that cannot, unless
    the asymmetry is declared -- and it is, in the run record."""
    plant, scenario = _plant(h0=30.0), _step(t_final=600.0)
    blind = simulate(plant, _mpc(R=1.0), scenario)
    seeing = simulate(plant, _mpc(R=1.0, preview=True), scenario)

    just_before = (blind["t"] >= 85.0) & (blind["t"] < 100.0)
    assert np.allclose(blind.loc[just_before, "u"].to_numpy(float), 20.0, atol=1e-6)
    assert not np.allclose(seeing.loc[just_before, "u"].to_numpy(float), 20.0, atol=1e-6)


# ----------------------------------------------------------------------
# the contract every controller signs
# ----------------------------------------------------------------------
def test_two_consecutive_runs_are_identical_after_reset():
    plant, scenario = _plant(noise_std=0.2, seed=3), setpoint_and_load(dt=1.0)
    mpc = _mpc(R=1.0)
    a = simulate(plant, mpc, scenario)
    b = simulate(plant, mpc, scenario)
    assert np.array_equal(a["y"].to_numpy(), b["y"].to_numpy())
    assert np.array_equal(a["u"].to_numpy(), b["u"].to_numpy())


def test_reset_clears_the_warm_start_and_the_observer():
    """A wound-up instance must give a fresh instance's first move. The warm
    start is the piece most easily forgotten, and `Controller.reset` names it."""
    plant, scenario = _plant(h0=30.0), _step()
    used = _mpc(R=0.5)
    simulate(plant, used, scenario)

    fresh = _mpc(R=0.5)
    assert float(simulate(plant, used, scenario)["u"].iloc[0]) == pytest.approx(
        float(simulate(plant, fresh, scenario)["u"].iloc[0])
    )


def test_it_starts_at_u0_rather_than_kicking_the_valve():
    df = simulate(_plant(h0=30.0), _mpc(), _step(y_from=30.0))
    assert float(df["u"].iloc[0]) == pytest.approx(20.0, abs=1e-6)


def test_the_tuning_note_is_never_left_unspecified():
    """Fairness rule 1. The default is deliberately embarrassing, and a
    model-based controller has more to declare than most."""
    mpc = _mpc()
    assert "unspecified" not in mpc.tuning_note
    assert "N=60" in mpc.tuning_note and "R=5" in mpc.tuning_note


def test_the_run_record_carries_the_model_the_observer_and_the_solver():
    """A run solved by one backend at one tolerance is not the same run as
    another, and the model a model-based controller was handed is exactly the
    thing fairness rule 3 exists to make visible."""
    info = simulate(_plant(), _mpc(), setpoint_and_load(dt=1.0)).attrs["controller_info"]
    assert info["solver"] == "dense"
    assert info["fallbacks"] == 0
    assert info["model"]["theta_realised"] == [15.0]
    assert info["observer"]["disturbance_kind"] == "output"
    assert info["N"] == 60 and info["M"] == 10


def test_the_solve_time_lands_in_the_metrics_table_and_exceeds_a_pid_s():
    """Fairness rule 5's column, finally earning its keep: MPC is the first
    controller here whose computational cost is worth reporting."""
    plant, scenario = _plant(), setpoint_and_load(dt=1.0)
    pid = compute_metrics(simulate(plant, _pi(), scenario))
    mpc = compute_metrics(simulate(plant, _mpc(), scenario))
    assert np.isfinite(mpc["solve_ms_mean"])
    assert mpc["solve_ms_mean"] > pid["solve_ms_mean"]


# ----------------------------------------------------------------------
# construction
# ----------------------------------------------------------------------
def test_a_zero_move_weight_is_refused():
    """R = 0 makes the QP non-strictly-convex. Refusing beats regularising
    silently, which would change the problem without saying so."""
    with pytest.raises(ValueError, match="R must be strictly positive"):
        _mpc(R=0.0)


def test_declaring_a_measured_disturbance_without_a_model_for_it_is_refused():
    with pytest.raises(ValueError, match="no disturbance path"):
        _mpc(uses_measured_disturbance=True)


def test_the_control_horizon_never_exceeds_the_prediction_horizon():
    assert _mpc(N=10, M=40).M == 10


@pytest.mark.parametrize("backend", [b for b in INSTALLED if b != "dense"])
def test_the_qp_backends_produce_the_same_closed_loop(backend):
    plant, scenario = _plant(h0=30.0, y_max=50.5), _step()
    dense = compute_metrics(simulate(plant, _mpc(R=0.5, y_max=50.5), scenario))
    other = compute_metrics(
        simulate(plant, _mpc(R=0.5, y_max=50.5, solver=backend), scenario)
    )
    assert other["IAE"] == pytest.approx(dense["IAE"], rel=1e-3)

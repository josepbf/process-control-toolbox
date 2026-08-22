"""The harness itself: logging, isolation between runs, reproducibility."""

import numpy as np
import pytest

from process_control.controllers.base import Controller
from process_control.controllers.pid import PIDController
from process_control.harness.metrics import summarize
from process_control.harness.scenarios import Scenario, constant, pulse, staircase
from process_control.harness.simulate import run_all, simulate
from process_control.plants.tank import Tank


class ConstantController(Controller):
    """Open-loop reference: ignores the measurement entirely."""

    def __init__(self, u):
        self.u = float(u)
        self.name = "constant"
        self.tuning_note = "open loop"

    def compute(self, y, setpoint, t):
        return np.array([self.u])

    def reset(self):
        pass


def _scenario(**kw):
    return Scenario("s", dt=1.0, t_final=200.0, setpoint=constant(50.0), **kw)


def test_log_shape_and_columns():
    plant, scenario = Tank(noise_std=0.1), _scenario()
    df = simulate(plant, ConstantController(30.0), scenario)

    assert len(df) == scenario.n_steps + 1
    for col in ("t", "y", "sp", "u", "d", "violation", "violated", "solve_time"):
        assert col in df.columns
    assert df["t"].iloc[-1] == pytest.approx(scenario.t_final)
    assert df.attrs["seed"] == scenario.seed
    assert df.attrs["controller"] == "constant"


def test_runs_are_reproducible():
    plant, scenario = Tank(noise_std=0.4, seed=11), _scenario()
    a = simulate(plant, ConstantController(30.0), scenario)
    b = simulate(plant, ConstantController(30.0), scenario)
    assert np.array_equal(a["y"].to_numpy(), b["y"].to_numpy())


def test_simulate_does_not_mutate_the_caller_plant():
    """Every controller must start from the same plant state, so the runner
    works on a copy rather than on the object the experiment holds."""
    plant, scenario = Tank(h0=30.0, noise_std=0.0), _scenario()
    simulate(plant, ConstantController(80.0), scenario)
    assert plant.t == 0.0
    assert plant.x[0] == pytest.approx(30.0)


def test_run_all_isolates_controllers_from_each_other():
    plant, scenario = Tank(noise_std=0.2, seed=5), _scenario()
    runs = run_all(
        plant,
        {"a": ConstantController(30.0), "b": ConstantController(60.0), "a2": ConstantController(30.0)},
        scenario,
    )
    assert np.array_equal(runs["a"]["y"].to_numpy(), runs["a2"]["y"].to_numpy())
    assert not np.array_equal(runs["a"]["y"].to_numpy(), runs["b"]["y"].to_numpy())


def test_actuator_limits_are_logged_for_the_metrics():
    plant = Tank(u_min=0.0, u_max=40.0)
    df = simulate(plant, ConstantController(30.0), _scenario())
    assert df.attrs["u_max"] == [40.0]


def test_output_limit_violations_are_counted():
    """Phase 1 records violations without enforcing them -- that band is what
    phase 2 will hand to MPC as a hard constraint."""
    plant = Tank(h0=30.0, noise_std=0.0, y_max=55.0)
    df = simulate(plant, ConstantController(60.0), _scenario())   # drives to 90 %
    assert df["violated"].sum() > 0
    assert df["violation"].max() > 0.0
    assert summarize({"c": df})["n_violations"].iloc[0] > 0


def test_solve_time_is_recorded():
    df = simulate(Tank(), PIDController(Kc=1.0, Ti=60.0, dt=1.0, u0=20.0), _scenario())
    assert (df["solve_time"].iloc[:-1] > 0).all()
    assert np.isnan(df["solve_time"].iloc[-1])       # no controller call on the last row


def test_scenario_signal_builders():
    sp = staircase([(0.0, 10.0), (50.0, 20.0)])
    assert sp(0.0)[0] == 10.0 and sp(49.9)[0] == 10.0 and sp(50.0)[0] == 20.0

    d = pulse(10.0, 20.0, amplitude=-5.0)
    assert d(9.9)[0] == 0.0 and d(10.0)[0] == -5.0 and d(20.0)[0] == 0.0

    with pytest.raises(ValueError):
        staircase([(5.0, 1.0)])                       # must define t = 0


def test_disturbance_reaches_the_plant():
    plant, scenario = Tank(noise_std=0.0), _scenario(disturbance=constant(-12.0))
    quiet = simulate(plant, ConstantController(30.0), _scenario())
    upset = simulate(plant, ConstantController(30.0), scenario)
    assert upset["y"].iloc[-1] < quiet["y"].iloc[-1] - 5.0


# ----------------------------------------------------------------------
# controller diagnostics
# ----------------------------------------------------------------------
def test_controllers_publish_nothing_by_default():
    df = simulate(Tank(), ConstantController(30.0), _scenario())
    assert not [c for c in df.columns if c.startswith("diag_")]


def test_diagnostics_are_logged_under_a_namespaced_column():
    class Chatty(ConstantController):
        def diagnostics(self):
            return {"half_u": self.u / 2.0}

    df = simulate(Tank(), Chatty(30.0), _scenario())
    assert "diag_half_u" in df.columns
    assert (df["diag_half_u"] == 15.0).all()        # including the final row


def test_diagnostic_columns_do_not_disturb_the_metrics():
    """The diag_ prefix must stay clear of the y/u/d/sp signal selection, or a
    diagnostic could silently be picked up as a process signal."""
    class Confusing(ConstantController):
        def diagnostics(self):
            return {"u_extra": 1.0, "y_hat": 2.0, "d_model": 3.0, "sp_internal": 4.0}

    plain = summarize({"c": simulate(Tank(), ConstantController(30.0), _scenario())})
    chatty = summarize({"c": simulate(Tank(), Confusing(30.0), _scenario())})
    # Solve time is wall clock and varies run to run; everything else must match.
    cols = [c for c in plain.columns if not c.startswith("solve_ms")]
    assert plain[cols].equals(chatty[cols])


def test_cascade_feedforward_and_pwm_publish_their_internal_signals():
    """The three controllers that had informative internals invisible in the log."""
    from process_control.controllers.cascade import CascadeController
    from process_control.controllers.feedforward import FeedforwardPID
    from process_control.controllers.onoff import TimeProportioningController
    from process_control.plants.cascade_process import CascadeProcess
    from process_control.tuning.rules import simc_pi

    casc_plant = CascadeProcess(noise_std=(0.0, 0.0))
    inner, outer = simc_pi(**casc_plant.inner_fopdt), simc_pi(**casc_plant.outer_fopdt)
    cascade = CascadeController(
        primary=PIDController(**outer.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0, u0=25.0),
        secondary=PIDController(**inner.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0, u0=25.0),
    )
    assert "inner_setpoint" in cascade.diagnostics()

    tank = Tank(K=1.5, Kd=1.0, tau_d=15.0, theta_d=45.0)
    ff = FeedforwardPID(
        feedback=PIDController(Kc=1.0, Ti=60.0, dt=1.0, u0=20.0),
        K_p=1.5, K_d=1.0, tau_p=60.0, tau_d=15.0, theta_p=15.0, theta_d=45.0,
        dt=1.0, u_min=0.0, u_max=100.0,
    )
    assert "u_feedforward" in ff.diagnostics()

    pwm = TimeProportioningController(
        PIDController(Kc=1.0, Ti=60.0, dt=1.0, u0=20.0), period=20.0, dt=1.0
    )
    assert "duty" in pwm.diagnostics()

    df = simulate(tank, ff, _scenario(disturbance=constant(-12.0)))
    assert (df["diag_u_feedforward"].abs() > 0).any()   # it actually contributed

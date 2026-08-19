"""Controller behaviour: the properties we actually rely on in the results."""

import numpy as np
import pytest

from src.controllers.onoff import OnOffController
from src.controllers.pid import PIDController
from src.harness.metrics import compute_metrics, reversals
from src.harness.scenarios import Scenario, constant, staircase
from src.harness.simulate import simulate
from src.plants.tank import Tank
from src.tuning.rules import simc_pi


# ----------------------------------------------------------------------
# ON/OFF
# ----------------------------------------------------------------------
def test_onoff_switches_only_outside_the_deadband():
    c = OnOffController(u_on=100.0, u_off=0.0, hysteresis=2.0)
    c.reset()
    assert c.compute(np.array([40.0]), np.array([50.0]), 0.0)[0] == 100.0  # below band
    assert c.compute(np.array([50.4]), np.array([50.0]), 1.0)[0] == 100.0  # inside -> hold
    assert c.compute(np.array([52.0]), np.array([50.0]), 2.0)[0] == 0.0    # above band
    assert c.compute(np.array([49.6]), np.array([50.0]), 3.0)[0] == 0.0    # inside -> hold


def test_onoff_limit_cycles_and_never_settles():
    """The defining property of two-position control on a lagged process."""
    plant = Tank(noise_std=0.0, seed=1)
    scenario = Scenario("s", dt=1.0, t_final=1200.0, setpoint=constant(50.0))
    df = simulate(plant, OnOffController(100.0, 0.0, hysteresis=1.0), scenario)
    m = compute_metrics(df, window=(400.0, 1200.0))

    assert m["reversals"] >= 5           # still swinging long after any transient
    assert np.isnan(m["settling_2pct_s"])  # never settles, by construction
    assert m["peak_dev"] > 5.0


# ----------------------------------------------------------------------
# PID
# ----------------------------------------------------------------------
def _pi_on_tank(plant, scenario, **kwargs):
    t = simc_pi(**plant.fopdt)
    return PIDController(
        **t.as_kwargs(), dt=scenario.dt, u_min=plant.u_min[0], u_max=plant.u_max[0], **kwargs
    )


def test_pi_removes_steady_state_offset():
    """Integral action, on both a setpoint change and a load disturbance."""
    plant = Tank(noise_std=0.0, seed=1)
    scenario = Scenario(
        "s", dt=1.0, t_final=3000.0,
        setpoint=staircase([(0.0, 30.0), (100.0, 50.0)]),
        disturbance=staircase([(0.0, 0.0), (1500.0, -12.0)]),
    )
    df = simulate(plant, _pi_on_tank(plant, scenario, u0=20.0), scenario)
    assert abs(df["y"].iloc[-1] - 50.0) < 1e-3


def test_p_only_leaves_offset():
    """Complement of the test above: without integral action there *is* offset,
    which is the whole reason PI exists."""
    plant = Tank(noise_std=0.0, seed=1)
    scenario = Scenario("s", dt=1.0, t_final=3000.0,
                        setpoint=staircase([(0.0, 30.0), (100.0, 50.0)]))
    ctrl = PIDController(Kc=1.0, Ti=None, dt=1.0, u_min=0.0, u_max=100.0, u0=20.0)
    df = simulate(plant, ctrl, scenario)
    assert abs(df["y"].iloc[-1] - 50.0) > 1.0


def test_bumpless_start_does_not_kick_the_valve():
    plant = Tank(h0=30.0, noise_std=0.0)
    scenario = Scenario("s", dt=1.0, t_final=50.0, setpoint=constant(30.0))
    df = simulate(plant, _pi_on_tank(plant, scenario, u0=plant.steady_input(30.0)), scenario)
    assert df["u"].iloc[0] == pytest.approx(20.0, abs=1e-9)


def test_anti_windup_recovers_far_faster_than_a_naive_integrator():
    """Same tuning, same plant, same scenario -- only the anti-windup differs.

    This is the regression test for the experiment 2 claim.
    """
    plant = Tank(h0=30.0, u_min=0.0, u_max=30.0, noise_std=0.0, seed=7)  # infeasible sp
    scenario = Scenario(
        "windup", dt=1.0, t_final=2400.0,
        setpoint=staircase([(0.0, 30.0), (100.0, 60.0), (600.0, 35.0)]),
    )
    kw = dict(u0=plant.steady_input(30.0))
    naive = simulate(plant, _pi_on_tank(plant, scenario, Tt=np.inf, **kw), scenario)
    aw = simulate(plant, _pi_on_tank(plant, scenario, **kw), scenario)

    recovery = (600.0, 2400.0)
    assert compute_metrics(aw, recovery)["IAE"] < 0.2 * compute_metrics(naive, recovery)["IAE"]


def test_derivative_acts_on_the_measurement_not_the_error():
    """A setpoint step must not produce a derivative impulse ('derivative kick')."""
    ctrl = PIDController(Kc=1.0, Ti=100.0, Td=10.0, dt=1.0, u0=0.0)
    ctrl.reset()
    y = np.array([50.0])
    ctrl.compute(y, np.array([50.0]), 0.0)
    u_step = ctrl.compute(y, np.array([60.0]), 1.0)[0]   # setpoint jumps, y does not
    assert u_step == pytest.approx(10.0, abs=1e-6)       # proportional only, Kc * 10


def test_derivative_filter_limits_noise_amplification():
    """Unfiltered derivative would multiply a one-sample blip by Kc*Td/dt = 100;
    the Td/N filter must keep it far below that."""
    ctrl = PIDController(Kc=1.0, Ti=None, Td=10.0, dt=1.0, N=10.0, u0=0.0)
    ctrl.reset()
    ctrl.compute(np.array([50.0]), np.array([50.0]), 0.0)
    u = ctrl.compute(np.array([51.0]), np.array([50.0]), 1.0)[0]
    assert abs(u) < 10.0


def test_reset_clears_controller_state():
    ctrl = PIDController(Kc=1.0, Ti=10.0, dt=1.0, u0=0.0)
    for k in range(50):
        ctrl.compute(np.array([40.0]), np.array([50.0]), float(k))
    wound_up = ctrl.compute(np.array([40.0]), np.array([50.0]), 50.0)[0]
    ctrl.reset()
    fresh = ctrl.compute(np.array([40.0]), np.array([50.0]), 0.0)[0]
    assert fresh < wound_up
    assert fresh == pytest.approx(0.0, abs=1e-9)   # u0 = 0 bumpless start

"""The plants added after phase 1's first pass: integrating, high-order,
inverse-response and cascade processes, plus the delay machinery they use."""

import numpy as np
import pytest

from process_control.plants.base import Plant
from process_control.plants.cascade_process import CascadeProcess
from process_control.plants.integrating_tank import IntegratingTank
from process_control.plants.inverse_response import InverseResponseTank
from process_control.plants.series_tanks import SeriesTanks
from process_control.plants.tank import Tank


def _run(plant, u, n, dt=1.0, d=0.0):
    ys = [plant.reset()]
    u = np.atleast_1d(np.asarray(u, float))
    d = np.atleast_1d(np.asarray(d, float))
    for _ in range(n):
        ys.append(plant.step(u, dt, d))
    return np.array(ys)


# ----------------------------------------------------------------------
# integrating process
# ----------------------------------------------------------------------
def test_integrating_tank_ramps_and_never_self_regulates():
    plant = IntegratingTank(k_prime=0.02, u_bias=50.0, theta=0.0, h0=50.0, noise_std=0.0)
    y = _run(plant, 60.0, 500).ravel()
    assert y[100] == pytest.approx(50.0 + 0.02 * 10 * 100, abs=1e-6)
    assert y[500] == pytest.approx(50.0 + 0.02 * 10 * 500, abs=1e-6)   # still ramping


def test_integrating_tank_holds_at_the_balance_point():
    plant = IntegratingTank(k_prime=0.02, u_bias=50.0, h0=50.0, noise_std=0.0)
    y = _run(plant, 50.0, 500).ravel()
    assert np.allclose(y, 50.0, atol=1e-9)


def test_integrating_tank_any_level_is_held_by_the_same_valve():
    plant = IntegratingTank(u_bias=50.0)
    assert plant.steady_input(20.0) == plant.steady_input(80.0) == 50.0


# ----------------------------------------------------------------------
# high-order process
# ----------------------------------------------------------------------
def test_series_tanks_steady_state_gain():
    plant = SeriesTanks(K=2.0, taus=(30.0, 15.0, 5.0), noise_std=0.0)
    y = _run(plant, 10.0, 4000).ravel()
    assert y[-1] == pytest.approx(20.0, rel=1e-4)


def test_series_tanks_step_response_starts_flat():
    """Three lags in series: the output has zero initial slope, which is what a
    first-order model cannot represent and what the half rule converts into
    apparent dead time."""
    plant = SeriesTanks(K=1.0, taus=(30.0, 15.0, 5.0), noise_std=0.0)
    y = _run(plant, 10.0, 60).ravel()
    assert y[1] < 0.02 * y[-1]                    # essentially nothing yet
    assert y[-1] > 0.2                            # but it does move eventually


def test_series_tanks_half_rule_reduction():
    plant = SeriesTanks(K=1.0, taus=(40.0, 20.0, 10.0), theta=3.0)
    m = plant.fopdt_half_rule()
    assert m["tau"] == pytest.approx(50.0)        # 40 + 20/2
    assert m["theta"] == pytest.approx(23.0)      # 3 + 20/2 + 10


def test_series_tanks_reduction_accounts_for_sampling():
    plant = SeriesTanks(K=1.0, taus=(40.0,), theta=0.0)
    assert plant.fopdt_half_rule(dt=2.0)["theta"] == pytest.approx(1.0)


# ----------------------------------------------------------------------
# inverse response
# ----------------------------------------------------------------------
def test_inverse_response_moves_the_wrong_way_first():
    plant = InverseResponseTank(K_slow=1.5, tau_slow=60.0, K_fast=-0.5, tau_fast=5.0,
                                theta=0.0, noise_std=0.0)
    y = _run(plant, 10.0, 600).ravel()
    assert y.min() < -0.5                          # dips below where it started
    assert y[-1] == pytest.approx(10.0 * plant.K, rel=1e-3)   # ends the right way
    assert np.argmin(y) < 60                       # and the dip is early


def test_no_inverse_response_when_the_fast_path_is_absent():
    plant = InverseResponseTank(K_slow=1.0, K_fast=0.0, noise_std=0.0)
    assert not plant.has_inverse_response
    assert plant.zero < 0                          # ordinary left-half-plane zero
    y = _run(plant, 10.0, 300).ravel()
    assert y.min() >= -1e-9


def test_rhp_zero_grows_with_the_opposing_path():
    mild = InverseResponseTank(K_slow=1.1, K_fast=-0.1)
    severe = InverseResponseTank(K_slow=1.8, K_fast=-0.8)
    assert 0 < mild.zero < severe.zero


def test_inverse_response_reduction_puts_the_zero_into_the_dead_time():
    plant = InverseResponseTank(K_slow=1.5, tau_slow=60.0, K_fast=-0.5, tau_fast=5.0, theta=2.0)
    m = plant.fopdt_half_rule()
    assert m["theta"] > plant.zero                 # the zero is in there
    assert m["K"] == pytest.approx(1.0)


# ----------------------------------------------------------------------
# cascade process
# ----------------------------------------------------------------------
def test_cascade_secondary_responds_before_the_primary():
    plant = CascadeProcess(tau1=5.0, tau2=60.0, theta=0.0, y_dead_time=(0.0, 0.0),
                           noise_std=(0.0, 0.0))
    y = _run(plant, 50.0, 30)
    primary, secondary = y[:, 0], y[:, 1]
    assert secondary[10] > 20.0                    # flow is already most of the way
    assert primary[10] < 0.2 * secondary[10]       # temperature has barely moved


def test_cascade_output_order_is_primary_then_secondary():
    plant = CascadeProcess(K1=1.0, K2=1.2, theta=0.0, y_dead_time=(0.0, 0.0),
                           noise_std=(0.0, 0.0))
    y = _run(plant, 50.0, 5000)
    assert y[-1, 1] == pytest.approx(50.0, rel=1e-3)            # x1 = K1*u
    assert y[-1, 0] == pytest.approx(1.2 * 50.0, rel=1e-3)      # x2 = K2*x1


def test_cascade_inner_disturbance_moves_the_secondary_first():
    """An upset in the inner stage shows up in the flow almost immediately and
    in the temperature hardly at all -- which is the information a cascade buys."""
    kw = dict(theta=0.0, y_dead_time=(0.0, 0.0), noise_std=(0.0, 0.0))
    quiet = _run(CascadeProcess(**kw), 50.0, 20)
    upset = _run(CascadeProcess(**kw), 50.0, 20, d=[-20.0, 0.0])

    secondary_shift = quiet[10, 1] - upset[10, 1]
    primary_shift = quiet[10, 0] - upset[10, 0]
    assert secondary_shift > 5 * primary_shift > 0


# ----------------------------------------------------------------------
# the three delay paths
# ----------------------------------------------------------------------
def test_measurement_dead_time_delays_only_the_reported_value():
    fast = Tank(K=1.5, tau=60.0, theta=0.0, h0=0.0, noise_std=0.0)
    slow = Tank(K=1.5, tau=60.0, theta=0.0, h0=0.0, noise_std=0.0)
    slow.y_dead_time = np.array([10.0])            # sensor reports 10 s late

    a = _run(fast, 20.0, 40).ravel()
    b = _run(slow, 20.0, 40).ravel()
    assert np.allclose(b[10:], a[:31], atol=1e-9)   # reported 10 samples late
    assert np.allclose(b[:10], 0.0, atol=1e-9)      # nothing reported yet


def test_disturbance_dead_time_delays_the_upset():
    plant = Tank(K=1.5, tau=60.0, theta=0.0, theta_d=30.0, h0=30.0, noise_std=0.0)
    y = _run(plant, plant.steady_input(30.0), 60, d=-12.0).ravel()
    assert np.allclose(y[:31], 30.0, atol=1e-9)
    assert y[40] < 30.0


def test_the_three_delays_are_independent():
    """Process, disturbance and measurement dead times are separate mechanisms
    and must compose without interfering."""
    plant = Tank(K=1.5, tau=60.0, theta=5.0, theta_d=20.0, h0=30.0, noise_std=0.0)
    plant.y_dead_time = np.array([7.0])
    y = _run(plant, 40.0, 80, d=-12.0).ravel()
    assert np.allclose(y[:13], 30.0, atol=1e-9)    # 5 s process + 7 s sensor
    assert y[20] > 30.0                            # valve effect arrives first


def test_negative_dead_times_are_rejected():
    for kwargs in ({"dead_time": -1.0}, {"d_dead_time": -1.0}, {"y_dead_time": -1.0}):
        with pytest.raises(ValueError):
            class _P(Plant):
                def dynamics(self, x, u, d):
                    return np.zeros(1)

            _P(n_states=1, n_inputs=1, n_outputs=1, u_min=0, u_max=1, x0=[0], **kwargs)

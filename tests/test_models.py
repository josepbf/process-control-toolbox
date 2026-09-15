"""The controller's internal model: is it the process, and does it admit where it isn't."""

import numpy as np
import pytest

from process_control.controllers.base import Controller
from process_control.harness.scenarios import Scenario, constant
from process_control.harness.simulate import simulate
from process_control.models.discrete import (
    DiscreteModel,
    append_disturbance_path,
    fopdt_model,
    integrating_model,
    series_lags_model,
    zoh,
)
from process_control.plants.integrating_tank import IntegratingTank
from process_control.plants.series_tanks import SeriesTanks
from process_control.plants.tank import Tank


class ConstantController(Controller):
    """Open-loop reference, as in test_harness: ignores the measurement."""

    def __init__(self, u):
        self.u = float(u)
        self.name = "constant"
        self.tuning_note = "open loop"

    def compute(self, y, setpoint, t):
        return np.array([self.u])

    def reset(self):
        pass


def _step_response(model: DiscreteModel, u: float, n: int) -> np.ndarray:
    """Output after each of n unit-held moves, starting from rest."""
    _, Y = model.rollout(np.zeros(model.n_states), np.full((n, 1), float(u)))
    return Y[:, 0]


# ----------------------------------------------------------------------
# discretisation
# ----------------------------------------------------------------------
def test_discretised_fopdt_matches_the_analytic_step_response():
    """Exact ZOH, so the agreement is to machine precision, not to a tolerance
    that quietly hides a first-order discretisation."""
    K, tau, dt, n = 1.5, 60.0, 1.0, 300
    y = _step_response(fopdt_model(K, tau, theta=0.0, dt=dt), u=1.0, n=n)
    t = np.arange(n + 1) * dt
    assert np.allclose(y, K * (1.0 - np.exp(-t / tau)), atol=1e-12)


def test_a_coarse_sample_time_does_not_degrade_the_discretisation():
    """dt = tau/2 is well past where Euler would be visibly wrong; exact ZOH
    does not care, and the dead-time sweeps rely on that."""
    K, tau, dt, n = 1.5, 60.0, 30.0, 20
    y = _step_response(fopdt_model(K, tau, theta=0.0, dt=dt), u=1.0, n=n)
    t = np.arange(n + 1) * dt
    assert np.allclose(y, K * (1.0 - np.exp(-t / tau)), atol=1e-12)


def test_zoh_of_an_integrator_is_exact():
    Ad, Bd = zoh([[0.0]], [[0.02]], dt=1.0)
    assert Ad[0, 0] == pytest.approx(1.0)
    assert Bd[0, 0] == pytest.approx(0.02)


def test_dc_gain_equals_the_process_gain():
    model = fopdt_model(K=1.5, tau=60.0, theta=15.0, dt=1.0)
    assert model.dc_gain()[0, 0] == pytest.approx(1.5, abs=1e-9)


def test_an_integrating_model_reports_an_infinite_steady_state_gain():
    """Not an error: a non-self-regulating process genuinely has no gain, which
    is why it needs its own tuning rules."""
    assert np.isinf(integrating_model(k_prime=0.02, theta=10.0, dt=1.0).dc_gain()[0, 0])


# ----------------------------------------------------------------------
# dead time
# ----------------------------------------------------------------------
def test_dead_time_augmentation_delays_the_response_by_exactly_theta():
    theta, dt = 15.0, 1.0
    model = fopdt_model(K=1.5, tau=60.0, theta=theta, dt=dt)
    y = _step_response(model, u=1.0, n=60)

    n_delay = int(theta / dt)
    assert model.n_states == 1 + n_delay
    assert np.allclose(y[: n_delay + 1], 0.0, atol=1e-12)     # nothing arrives early
    assert y[n_delay + 1] > 0.0                               # and nothing arrives late


def test_the_delayed_response_is_the_undelayed_one_shifted():
    """The augmentation must add dead time and change nothing else."""
    plain = _step_response(fopdt_model(1.5, 60.0, theta=0.0, dt=1.0), 1.0, 80)
    delayed = _step_response(fopdt_model(1.5, 60.0, theta=15.0, dt=1.0), 1.0, 80)
    assert np.allclose(delayed[15:], plain[: 80 - 15 + 1], atol=1e-12)


def test_a_non_integer_dead_time_is_rounded_and_the_model_says_so():
    """The plant rounds theta/dt to whole samples, so the model must round the
    same way -- and must declare the residual, because an undeclared mismatch is
    the kind that silently flatters a model-based controller."""
    model = fopdt_model(K=1.5, tau=60.0, theta=15.4, dt=1.0)
    assert model.n_delay == (15,)
    assert model.theta_requested == (15.4,)
    assert model.theta_realised == (15.0,)
    assert model.describe()["theta_realised"] == [15.0]
    assert "rounded" in model.note and "0.4" in model.note


def test_an_exact_dead_time_adds_no_rounding_note():
    assert "rounded" not in fopdt_model(1.5, 60.0, theta=15.0, dt=1.0).note


def test_zero_dead_time_adds_no_states():
    assert fopdt_model(1.5, 60.0, theta=0.0, dt=1.0).n_states == 1


def test_a_negative_dead_time_is_refused():
    with pytest.raises(ValueError, match="negative"):
        fopdt_model(1.5, 60.0, theta=-1.0, dt=1.0)


# ----------------------------------------------------------------------
# the model against the plant it claims to describe
# ----------------------------------------------------------------------
def test_the_model_agrees_with_the_plant_it_claims_to_describe():
    """The strongest test of the discretisation: run the real plant open loop
    through the real harness, and roll the model out over the same moves.

    They are not the same computation -- the plant is fixed-step RK4 with four
    substeps, the model is an exact matrix exponential -- so they agree to
    integration error, not to machine precision. A gap larger than that means
    the model is describing a different process.
    """
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=0.0, noise_std=0.0, u_min=-1e9, u_max=1e9)
    scenario = Scenario("open loop", dt=1.0, t_final=400.0, setpoint=constant(0.0))
    u = 20.0
    df = simulate(plant, ConstantController(u), scenario)

    model = fopdt_model(**plant.fopdt, dt=scenario.dt)
    y_model = _step_response(model, u, scenario.n_steps)

    # The plant primes its delay line with the input that holds h0 = 0, i.e. 0,
    # so both start from rest and the comparison is like for like.
    assert np.max(np.abs(df["y"].to_numpy(float) - y_model)) < 1e-3 * abs(1.5 * u)


def test_the_series_lags_model_agrees_with_the_series_tanks_plant():
    plant = SeriesTanks(K=1.0, taus=(40.0, 20.0, 10.0), theta=0.0, noise_std=0.0)
    scenario = Scenario("open loop", dt=1.0, t_final=500.0, setpoint=constant(0.0))
    u = 30.0
    df = simulate(plant, ConstantController(u), scenario)

    model = series_lags_model(K=plant.K, taus=plant.taus, theta=plant.theta, dt=scenario.dt)
    y_model = _step_response(model, u, scenario.n_steps)
    assert np.max(np.abs(df["y"].to_numpy(float) - y_model)) < 1e-3 * abs(plant.K * u)


def test_the_integrating_model_agrees_with_the_integrating_tank():
    plant = IntegratingTank(k_prime=0.02, u_bias=50.0, theta=10.0, h0=50.0, noise_std=0.0)
    scenario = Scenario("open loop", dt=1.0, t_final=300.0, setpoint=constant(0.0))
    u = 60.0
    df = simulate(plant, ConstantController(u), scenario)

    # The plant integrates the deviation from its bias, so the model is driven
    # by the same deviation. Starting level is carried as the initial state.
    model = integrating_model(k_prime=plant.k_prime, theta=plant.theta, dt=scenario.dt)
    x0 = np.zeros(model.n_states)
    x0[0] = 50.0
    _, Y = model.rollout(x0, np.full((scenario.n_steps, 1), u - plant.u_bias))
    assert np.max(np.abs(df["y"].to_numpy(float) - Y[:, 0])) < 1e-6


# ----------------------------------------------------------------------
# the disturbance path
# ----------------------------------------------------------------------
def test_a_measured_disturbance_path_reproduces_the_tank_load_response():
    """Tank sends the load down its own first-order path with its own gain and
    delay, and sums at the output. The model has to do the same, or a
    model-based feedforward is compensating for a process that does not exist.
    """
    plant = Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, theta_d=45.0,
                 h0=0.0, noise_std=0.0, u_min=-1e9, u_max=1e9)
    scenario = Scenario(
        "load", dt=1.0, t_final=400.0, setpoint=constant(0.0), disturbance=constant(-12.0)
    )
    df = simulate(plant, ConstantController(0.0), scenario)

    model = append_disturbance_path(
        fopdt_model(**plant.fopdt, dt=scenario.dt),
        Kd=plant.Kd, tau_d=plant.tau_d, theta_d=plant.theta_d,
    )
    _, Y = model.rollout(
        np.zeros(model.n_states),
        np.zeros((scenario.n_steps, 1)),
        np.full((scenario.n_steps, 1), -12.0),
    )
    assert np.max(np.abs(df["y"].to_numpy(float) - Y[:, 0])) < 1e-3 * 12.0


def test_a_model_without_a_disturbance_path_refuses_a_disturbance():
    model = fopdt_model(1.5, 60.0, theta=0.0, dt=1.0)
    with pytest.raises(ValueError, match="no disturbance path"):
        model.step(np.zeros(1), np.zeros(1), d=np.array([1.0]))


def test_a_second_disturbance_path_is_refused():
    model = append_disturbance_path(fopdt_model(1.5, 60.0, dt=1.0), Kd=1.0, tau_d=60.0)
    with pytest.raises(ValueError, match="already carries"):
        append_disturbance_path(model, Kd=1.0, tau_d=60.0)


# ----------------------------------------------------------------------
# shape and bookkeeping
# ----------------------------------------------------------------------
def test_delay_states_come_after_the_process_states():
    """A convention the observer, the prediction matrices and the tests all
    index on, so it is pinned here rather than assumed."""
    model = fopdt_model(K=1.5, tau=60.0, theta=5.0, dt=1.0)
    assert model.C.shape == (1, 6)
    assert model.C[0, 0] == 1.0
    assert np.allclose(model.C[0, 1:], 0.0)      # the queue is not the output


def test_a_mis_shaped_model_is_refused_at_construction():
    with pytest.raises(ValueError, match="square"):
        DiscreteModel(A=np.zeros((2, 3)), B=np.zeros((2, 1)), C=np.zeros((1, 2)), dt=1.0)
    with pytest.raises(ValueError, match="columns"):
        DiscreteModel(A=np.eye(2), B=np.zeros((2, 1)), C=np.zeros((1, 3)), dt=1.0)
    with pytest.raises(ValueError, match="dt must be positive"):
        DiscreteModel(A=np.eye(1), B=np.zeros((1, 1)), C=np.zeros((1, 1)), dt=0.0)


def test_the_model_record_carries_what_a_reader_needs_to_judge_it():
    record = fopdt_model(K=1.5, tau=60.0, theta=15.0, dt=1.0).describe()
    assert record["n_states"] == 16
    assert record["dt"] == 1.0
    assert record["theta_realised"] == [15.0]
    assert "FOPDT" in record["note"]

"""The estimator, and the disturbance state that is a model-based law's integral action."""

import numpy as np
import pytest

from process_control.models.discrete import fopdt_model, integrating_model
from process_control.models.observer import KalmanObserver, augment_disturbance
from process_control.plants.tank import Tank


def _observer(theta=15.0, kind="output", **kw):
    model = augment_disturbance(fopdt_model(K=1.5, tau=60.0, theta=theta, dt=1.0), kind)
    return model, KalmanObserver(model, **kw)


def _run_open_loop(plant, obs, u, d, n_steps, seed=0):
    """Drive the real plant, feed the observer the measurement and the move.

    The observer never sees the plant state -- only what a controller would see,
    in the order a controller would see it.
    """
    y = plant.reset(seed=seed)
    obs.seed_from_measurement(y, u0=u)
    trace = []
    for _ in range(n_steps):
        obs.correct(y)                       # this sample's measurement, first
        trace.append(obs.d_hat.copy())
        obs.predict(np.array([u]))           # then the move that is about to be held
        y = plant.step(np.array([u]), 1.0, np.array([d]))
    return y, np.array(trace)


# ----------------------------------------------------------------------
# convergence
# ----------------------------------------------------------------------
def test_the_observer_converges_to_the_true_state_from_a_wrong_start():
    model, obs = _observer(theta=0.0)
    x_true = np.array([42.0, 0.0])
    obs.reset(np.array([0.0, 0.0]))          # deliberately wrong by the whole level

    u = np.array([28.0])
    for _ in range(1500):
        obs.correct(model.C @ x_true)
        obs.predict(u)
        x_true = model.step(x_true, u)

    assert np.allclose(obs.x_hat, x_true, atol=1e-4)


def test_the_innovation_dies_away_once_the_estimate_has_caught_up():
    model, obs = _observer(theta=0.0)
    x_true = np.array([42.0, 0.0])
    first = None
    for k in range(1500):
        obs.correct(model.C @ x_true)
        if k == 0:
            first = abs(float(obs.innovation[0]))
        obs.predict(np.array([28.0]))
        x_true = model.step(x_true, np.array([28.0]))
    assert first > 10.0
    assert abs(float(obs.innovation[0])) < 1e-4


# ----------------------------------------------------------------------
# the disturbance state
# ----------------------------------------------------------------------
def test_the_disturbance_state_absorbs_an_unmeasured_load_step():
    """An unmeasured load of -12 pushes the level 12 below where the model says
    it should be. That gap has to end up somewhere, and the whole point of the
    disturbance state is that it ends up there rather than in the tracking
    error."""
    plant = Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=45.0, noise_std=0.0)
    _, obs = _observer(theta=15.0)

    y_end, trace = _run_open_loop(plant, obs, u=30.0, d=-12.0, n_steps=1200)

    assert float(obs.d_hat[0]) == pytest.approx(-12.0, abs=1e-2)
    assert abs(trace[0, 0]) < 1e-6                 # it starts knowing nothing
    assert float((obs.model.C @ obs.x_hat)[0]) == pytest.approx(float(y_end[0]), abs=1e-2)


def test_a_faster_disturbance_estimate_is_the_shorter_integral_time():
    """q_disturbance / r_noise is the tuning knob, and it behaves like 1/Ti:
    raise it and the estimate chases the load faster."""
    plant_kw = dict(K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=45.0, noise_std=0.0)

    _, slow = _observer(q_disturbance=1e-4, r_noise=1e-2)
    _, fast = _observer(q_disturbance=1e-0, r_noise=1e-2)
    _, slow_trace = _run_open_loop(Tank(**plant_kw), slow, u=30.0, d=-12.0, n_steps=200)
    _, fast_trace = _run_open_loop(Tank(**plant_kw), fast, u=30.0, d=-12.0, n_steps=200)

    # Same target, so "faster" means further along at the same sample.
    assert abs(fast_trace[-1, 0]) > abs(slow_trace[-1, 0])


def test_without_a_disturbance_state_the_innovation_never_settles_to_zero():
    """The complement of the claim above, and the reason a model-based law needs
    the augmentation.

    Without a disturbance state the observer still pulls its estimate toward the
    measurement -- it has a gain on the process state -- so the estimate is not
    simply wrong. What it cannot do is be consistent with the measurement *and*
    with the model at the same time, because there is nowhere to put the
    difference. So a standing innovation remains: the model keeps predicting a
    level the load has moved away from, every sample, for ever. That leftover is
    exactly what the disturbance state exists to absorb, and it is why a
    controller built on this observer would sit on an offset.
    """
    plant_kw = dict(K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=45.0, noise_std=0.0)
    base = fopdt_model(K=1.5, tau=60.0, theta=15.0, dt=1.0)

    plain = KalmanObserver(base)
    assert plain.d_hat.size == 0
    _run_open_loop(Tank(**plant_kw), plain, u=30.0, d=-12.0, n_steps=1200)

    _, augmented = _observer(theta=15.0)
    _run_open_loop(Tank(**plant_kw), augmented, u=30.0, d=-12.0, n_steps=1200)

    assert abs(float(plain.innovation[0])) > 1.0        # a standing disagreement
    assert abs(float(augmented.innovation[0])) < 1e-6   # fully reconciled


def test_an_input_disturbance_model_also_reaches_the_right_output():
    """Both augmentations are offset-free; they differ in *where* they believe
    the upset entered, which shows up in the transient, not the steady state."""
    plant = Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=45.0, noise_std=0.0)
    _, obs = _observer(theta=15.0, kind="input")
    y_end, _ = _run_open_loop(plant, obs, u=30.0, d=-12.0, n_steps=1500)
    assert float((obs.model.C @ obs.x_hat)[0]) == pytest.approx(float(y_end[0]), abs=1e-2)


# ----------------------------------------------------------------------
# detectability
# ----------------------------------------------------------------------
def test_an_undetectable_disturbance_augmentation_is_refused():
    """An integrating process with an output-disturbance model has two
    integrators that look identical from the measurement. No observer can
    separate them, so the model is refused rather than quietly misbehaving."""
    with pytest.raises(ValueError, match="not detectable"):
        augment_disturbance(integrating_model(k_prime=0.02, theta=10.0, dt=1.0), "output")


def test_an_integrating_process_takes_the_input_disturbance_model():
    model = augment_disturbance(integrating_model(k_prime=0.02, theta=10.0, dt=1.0), "input")
    assert model.n_disturbance_states == 1
    assert model.disturbance_kind == "input"
    KalmanObserver(model)                     # the Riccati solve must succeed too


def test_augmenting_twice_is_refused():
    model = augment_disturbance(fopdt_model(1.5, 60.0, theta=0.0, dt=1.0), "output")
    with pytest.raises(ValueError, match="already carries"):
        augment_disturbance(model, "output")


def test_an_unknown_disturbance_kind_is_refused():
    with pytest.raises(ValueError, match="output.*input"):
        augment_disturbance(fopdt_model(1.5, 60.0, dt=1.0), "sideways")


# ----------------------------------------------------------------------
# timing and state
# ----------------------------------------------------------------------
def test_the_correction_uses_this_sample_s_measurement_not_the_last_one():
    """The harness hands the controller y(t) before it asks for u(t), so the
    estimate must already contain y(t) when the move is chosen. A predictor-form
    observer passes nearly every other test in this file and quietly costs a
    whole sample of dead time -- on these loops, the one thing that is scarce."""
    model, obs = _observer(theta=0.0)
    obs.reset(np.zeros(model.n_states))

    before = obs.x_hat.copy()
    returned = obs.correct(np.array([50.0]))

    assert not np.allclose(returned, before)          # it moved on this sample
    assert np.allclose(returned, obs.x_hat)           # and correct() returns x(k|k)
    assert float(obs.innovation[0]) == pytest.approx(50.0)


def test_seeding_from_the_first_measurement_starts_without_a_kick():
    """Bumpless start: the estimate begins consistent with what was measured, so
    the first innovation is zero rather than the whole operating point."""
    model, obs = _observer(theta=15.0)
    obs.seed_from_measurement(np.array([45.0]), u0=30.0)

    assert float((model.C @ obs.x_hat)[0]) == pytest.approx(45.0)
    obs.correct(np.array([45.0]))
    assert abs(float(obs.innovation[0])) < 1e-9
    assert float(obs.d_hat[0]) == pytest.approx(0.0)


def test_the_seeded_delay_queue_holds_the_starting_move():
    model, obs = _observer(theta=5.0)
    obs.seed_from_measurement(np.array([45.0]), u0=30.0)
    # process state, then five delay states, then the disturbance state
    assert np.allclose(obs.x_hat[1:6], 30.0)
    assert obs.x_hat[-1] == pytest.approx(0.0)


def test_reset_clears_the_estimate_completely():
    """Two runs of the same controller must be identical, and a leftover
    disturbance estimate is the easiest piece to forget."""
    model, obs = _observer(theta=0.0)
    for _ in range(50):
        obs.correct(np.array([80.0]))
        obs.predict(np.array([10.0]))
    assert abs(float(obs.d_hat[0])) > 1e-6

    obs.reset()
    assert np.allclose(obs.x_hat, 0.0)
    assert np.allclose(obs.innovation, 0.0)


def test_an_explicit_gain_skips_the_riccati_solve():
    model = augment_disturbance(fopdt_model(1.5, 60.0, theta=0.0, dt=1.0), "output")
    L = np.array([[0.5], [0.1]])
    obs = KalmanObserver(model, gain=L)
    assert np.allclose(obs.L, L)
    assert "explicit" in obs.describe()["gain"]


def test_a_mis_shaped_gain_is_refused():
    model = augment_disturbance(fopdt_model(1.5, 60.0, theta=0.0, dt=1.0), "output")
    with pytest.raises(ValueError, match="shape"):
        KalmanObserver(model, gain=np.zeros((3, 1)))


def test_the_observer_record_names_its_tuning():
    """Fairness rule 1 reaches the observer too: the ratio that sets the
    integral action has to be readable from the run record."""
    _, obs = _observer()
    record = obs.describe()
    assert record["disturbance_kind"] == "output"
    assert record["n_disturbance_states"] == 1
    assert "q_disturbance" in record["gain"] and "integral action" in record["gain"]

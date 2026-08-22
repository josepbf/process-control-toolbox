"""The plant is the ground truth, so its physics has to be right first."""

import numpy as np
import pytest

from process_control.plants.tank import Tank


def _run(plant, u, n, dt=1.0, d=0.0):
    ys = [plant.reset()]
    for _ in range(n):
        ys.append(plant.step(np.array([u]), dt, np.array([d])))
    return np.array(ys).ravel()


def test_steady_state_gain():
    """Holding u forever must give y = K*u, independent of tau and theta."""
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=0.0, noise_std=0.0)
    y = _run(plant, u=40.0, n=3000)
    assert y[-1] == pytest.approx(1.5 * 40.0, rel=1e-6)


def test_disturbance_gain():
    plant = Tank(K=1.5, tau=60.0, theta=0.0, Kd=1.0, h0=0.0, noise_std=0.0)
    y = _run(plant, u=0.0, n=3000, d=-12.0)
    assert y[-1] == pytest.approx(-12.0, rel=1e-6)


def test_dead_time_delays_the_response_exactly():
    """Nothing may happen before t = theta; something must happen right after."""
    theta, dt = 10.0, 1.0
    plant = Tank(K=1.0, tau=10.0, theta=theta, h0=0.0, noise_std=0.0)
    plant.u_init = np.zeros(1)  # empty pipeline, so the step is the only input
    y = _run(plant, u=50.0, n=20, dt=dt)

    assert np.allclose(y[: int(theta / dt) + 1], 0.0, atol=1e-12)
    assert y[int(theta / dt) + 1] > 0.0


def test_first_order_time_constant():
    """After one tau the step response must have covered 63.2 %% of its span."""
    plant = Tank(K=1.0, tau=60.0, theta=0.0, h0=0.0, noise_std=0.0)
    y = _run(plant, u=10.0, n=60)
    assert y[60] == pytest.approx(10.0 * (1 - np.exp(-1.0)), rel=1e-4)


def test_saturation_is_enforced_by_the_plant():
    """A controller may ask for anything; the actuator range is the plant's."""
    plant = Tank(K=1.5, tau=60.0, theta=0.0, h0=0.0, u_min=0.0, u_max=100.0, noise_std=0.0)
    y = _run(plant, u=500.0, n=3000)
    assert y[-1] == pytest.approx(1.5 * 100.0, rel=1e-6)


def test_reset_reproduces_the_noise_realisation():
    """Fairness rule 2: two controllers on the same plant see the same noise."""
    plant = Tank(noise_std=0.5, seed=3)
    a = _run(plant, u=25.0, n=50)
    b = _run(plant, u=25.0, n=50)
    assert np.array_equal(a, b)


def test_noise_is_actually_applied():
    plant = Tank(noise_std=0.5, seed=3)
    y = _run(plant, u=20.0, n=200)
    assert np.std(np.diff(y)) > 0.1


def test_changing_dt_midrun_is_rejected():
    """The delay line is a fixed-length FIFO; a changing dt would silently
    change the dead time, so it is an error rather than a surprise."""
    plant = Tank(theta=10.0)
    plant.reset()
    plant.step(np.array([20.0]), 1.0)
    with pytest.raises(ValueError):
        plant.step(np.array([20.0]), 2.0)

"""Metric definitions, checked on signals whose answer is known by hand."""

import numpy as np
import pandas as pd
import pytest

from src.harness.metrics import (
    iae,
    ise,
    itae,
    noise_sigma,
    overshoot,
    peak_deviation,
    reversals,
    settling_time,
    steady_state_offset,
    summarize,
    total_variation,
)


def test_iae_ise_itae_on_a_constant_error():
    t = np.linspace(0.0, 10.0, 11)
    e = np.ones_like(t)
    assert iae(t, e) == pytest.approx(10.0)
    assert ise(t, e) == pytest.approx(10.0)
    assert itae(t, e) == pytest.approx(50.0)      # integral of t dt over [0,10]


def test_iae_ignores_the_sign_of_the_error():
    t = np.linspace(0.0, 10.0, 11)
    assert iae(t, -np.ones_like(t)) == pytest.approx(10.0)


def test_total_variation_and_reversals():
    u = np.array([0.0, 1.0, 0.0, 1.0])
    assert total_variation(u) == pytest.approx(3.0)
    assert reversals(u) == 2                      # +,-,+ -> two direction changes
    assert reversals(np.array([0.0, 1.0, 2.0, 3.0])) == 0   # monotone ramp


def test_reversals_ignores_dither_below_the_threshold():
    u = np.array([0.0, 0.01, 0.0, 0.01, 0.0])
    assert reversals(u, eps=0.5) == 0
    assert reversals(u, eps=1e-9) == 3


def test_settling_time_on_a_clean_step():
    t = np.arange(0.0, 100.0)
    y = np.where(t < 20.0, 0.0, 10.0)
    sp = np.full_like(t, 10.0)
    assert settling_time(t, y, sp, step_size=10.0) == pytest.approx(20.0)


def test_settling_time_is_nan_if_never_settled():
    """A limit cycle of the shape a real lagged process produces: many samples
    per period, so the noise estimator sees it as structure, not noise."""
    t = np.arange(0.0, 600.0)
    y = 10.0 + 5.0 * np.sin(2 * np.pi * t / 60.0)
    sp = np.full_like(t, 10.0)
    assert np.isnan(settling_time(t, y, sp, step_size=10.0))


def test_settling_time_tolerates_measurement_noise():
    """A loop that reaches setpoint at t=20 and then only wobbles on noise must
    be reported as settled at 20 s, not as still moving at the end of the run."""
    rng = np.random.default_rng(0)
    t = np.arange(0.0, 2000.0)
    y = np.where(t < 20.0, 0.0, 10.0) + rng.normal(0.0, 0.15, size=t.size)
    sp = np.full_like(t, 10.0)
    assert settling_time(t, y, sp, step_size=10.0) == pytest.approx(20.0, abs=1.0)


def test_overshoot_is_undefined_without_a_setpoint_change():
    t = np.arange(0.0, 100.0)
    y = 10.0 + np.where(t == 50.0, -4.0, 0.0)     # pure disturbance dip
    sp = np.full_like(t, 10.0)
    assert np.isnan(overshoot(y, sp))
    assert peak_deviation(y, sp) == pytest.approx(4.0)


def test_overshoot_on_a_step_with_a_known_peak():
    y = np.array([0.0, 12.0, 10.0, 10.0])
    sp = np.full(4, 10.0)
    assert overshoot(y, sp, y_start=0.0) == pytest.approx(20.0)


def test_steady_state_offset():
    t = np.arange(0.0, 100.0)
    y = np.full_like(t, 9.5)
    sp = np.full_like(t, 10.0)
    assert steady_state_offset(t, y, sp) == pytest.approx(0.5)


def test_noise_sigma_recovers_the_injected_noise():
    rng = np.random.default_rng(0)
    y = 50.0 + rng.normal(0.0, 0.3, size=5000)
    assert noise_sigma(y) == pytest.approx(0.3, rel=0.1)


def test_noise_sigma_is_not_fooled_by_a_ramp():
    """A smooth trend contributes a constant offset to the differences, which
    the median removes -- so the estimate stays near zero noise."""
    y = np.arange(1000.0) * 0.1
    assert noise_sigma(y) < 1e-9


def _fake_run(u):
    n = len(u)
    df = pd.DataFrame(
        {
            "t": np.arange(float(n)),
            "y": np.full(n, 10.0),
            "sp": np.full(n, 10.0),
            "u": u,
            "d": np.zeros(n),
            "violation": np.zeros(n),
            "violated": np.zeros(n, dtype=bool),
            "solve_time": np.full(n, 1e-4),
        }
    )
    df.attrs.update({"u_min": [0.0], "u_max": [100.0], "tuning": "n/a"})
    return df


def test_summarize_builds_a_row_per_controller_and_window():
    runs = {"a": _fake_run(np.zeros(50)), "b": _fake_run(np.ones(50))}
    table = summarize(runs, windows={"late": (25.0, 50.0)})
    assert set(table.index) == {("a", "full"), ("a", "late"), ("b", "full"), ("b", "late")}
    assert "IAE" in table.columns and "TV_u" in table.columns

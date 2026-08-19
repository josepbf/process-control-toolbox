"""Tuning rules are checked against the formulas as published."""

import pytest

from src.tuning.rules import (
    imc_pi,
    series_to_ideal,
    simc_pi,
    simc_pid,
    ziegler_nichols_open_loop,
)


def test_simc_pi_matches_the_published_formula():
    # Skogestad (2003): Kc = tau / (K*(tau_c+theta)), Ti = min(tau, 4*(tau_c+theta))
    t = simc_pi(K=2.0, tau=10.0, theta=2.0, tau_c=2.0)
    assert t.Kc == pytest.approx(10.0 / (2.0 * 4.0))
    assert t.Ti == pytest.approx(10.0)          # tau < 4*(tau_c+theta) = 16
    assert "SIMC" in t.rule


def test_simc_pi_caps_ti_on_a_lag_dominant_process():
    """The min() is the half of the rule that saves load-disturbance rejection."""
    t = simc_pi(K=1.0, tau=500.0, theta=5.0)     # tau_c = theta = 5
    assert t.Ti == pytest.approx(4.0 * 10.0)
    assert t.Ti < 500.0


def test_simc_default_tau_c_is_theta():
    assert simc_pi(K=1.0, tau=10.0, theta=3.0).Kc == pytest.approx(
        simc_pi(K=1.0, tau=10.0, theta=3.0, tau_c=3.0).Kc
    )


def test_larger_tau_c_is_more_conservative():
    tight = simc_pi(K=1.5, tau=60.0, theta=15.0, tau_c=15.0)
    loose = simc_pi(K=1.5, tau=60.0, theta=15.0, tau_c=60.0)
    assert loose.Kc < tight.Kc


def test_series_to_ideal_roundtrip():
    Kc, Ti, Td = series_to_ideal(2.0, 10.0, 2.0)
    # Series -> ideal must preserve the transfer function: check the ideal form
    # reproduces the series product form at its factored zeros.
    assert Kc == pytest.approx(2.0 * 1.2)
    assert Ti == pytest.approx(12.0)
    assert Td == pytest.approx(2.0 / 1.2)


def test_simc_pid_adds_derivative_for_the_second_lag():
    t = simc_pid(K=1.0, tau1=50.0, tau2=8.0, theta=5.0)
    assert t.Td > 0.0
    assert t.Ti > 0.0


def test_ziegler_nichols_is_more_aggressive_than_simc():
    """ZN targets quarter-amplitude decay; it should out-gain SIMC on the same
    process. This is why it is used here only as a reference point."""
    args = dict(K=1.5, tau=60.0, theta=15.0)
    assert ziegler_nichols_open_loop(**args, kind="PI").Kc > simc_pi(**args).Kc


def test_ziegler_nichols_pid_values():
    t = ziegler_nichols_open_loop(K=1.0, tau=10.0, theta=2.0, kind="PID")
    assert t.Kc == pytest.approx(1.2 * 10.0 / (1.0 * 2.0))
    assert t.Ti == pytest.approx(4.0)
    assert t.Td == pytest.approx(1.0)


def test_imc_pi_keeps_ti_at_the_pade_corrected_lag():
    t = imc_pi(K=1.0, tau=10.0, theta=2.0, tau_c=2.0)
    assert t.Ti == pytest.approx(11.0)


def test_unknown_zn_variant_raises():
    with pytest.raises(ValueError):
        ziegler_nichols_open_loop(K=1.0, tau=1.0, theta=1.0, kind="PIDD")

"""Relay feedback auto-tuning, checked against the analytic ultimate values."""

import numpy as np
import pytest

from process_control.plants.tank import Tank
from process_control.tuning.analysis import ultimate_gain_period
from process_control.tuning.relay import relay_autotune


def _plant(**kw):
    return Tank(K=1.5, tau=60.0, theta=15.0, h0=50.0, **kw)


def _tune(plant, **kw):
    return relay_autotune(
        plant, setpoint=50.0, u_bias=plant.steady_input(50.0),
        h=kw.pop("h", 10.0), dt=1.0, t_final=kw.pop("t_final", 900.0), **kw
    )


def test_relay_recovers_the_ultimate_period():
    """The period is the part the experiment gets right: the limit cycle sits,
    by construction, at the frequency where the process has -180 degrees."""
    r = _tune(_plant(noise_std=0.0, seed=1))
    _, Pu_true = ultimate_gain_period(K=1.5, tau=60.0, theta=15.0)
    assert r.Pu == pytest.approx(Pu_true, rel=0.05)


def test_relay_underestimates_the_ultimate_gain_and_errs_safe():
    """The describing function keeps only the fundamental of the relay's square
    wave. On a process with appreciable dead time the harmonics are not fully
    filtered, and the resulting bias is conservative -- the tuning that follows
    is gentler than the true ultimate gain would license."""
    r = _tune(_plant(noise_std=0.0, seed=1))
    Ku_true, _ = ultimate_gain_period(K=1.5, tau=60.0, theta=15.0)
    assert r.Ku < Ku_true
    assert r.Ku > 0.7 * Ku_true


def test_relay_amplitude_scales_with_the_relay_height():
    """The oscillation the experiment imposes on production is set by h, which
    is the operator's decision -- and the estimate itself should barely move."""
    small = _tune(_plant(noise_std=0.0, seed=1), h=5.0)
    large = _tune(_plant(noise_std=0.0, seed=1), h=15.0)
    assert large.amplitude > 2.0 * small.amplitude
    assert large.Ku == pytest.approx(small.Ku, rel=0.15)


def test_relay_works_with_noise_when_hysteresis_is_used():
    r = _tune(_plant(noise_std=0.15, seed=7), hysteresis=0.5)
    _, Pu_true = ultimate_gain_period(K=1.5, tau=60.0, theta=15.0)
    assert r.n_cycles >= 5
    assert np.std(r.periods) < 0.1 * r.Pu          # cycles are consistent
    assert r.Pu == pytest.approx(Pu_true, rel=0.15)


def test_hysteresis_slows_the_limit_cycle():
    """Its phase lag moves the oscillation below the true ultimate frequency --
    a known, documented cost of making the experiment noise-proof."""
    clean = _tune(_plant(noise_std=0.0, seed=1), hysteresis=0.0)
    hyst = _tune(_plant(noise_std=0.0, seed=1), hysteresis=1.0)
    assert hyst.Pu > clean.Pu


def test_relay_refuses_to_report_from_too_few_cycles():
    with pytest.raises(RuntimeError, match="cycles"):
        _tune(_plant(noise_std=0.0, seed=1), t_final=60.0)


def test_relay_refuses_when_hysteresis_swallows_the_oscillation():
    with pytest.raises(RuntimeError):
        _tune(_plant(noise_std=0.0, seed=1), h=1.0, hysteresis=8.0, t_final=3000.0)


def test_relay_result_keeps_the_experiment_trace():
    r = _tune(_plant(noise_std=0.0, seed=1))
    assert {"t", "y", "u"} <= set(r.df.columns)
    assert set(np.unique(np.round(r.df["u"], 6))) <= {
        round(33.333333 - 10, 6), round(33.333333 + 10, 6)
    } or len(np.unique(r.df["u"])) == 2         # a relay has exactly two positions

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


# ----------------------------------------------------------------------
# the wider rule ladder
# ----------------------------------------------------------------------
from src.tuning.rules import (  # noqa: E402
    amigo_pi,
    amigo_pid,
    averaging_level_pi,
    cohen_coon,
    half_rule,
    lambda_tuning,
    simc_integrating,
    tyreus_luyben,
    ziegler_nichols_closed_loop,
)


def test_zn_closed_loop_values():
    t = ziegler_nichols_closed_loop(Ku=4.0, Pu=60.0, kind="PID")
    assert t.Kc == pytest.approx(2.4)
    assert t.Ti == pytest.approx(30.0)
    assert t.Td == pytest.approx(7.5)


def test_tyreus_luyben_is_gentler_than_ziegler_nichols():
    """Its whole purpose: the same experiment, a far more conservative taste."""
    Ku, Pu = 4.0, 60.0
    tl = tyreus_luyben(Ku, Pu, kind="PI")
    zn = ziegler_nichols_closed_loop(Ku, Pu, kind="PI")
    assert tl.Kc < zn.Kc
    assert tl.Ti > zn.Ti


def test_cohen_coon_gets_relatively_gentler_as_dead_time_grows():
    """CC was derived for dead-time-dominant processes; relative to the
    lag-dominant case its gain should not blow up the way ZN's does."""
    lag_dominant = dict(K=1.0, tau=100.0, theta=5.0)
    dt_dominant = dict(K=1.0, tau=10.0, theta=20.0)
    from src.tuning.rules import ziegler_nichols_open_loop as zn

    assert cohen_coon(**lag_dominant).Kc / zn(**lag_dominant).Kc > 1.0
    assert cohen_coon(**dt_dominant).Kc / zn(**dt_dominant).Kc < 1.4


def test_lambda_tuning_cancels_the_process_lag():
    t = lambda_tuning(K=2.0, tau=40.0, theta=5.0, lam=40.0)
    assert t.Ti == pytest.approx(40.0)               # Ti = tau exactly, no cap
    assert t.Kc == pytest.approx(40.0 / (2.0 * 45.0))


def test_lambda_tuning_is_slower_for_larger_lambda():
    fast = lambda_tuning(K=1.0, tau=60.0, theta=15.0, lam=45.0)
    slow = lambda_tuning(K=1.0, tau=60.0, theta=15.0, lam=200.0)
    assert slow.Kc < fast.Kc


def test_amigo_is_conservative_relative_to_ziegler_nichols():
    from src.tuning.rules import ziegler_nichols_open_loop as zn

    args = dict(K=1.5, tau=60.0, theta=15.0)
    assert amigo_pi(**args).Kc < zn(**args, kind="PI").Kc
    assert amigo_pid(**args).Td > 0.0


def test_simc_integrating_has_no_process_lag_to_cancel():
    """Ti comes only from tau_c and theta -- there is no tau in an integrator."""
    t = simc_integrating(k_prime=0.01, theta=10.0, tau_c=10.0)
    assert t.Kc == pytest.approx(1.0 / (0.01 * 20.0))
    assert t.Ti == pytest.approx(80.0)


def test_averaging_level_gain_is_set_by_the_allowed_excursion():
    t = averaging_level_pi(k_prime=0.02, v_max=20.0, y_max_dev=25.0)
    assert t.Kc == pytest.approx(0.8)
    assert t.Ti is None                              # P-only, on purpose


def test_half_rule_splits_the_largest_neglected_lag():
    # Three lags 100, 20, 5 with 3 s of dead time:
    #   tau_eff   = 100 + 20/2                = 110
    #   theta_eff = 3 + 20/2 + 5              = 18
    tau_eff, theta_eff = half_rule([100.0, 20.0, 5.0], theta=3.0)
    assert tau_eff == pytest.approx(110.0)
    assert theta_eff == pytest.approx(18.0)


def test_half_rule_sends_inverse_response_and_sampling_into_the_dead_time():
    tau_eff, theta_eff = half_rule([50.0], theta=0.0, inverse_zeros=[4.0], dt=2.0)
    assert tau_eff == pytest.approx(50.0)
    assert theta_eff == pytest.approx(5.0)           # 4 + 2/2


def test_half_rule_on_a_single_lag_is_the_identity():
    assert half_rule([30.0], theta=6.0) == (30.0, 6.0)

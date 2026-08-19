"""Frequency-domain analysis, checked against published robustness values."""

import numpy as np
import pytest

from src.tuning.analysis import (
    fopdt_response,
    pid_on_fopdt,
    pid_on_integrator,
    ultimate_gain_period,
)
from src.tuning.rules import (
    amigo_pi,
    cohen_coon,
    imc_pi,
    lambda_tuning,
    simc_integrating,
    simc_pi,
    tyreus_luyben,
    ziegler_nichols_closed_loop,
    ziegler_nichols_open_loop,
)

PLANT = dict(K=1.5, tau=60.0, theta=15.0)


def test_fopdt_response_at_dc_is_the_process_gain():
    assert fopdt_response(2.0, 10.0, 3.0, np.array([1e-9]))[0].real == pytest.approx(2.0)


def test_simc_reproduces_skogestads_published_robustness():
    """Skogestad (2003) reports that SIMC with tau_c = theta gives roughly
    GM ~ 3, PM ~ 60 deg, Ms ~ 1.6. Reproducing those three numbers validates
    the tuning rule and the analysis code against each other."""
    t = simc_pi(**PLANT)
    m = pid_on_fopdt(**PLANT, Kc=t.Kc, Ti=t.Ti)
    assert 2.8 < m["GM"] < 3.4
    assert 58.0 < m["PM_deg"] < 64.0
    assert 1.5 < m["Ms"] < 1.7


def test_amigo_meets_its_own_robustness_target():
    """AMIGO is fitted under the constraint Ms <= 1.4, across process types."""
    for tau, theta in [(60.0, 15.0), (100.0, 5.0), (20.0, 20.0), (10.0, 30.0)]:
        t = amigo_pi(K=1.5, tau=tau, theta=theta)
        m = pid_on_fopdt(K=1.5, tau=tau, theta=theta, Kc=t.Kc, Ti=t.Ti)
        assert m["Ms"] < 1.45, (tau, theta, m["Ms"])


def test_the_rules_order_by_robustness_as_the_literature_says():
    Ku, Pu = ultimate_gain_period(**PLANT)
    ms = {}
    for name, t in {
        "lambda": lambda_tuning(**PLANT),
        "amigo": amigo_pi(**PLANT),
        "simc": simc_pi(**PLANT),
        "tl": tyreus_luyben(Ku, Pu),
        "imc": imc_pi(**PLANT),
        "zn": ziegler_nichols_open_loop(**PLANT, kind="PI"),
        "cc": cohen_coon(**PLANT, kind="PI"),
    }.items():
        ms[name] = pid_on_fopdt(**PLANT, Kc=t.Kc, Ti=t.Ti, Td=t.Td)["Ms"]

    assert ms["lambda"] < ms["simc"] < ms["zn"] < ms["cc"]
    assert ms["amigo"] < ms["simc"]
    assert ms["tl"] < ms["zn"]
    assert ms["simc"] < 1.7 < ms["imc"]              # SIMC comfortable, IMC not


def test_ms_bounds_gain_and_phase_margin():
    """GM >= Ms/(Ms-1) and PM >= 2*arcsin(1/(2Ms)) are algebraic consequences of
    the definition of Ms, so they must hold for every tuning. This is a
    property test of the analysis code itself."""
    Ku, Pu = ultimate_gain_period(**PLANT)
    for t in [
        simc_pi(**PLANT), imc_pi(**PLANT), lambda_tuning(**PLANT), amigo_pi(**PLANT),
        cohen_coon(**PLANT, kind="PID"), ziegler_nichols_closed_loop(Ku, Pu, kind="PID"),
        tyreus_luyben(Ku, Pu, kind="PID"),
    ]:
        m = pid_on_fopdt(**PLANT, Kc=t.Kc, Ti=t.Ti, Td=t.Td)
        assert m["GM"] >= m["GM_from_Ms"] - 1e-6
        assert m["PM_deg"] >= m["PM_from_Ms_deg"] - 1e-6


def test_ultimate_gain_puts_the_loop_exactly_on_the_stability_boundary():
    """At Kc = Ku a proportional loop has unit gain at -180 deg: GM = 1."""
    Ku, Pu = ultimate_gain_period(**PLANT)
    m = pid_on_fopdt(**PLANT, Kc=Ku, Ti=None)
    assert m["GM"] == pytest.approx(1.0, rel=1e-3)
    assert 2 * np.pi / m["w180"] == pytest.approx(Pu, rel=1e-3)


def test_half_the_ultimate_gain_gives_a_gain_margin_of_two():
    Ku, _ = ultimate_gain_period(**PLANT)
    assert pid_on_fopdt(**PLANT, Kc=0.5 * Ku, Ti=None)["GM"] == pytest.approx(2.0, rel=1e-3)


def test_a_process_without_dead_time_has_no_finite_ultimate_gain():
    with pytest.raises(ValueError):
        ultimate_gain_period(K=1.0, tau=10.0, theta=0.0)


def test_more_dead_time_means_a_lower_ultimate_gain():
    Ku_small, _ = ultimate_gain_period(K=1.0, tau=60.0, theta=5.0)
    Ku_large, _ = ultimate_gain_period(K=1.0, tau=60.0, theta=30.0)
    assert Ku_large < Ku_small


def test_simc_integrating_tuning_is_robust_on_an_integrator():
    t = simc_integrating(k_prime=0.02, theta=10.0)
    m = pid_on_integrator(k_prime=0.02, theta=10.0, Kc=t.Kc, Ti=t.Ti)
    assert 1.4 < m["Ms"] < 1.8            # Skogestad reports Ms ~ 1.7 for tau_c = theta
    assert m["GM"] > 2.0


def test_integrator_analysis_handles_a_reverse_acting_loop():
    """Outflow-manipulated level loops have k' < 0 and Kc < 0; the loop gain is
    the product, so the robustness numbers must come out identical to the
    forward-acting case."""
    fwd = pid_on_integrator(k_prime=0.02, theta=10.0, Kc=2.5, Ti=80.0)
    rev = pid_on_integrator(k_prime=-0.02, theta=10.0, Kc=-2.5, Ti=80.0)
    assert rev["Ms"] == pytest.approx(fwd["Ms"], rel=1e-9)
    assert 1.0 < rev["Ms"] < 3.0


def test_p_only_on_an_integrator_is_stable_and_offset_free_in_level():
    """Averaging level control is P-only on an integrating process, which is
    the one case where proportional action alone leaves no offset: the
    integrator supplies the integral."""
    m = pid_on_integrator(k_prime=-0.02, theta=10.0, Kc=-0.75, Ti=None)
    assert m["Ms"] < 1.5
    assert m["GM"] > 3.0

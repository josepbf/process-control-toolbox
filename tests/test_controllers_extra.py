"""Cascade, feedforward, velocity-form PID and time-proportioning output."""

import numpy as np
import pytest

from src.controllers.cascade import CascadeController
from src.controllers.feedforward import FeedforwardPID, LeadLag
from src.controllers.onoff import TimeProportioningController
from src.controllers.pid import PIDController, VelocityPIDController
from src.harness.metrics import compute_metrics
from src.harness.scenarios import Scenario, constant, staircase
from src.harness.simulate import simulate
from src.plants.cascade_process import CascadeProcess
from src.plants.tank import Tank
from src.tuning.rules import simc_pi


# ----------------------------------------------------------------------
# cascade
# ----------------------------------------------------------------------
def _cascade_pair(plant, dt=1.0):
    inner = simc_pi(**plant.inner_fopdt)
    outer = simc_pi(**plant.outer_fopdt)
    u0 = plant.steady_input(30.0)
    return CascadeController(
        primary=PIDController(**outer.as_kwargs(), dt=dt, u_min=0.0, u_max=100.0,
                              u0=plant.K1 * u0, tuning_note=outer.rule),
        secondary=PIDController(**inner.as_kwargs(), dt=dt, u_min=plant.u_min[0],
                                u_max=plant.u_max[0], u0=u0, tuning_note=inner.rule),
    )


def test_cascade_rejects_an_inner_disturbance_far_better_than_a_single_loop():
    plant = CascadeProcess(noise_std=(0.15, 0.25), seed=7)
    scenario = Scenario(
        "inner_upset", dt=1.0, t_final=1600.0,
        setpoint=constant(30.0),
        disturbance=staircase([(0.0, [0.0, 0.0]), (400.0, [-15.0, 0.0])]),
        seed=7,
    )
    single = simc_pi(**plant.single_loop_fopdt)
    single_pi = PIDController(
        **single.as_kwargs(), dt=1.0, u_min=0.0, u_max=100.0,
        u0=plant.steady_input(30.0), tuning_note=single.rule,
    )

    a = compute_metrics(simulate(plant, single_pi, scenario), (400.0, 1600.0))
    b = compute_metrics(simulate(plant, _cascade_pair(plant), scenario), (400.0, 1600.0))
    assert b["IAE"] < a["IAE"] / 3.0
    assert b["peak_dev"] < a["peak_dev"] / 3.0


def test_cascade_needs_a_secondary_measurement():
    plant = CascadeProcess()
    ctrl = _cascade_pair(plant)
    with pytest.raises(ValueError):
        ctrl.compute(np.array([30.0]), np.array([30.0]), 0.0)


def test_cascade_primary_output_is_the_inner_setpoint():
    plant = CascadeProcess()
    ctrl = _cascade_pair(plant)
    # The first sample is pinned by the bumpless start, so let integral action
    # get going before checking that the outer loop asks for more flow.
    for k in range(20):
        ctrl.compute(np.array([20.0, 25.0]), np.array([40.0]), float(k))
    assert ctrl.inner_setpoint > 25.0


def test_cascade_reset_clears_both_loops():
    plant = CascadeProcess()
    ctrl = _cascade_pair(plant)
    for k in range(100):
        ctrl.compute(np.array([10.0, 10.0]), np.array([50.0]), float(k))
    wound = ctrl.inner_setpoint
    ctrl.reset()
    ctrl.compute(np.array([10.0, 10.0]), np.array([50.0]), 0.0)
    assert ctrl.inner_setpoint < wound


# ----------------------------------------------------------------------
# feedforward
# ----------------------------------------------------------------------
def test_lead_lag_with_equal_time_constants_is_a_pass_through():
    f = LeadLag(tau_lead=30.0, tau_lag=30.0, dt=1.0)
    for x in (1.0, 1.0, -2.0, 5.0):
        assert f(x) == pytest.approx(x, abs=1e-12)


def test_lead_lag_has_unit_steady_state_gain():
    f = LeadLag(tau_lead=60.0, tau_lag=10.0, dt=1.0)
    for _ in range(500):
        y = f(3.0)
    assert y == pytest.approx(3.0, rel=1e-6)


def test_lead_lag_leads():
    """A lead-lag with tau_lead > tau_lag overshoots on a step: that overshoot
    is the anticipation feedforward is buying."""
    f = LeadLag(tau_lead=60.0, tau_lag=10.0, dt=1.0)
    first = f(1.0)
    assert first > 1.0


def _ff(plant, scenario, static=False, gain_error=0.0):
    p, pd_ = plant.fopdt, plant.fopdt_disturbance
    t = simc_pi(**p)
    return FeedforwardPID(
        feedback=PIDController(**t.as_kwargs(), dt=scenario.dt, u_min=0.0, u_max=100.0,
                               u0=plant.steady_input(50.0), tuning_note=t.rule),
        K_p=p["K"] * (1 + gain_error), K_d=pd_["K"], tau_p=p["tau"], tau_d=pd_["tau"],
        theta_p=p["theta"], theta_d=pd_["theta"], dt=scenario.dt,
        u_min=0.0, u_max=100.0, static_only=static,
    )


def _ff_scenario():
    return Scenario("upset", dt=1.0, t_final=1200.0, setpoint=constant(50.0),
                    disturbance=staircase([(0.0, 0.0), (200.0, -12.0)]), seed=7)


def test_feedforward_gain_is_the_ratio_of_the_two_path_gains():
    plant = Tank(K=1.5, Kd=1.0, tau_d=15.0, theta_d=45.0)
    ff = _ff(plant, _ff_scenario())
    assert ff.gain == pytest.approx(-1.0 / 1.5)


def test_feedforward_realisability_is_reported_honestly():
    ok = _ff(Tank(theta=15.0, theta_d=45.0), _ff_scenario())
    late = _ff(Tank(theta=15.0, theta_d=0.0), _ff_scenario())
    assert ok.realisable and ok.delay_samples == 30
    assert not late.realisable and late.delay_samples == 0
    assert "NOT realisable" in late.tuning_note


def test_feedforward_cuts_the_peak_deviation_when_it_is_realisable():
    plant = Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, theta_d=45.0,
                 h0=50.0, noise_std=0.15, seed=7)
    scenario = _ff_scenario()
    fb = compute_metrics(simulate(plant, _ff(plant, scenario).feedback, scenario), (200.0, 1200.0))
    ff = compute_metrics(simulate(plant, _ff(plant, scenario), scenario), (200.0, 1200.0))
    assert ff["peak_dev"] < fb["peak_dev"] / 5.0


def test_feedforward_helps_much_less_when_it_cannot_be_realised():
    """theta_d < theta_p: the ideal compensator is non-causal, so the best
    available design acts late and most of the benefit disappears."""
    kw = dict(K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, h0=50.0,
              noise_std=0.15, seed=7)
    scenario = _ff_scenario()
    good = Tank(**kw, theta_d=45.0)
    bad = Tank(**kw, theta_d=0.0)

    gain_good = (compute_metrics(simulate(good, _ff(good, scenario).feedback, scenario), (200.0, 1200.0))["IAE"]
                 / compute_metrics(simulate(good, _ff(good, scenario), scenario), (200.0, 1200.0))["IAE"])
    gain_bad = (compute_metrics(simulate(bad, _ff(bad, scenario).feedback, scenario), (200.0, 1200.0))["IAE"]
                / compute_metrics(simulate(bad, _ff(bad, scenario), scenario), (200.0, 1200.0))["IAE"])
    assert gain_good > 3.0 > gain_bad


def test_feedforward_survives_a_wrong_gain_because_feedback_is_underneath():
    plant = Tank(K=1.5, tau=60.0, theta=15.0, Kd=1.0, tau_d=15.0, theta_d=45.0,
                 h0=50.0, noise_std=0.15, seed=7)
    scenario = _ff_scenario()
    fb = compute_metrics(simulate(plant, _ff(plant, scenario).feedback, scenario), (200.0, 1200.0))
    wrong = compute_metrics(simulate(plant, _ff(plant, scenario, gain_error=0.30), scenario),
                            (200.0, 1200.0))
    assert wrong["IAE"] < fb["IAE"]                 # still better than no feedforward
    assert abs(wrong["ss_offset"]) < 0.1            # and feedback removes the offset


def test_feedforward_leaves_the_feedback_controller_the_range_it_still_has():
    plant = Tank(K=1.5, Kd=1.0, tau_d=15.0, theta_d=45.0)
    ff = _ff(plant, _ff_scenario())
    ff.reset()
    ff.compute(np.array([50.0]), np.array([50.0]), 0.0, d=np.array([-30.0]))
    assert ff.feedback.u_max == pytest.approx(100.0 - ff.last_ff)
    assert ff.feedback.u_min == pytest.approx(0.0 - ff.last_ff)


def test_the_harness_only_hands_the_disturbance_to_controllers_that_declare_it():
    plant = Tank(K=1.5, Kd=1.0, tau_d=15.0, theta_d=45.0, h0=50.0, noise_std=0.0)
    scenario = _ff_scenario()
    ff = _ff(plant, scenario)
    assert ff.uses_measured_disturbance
    assert not ff.feedback.uses_measured_disturbance
    assert simulate(plant, ff, scenario).attrs["uses_measured_disturbance"]
    assert not simulate(plant, ff.feedback, scenario).attrs["uses_measured_disturbance"]


# ----------------------------------------------------------------------
# velocity form
# ----------------------------------------------------------------------
def test_velocity_form_tracks_the_positional_form_when_nothing_saturates():
    plant = Tank(h0=30.0, noise_std=0.0, seed=1)
    scenario = Scenario("s", dt=1.0, t_final=1200.0,
                        setpoint=staircase([(0.0, 30.0), (100.0, 45.0)]))
    t = simc_pi(**plant.fopdt)
    kw = dict(dt=1.0, u_min=0.0, u_max=100.0, u0=plant.steady_input(30.0))

    pos = compute_metrics(simulate(plant, PIDController(**t.as_kwargs(), **kw), scenario))
    vel = compute_metrics(simulate(plant, VelocityPIDController(**t.as_kwargs(), **kw), scenario))
    assert vel["IAE"] == pytest.approx(pos["IAE"], rel=0.05)


def test_velocity_form_does_not_wind_up():
    """Same infeasible-setpoint scenario as experiment 2. The velocity form has
    no integral state to run away, so it recovers without any anti-windup
    machinery bolted on."""
    plant = Tank(h0=30.0, u_min=0.0, u_max=30.0, noise_std=0.0, seed=7)
    scenario = Scenario("windup", dt=1.0, t_final=2400.0,
                        setpoint=staircase([(0.0, 30.0), (100.0, 60.0), (600.0, 35.0)]))
    t = simc_pi(**plant.fopdt)
    kw = dict(dt=1.0, u_min=0.0, u_max=30.0, u0=plant.steady_input(30.0))

    naive = simulate(plant, PIDController(**t.as_kwargs(), Tt=np.inf, **kw), scenario)
    velocity = simulate(plant, VelocityPIDController(**t.as_kwargs(), **kw), scenario)
    recovery = (600.0, 2400.0)
    assert compute_metrics(velocity, recovery)["IAE"] < 0.25 * compute_metrics(naive, recovery)["IAE"]


def test_velocity_form_starts_from_the_current_valve_position():
    ctrl = VelocityPIDController(Kc=1.0, Ti=60.0, dt=1.0, u_min=0.0, u_max=100.0, u0=42.0)
    assert ctrl.compute(np.array([10.0]), np.array([50.0]), 0.0)[0] == pytest.approx(42.0)


def test_velocity_form_requires_integral_action():
    with pytest.raises(ValueError):
        VelocityPIDController(Kc=1.0, Ti=None, dt=1.0)


# ----------------------------------------------------------------------
# time proportioning
# ----------------------------------------------------------------------
class _Demand(PIDController):
    """Stub whose output is a fixed percentage."""

    def __init__(self, value):
        super().__init__(Kc=0.0, Ti=None, dt=1.0, u0=value)


def test_time_proportioning_duty_cycle_matches_the_demand():
    """Cycle of 100 samples, so the duty cycle resolves to 1 %. With a shorter
    cycle the duty quantises to dt/period, which is a real limitation of the
    scheme rather than an artefact of the test."""
    ctrl = TimeProportioningController(_Demand(37.0), period=100.0, dt=1.0)
    ctrl.reset()
    out = [ctrl.compute(np.array([0.0]), np.array([0.0]), float(k))[0] for k in range(500)]
    assert np.mean(out) == pytest.approx(37.0, abs=0.5)
    assert set(out) <= {0.0, 100.0}                 # only ever fully on or fully off


def test_time_proportioning_clamps_the_duty_cycle():
    on = TimeProportioningController(_Demand(140.0), period=20.0, dt=1.0)
    off = TimeProportioningController(_Demand(-20.0), period=20.0, dt=1.0)
    on.reset(), off.reset()
    assert all(on.compute(np.array([0.0]), np.array([0.0]), float(k))[0] == 100.0
               for k in range(40))
    assert all(off.compute(np.array([0.0]), np.array([0.0]), float(k))[0] == 0.0
               for k in range(40))


def test_time_proportioning_beats_plain_onoff_on_ripple():
    """Switching fast compared with the process lets the process average the
    pulses, which is why a relay-output temperature controller can hold a
    fraction of a degree while plain ON/OFF limit-cycles."""
    from src.controllers.onoff import OnOffController

    dt, period = 0.5, 5.0
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.0, seed=1)
    scenario = Scenario("s", dt=dt, t_final=2000.0, setpoint=constant(50.0))
    t = simc_pi(**plant.fopdt)

    onoff = compute_metrics(simulate(plant, OnOffController(100.0, 0.0, 1.0), scenario),
                            (500.0, 2000.0))
    pwm = compute_metrics(
        simulate(plant, TimeProportioningController(
            PIDController(**t.as_kwargs(), dt=dt, u_min=0.0, u_max=100.0, u0=20.0),
            period=period, dt=dt), scenario),
        (500.0, 2000.0),
    )
    # Ripple ratio scales as 4*theta/period: the faster the switching relative
    # to the dead time, the smaller the ripple.
    assert pwm["peak_dev"] < onoff["peak_dev"] / 5.0
    # And the cost is switching wear, which is the whole reason the cycle time
    # is not simply set as short as the hardware allows.
    assert pwm["reversals"] > 10 * onoff["reversals"]

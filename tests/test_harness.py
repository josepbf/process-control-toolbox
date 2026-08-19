"""The harness itself: logging, isolation between runs, reproducibility."""

import numpy as np
import pytest

from src.controllers.base import Controller
from src.controllers.pid import PIDController
from src.harness.metrics import summarize
from src.harness.scenarios import Scenario, constant, pulse, staircase
from src.harness.simulate import run_all, simulate
from src.plants.tank import Tank


class ConstantController(Controller):
    """Open-loop reference: ignores the measurement entirely."""

    def __init__(self, u):
        self.u = float(u)
        self.name = "constant"
        self.tuning_note = "open loop"

    def compute(self, y, setpoint, t):
        return np.array([self.u])

    def reset(self):
        pass


def _scenario(**kw):
    return Scenario("s", dt=1.0, t_final=200.0, setpoint=constant(50.0), **kw)


def test_log_shape_and_columns():
    plant, scenario = Tank(noise_std=0.1), _scenario()
    df = simulate(plant, ConstantController(30.0), scenario)

    assert len(df) == scenario.n_steps + 1
    for col in ("t", "y", "sp", "u", "d", "violation", "violated", "solve_time"):
        assert col in df.columns
    assert df["t"].iloc[-1] == pytest.approx(scenario.t_final)
    assert df.attrs["seed"] == scenario.seed
    assert df.attrs["controller"] == "constant"


def test_runs_are_reproducible():
    plant, scenario = Tank(noise_std=0.4, seed=11), _scenario()
    a = simulate(plant, ConstantController(30.0), scenario)
    b = simulate(plant, ConstantController(30.0), scenario)
    assert np.array_equal(a["y"].to_numpy(), b["y"].to_numpy())


def test_simulate_does_not_mutate_the_caller_plant():
    """Every controller must start from the same plant state, so the runner
    works on a copy rather than on the object the experiment holds."""
    plant, scenario = Tank(h0=30.0, noise_std=0.0), _scenario()
    simulate(plant, ConstantController(80.0), scenario)
    assert plant.t == 0.0
    assert plant.x[0] == pytest.approx(30.0)


def test_run_all_isolates_controllers_from_each_other():
    plant, scenario = Tank(noise_std=0.2, seed=5), _scenario()
    runs = run_all(
        plant,
        {"a": ConstantController(30.0), "b": ConstantController(60.0), "a2": ConstantController(30.0)},
        scenario,
    )
    assert np.array_equal(runs["a"]["y"].to_numpy(), runs["a2"]["y"].to_numpy())
    assert not np.array_equal(runs["a"]["y"].to_numpy(), runs["b"]["y"].to_numpy())


def test_actuator_limits_are_logged_for_the_metrics():
    plant = Tank(u_min=0.0, u_max=40.0)
    df = simulate(plant, ConstantController(30.0), _scenario())
    assert df.attrs["u_max"] == [40.0]


def test_output_limit_violations_are_counted():
    """Phase 1 records violations without enforcing them -- that band is what
    phase 2 will hand to MPC as a hard constraint."""
    plant = Tank(h0=30.0, noise_std=0.0, y_max=55.0)
    df = simulate(plant, ConstantController(60.0), _scenario())   # drives to 90 %
    assert df["violated"].sum() > 0
    assert df["violation"].max() > 0.0
    assert summarize({"c": df})["n_violations"].iloc[0] > 0


def test_solve_time_is_recorded():
    df = simulate(Tank(), PIDController(Kc=1.0, Ti=60.0, dt=1.0, u0=20.0), _scenario())
    assert (df["solve_time"].iloc[:-1] > 0).all()
    assert np.isnan(df["solve_time"].iloc[-1])       # no controller call on the last row


def test_scenario_signal_builders():
    sp = staircase([(0.0, 10.0), (50.0, 20.0)])
    assert sp(0.0)[0] == 10.0 and sp(49.9)[0] == 10.0 and sp(50.0)[0] == 20.0

    d = pulse(10.0, 20.0, amplitude=-5.0)
    assert d(9.9)[0] == 0.0 and d(10.0)[0] == -5.0 and d(20.0)[0] == 0.0

    with pytest.raises(ValueError):
        staircase([(5.0, 1.0)])                       # must define t = 0


def test_disturbance_reaches_the_plant():
    plant, scenario = Tank(noise_std=0.0), _scenario(disturbance=constant(-12.0))
    quiet = simulate(plant, ConstantController(30.0), _scenario())
    upset = simulate(plant, ConstantController(30.0), scenario)
    assert upset["y"].iloc[-1] < quiet["y"].iloc[-1] - 5.0

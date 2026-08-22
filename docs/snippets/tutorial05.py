"""Tutorial 5: a custom controller — a valve slew-rate limiter."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from src.controllers.base import Controller
from src.controllers.pid import PIDController
from src.harness.metrics import format_table, summarize
from src.harness.plotting import plot_runs
from src.harness.scenarios import setpoint_and_load
from src.harness.simulate import run_all
from src.plants.tank import Tank
from src.tuning.rules import simc_pi


class RateLimited(Controller):
    """Wrap any controller in a valve slew-rate limit.

    Large valves, dampers and variable-speed drives cannot move arbitrarily
    fast: a motorised control valve is typically 30-60 s from fully shut to
    fully open, which on a 1 s sample time is a limit of 1.7-3.3 % per second.
    A control law that assumes it can step the actuator anywhere it likes is
    modelling something the plant cannot do.

    This is a *composition* controller: it holds another controller and
    modifies its output, in the same style as
    ``controllers.onoff.TimeProportioningController``.
    """

    def __init__(self, inner: Controller, max_rate: float, dt: float,
                 u0: float = 0.0, name: str | None = None):
        self.inner = inner
        self.max_rate = float(max_rate)      # units of u per second
        self.dt = float(dt)
        self.u0 = float(u0)
        self.name = name or f"{inner.name} + rate limit"
        self.tuning_note = f"slew limit {self.max_rate:g}/s; inner: {inner.tuning_note}"
        # Pass the declaration through, or a wrapped feedforward controller
        # would silently stop being given the disturbance measurement.
        self.uses_measured_disturbance = inner.uses_measured_disturbance
        self.reset()

    def reset(self) -> None:
        self.inner.reset()
        self._u = self.u0

    def compute(self, y, setpoint, t, **kwargs) -> np.ndarray:
        demand = float(np.atleast_1d(self.inner.compute(y, setpoint, t, **kwargs))[0])
        step = self.max_rate * self.dt
        self._u = float(np.clip(demand, self._u - step, self._u + step))
        return np.array([self._u])

    def describe(self) -> dict:
        return {
            "controller": self.name, "tuning": self.tuning_note,
            "max_rate": self.max_rate, "inner": self.inner.describe(),
        }


if __name__ == "__main__":
    plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.15, seed=7)
    scenario = setpoint_and_load(dt=1.0, y_start=30.0, y_step=50.0, seed=7)
    tuning = simc_pi(**plant.fopdt)

    def pi(name):
        return PIDController(
            **tuning.as_kwargs(), dt=scenario.dt,
            u_min=plant.u_min[0], u_max=plant.u_max[0],
            u0=plant.steady_input(30.0), name=name, tuning_note=tuning.rule,
        )

    controllers = {"PI": pi("PI")}
    for rate in (2.0, 0.5, 0.2):
        controllers[f"PI + {rate} %/s"] = RateLimited(
            pi("PI"), max_rate=rate, dt=scenario.dt, u0=plant.steady_input(30.0)
        )

    runs = run_all(plant, controllers, scenario)
    print(format_table(
        summarize(runs, windows=scenario.windows),
        columns=["IAE", "settling_2pct_s", "overshoot_pct", "peak_dev",
                 "TV_u", "max_du", "reversals"],
    ))
    plot_runs(runs, title="Valve slew-rate limiting", scenario=scenario,
              path="results/tutorial05.png")

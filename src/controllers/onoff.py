"""ON/OFF (bang-bang) control with hysteresis.

The oldest control law there is: the actuator has two positions and the
controller picks one. A deadband of width ``hysteresis`` around the setpoint
stops the output chattering on measurement noise -- at the price of a
guaranteed **limit cycle**: the loop never settles, it oscillates forever with
an amplitude set by the deadband and a period set by the process lag and dead
time. That limit cycle is not a bug to be tuned away; it is what this control
structure *is*, and it is the baseline every smoother method has to beat.
"""

from __future__ import annotations

import numpy as np

from .base import Controller


class OnOffController(Controller):
    """Two-position controller with symmetric hysteresis about the setpoint."""

    def __init__(
        self,
        u_on: float,
        u_off: float,
        hysteresis: float = 1.0,
        name: str = "ON/OFF",
    ):
        self.u_on = float(u_on)
        self.u_off = float(u_off)
        self.hysteresis = float(hysteresis)
        self.name = name
        self.tuning_note = (
            f"two-position, deadband = {self.hysteresis:g} (units of y); "
            f"u_on = {self.u_on:g}, u_off = {self.u_off:g}"
        )
        self._u = None

    def reset(self) -> None:
        self._u = None

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        y0 = float(np.atleast_1d(y)[0])
        sp = float(np.atleast_1d(setpoint)[0])
        half = 0.5 * self.hysteresis

        if y0 < sp - half:
            self._u = self.u_on
        elif y0 > sp + half:
            self._u = self.u_off
        elif self._u is None:
            # Inside the deadband on the very first sample: no previous state to
            # hold, so pick the action that moves toward the setpoint.
            self._u = self.u_on if y0 < sp else self.u_off
        # else: inside the deadband -> hold the previous position.

        return np.array([self._u])


class TimeProportioningController(Controller):
    """Turn a continuous controller's output into ON/OFF pulses (PWM).

    The industrial situation: the control law wants 37 %% of full power, but
    the actuator is a contactor, a solenoid or a solid-state relay that is only
    ever fully on or fully off. Time proportioning resolves that by switching
    fast compared with the process: within each cycle of length ``period``, the
    output is ON for 37 %% of the cycle and OFF for the rest.

    The process, being a low-pass filter, averages the pulses -- so the
    *effective* manipulated variable is continuous even though the actuator is
    binary. This is why an oven or an extruder barrel holds temperature to a
    fraction of a degree with a relay output, while the plain ON/OFF controller
    of ``OnOffController`` limit-cycles by several degrees.

    The design constraint is the cycle time: short compared with the process
    time constant (or the ripple shows up in the controlled variable), long
    compared with the actuator's tolerance for switching (or the contactor
    wears out). The ratio ``period / tau`` is the number to watch, and the
    ``reversals`` metric is what it costs.
    """

    def __init__(
        self,
        inner: Controller,
        period: float,
        dt: float,
        u_on: float = 100.0,
        u_off: float = 0.0,
        name: str = "time-proportioning",
    ):
        self.inner = inner
        self.period = float(period)
        self.dt = float(dt)
        self.u_on, self.u_off = float(u_on), float(u_off)
        self.name = name
        self.tuning_note = (
            f"time-proportioning, cycle {self.period:g} s, "
            f"inner: {getattr(inner, 'tuning_note', 'n/a')}"
        )
        self.reset()

    def reset(self) -> None:
        self.inner.reset()
        self.duty = 0.0

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        demand = float(np.atleast_1d(self.inner.compute(y, setpoint, t))[0])
        span = self.u_on - self.u_off
        self.duty = float(np.clip((demand - self.u_off) / span, 0.0, 1.0)) if span else 0.0

        phase = (t % self.period) / self.period
        return np.array([self.u_on if phase < self.duty else self.u_off])

    def describe(self) -> dict:
        return {
            "controller": self.name, "tuning": self.tuning_note,
            "period_s": self.period, "inner": self.inner.describe(),
        }

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

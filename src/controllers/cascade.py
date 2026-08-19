"""Cascade control: an inner loop that catches the upset before it arrives.

Structure:

    setpoint --> [primary PID] --> setpoint for the secondary
                                       |
                            [secondary PID] --> valve --> process

The primary controller's *output is not a valve position*; it is the setpoint
handed to a second, faster controller that closes its own loop on an
intermediate measurement. Two things follow, and both matter:

* A disturbance entering the inner stage (supply pressure moving the flow a
  valve position delivers) is seen and corrected by the secondary controller
  within its own, fast, time constant -- before the primary variable has moved
  enough for the outer loop to notice.
* Nonlinearity in the actuator (valve characteristic, hysteresis) is absorbed
  by the inner loop, so the outer loop sees a much closer-to-linear process.

Design rules, from Shinskey (*Process Control Systems*, 1996) and standard
practice: the inner loop must be roughly 3-5 times faster than the outer one,
it is tuned first and tuned tight (it does not need to be smooth, only fast),
and only then is the outer loop tuned against the *closed* inner loop.

Windup has an extra wrinkle here. If the secondary hits a valve limit, the
primary must stop integrating too, or it winds up against a loop that cannot
respond -- exactly the failure of experiment 2, one level up. That is handled
by giving the primary controller output limits equal to the achievable range
of the inner setpoint, so its own back-calculation does the work.
"""

from __future__ import annotations

import numpy as np

from .base import Controller
from .pid import PIDController


class CascadeController(Controller):
    """Primary/secondary PID pair on a plant reporting ``y = [primary, secondary]``."""

    def __init__(
        self,
        primary: PIDController,
        secondary: PIDController,
        name: str = "cascade PI/PI",
        tuning_note: str = "",
    ):
        self.primary = primary
        self.secondary = secondary
        self.name = name
        self.tuning_note = tuning_note or (
            f"inner: {secondary.tuning_note} | outer: {primary.tuning_note}"
        )
        #: Last inner setpoint, exposed for plotting the cascade's inner signal.
        self.inner_setpoint = 0.0
        self.reset()

    def reset(self) -> None:
        self.primary.reset()
        self.secondary.reset()
        self.inner_setpoint = 0.0

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        y = np.atleast_1d(y)
        if y.size < 2:
            raise ValueError("cascade control needs a secondary measurement: y = [primary, secondary]")
        sp_primary = np.atleast_1d(setpoint)[0]

        # Outer loop: its output *is* the inner setpoint. Its u_min/u_max were
        # set to the achievable inner range, so its anti-windup already knows
        # what the inner loop can deliver.
        sp_inner = float(self.primary.compute(y[:1], np.array([sp_primary]), t)[0])
        self.inner_setpoint = sp_inner

        # Inner loop: closes on the secondary measurement, drives the actuator.
        return self.secondary.compute(y[1:2], np.array([sp_inner]), t)

    def describe(self) -> dict:
        return {
            "controller": self.name,
            "tuning": self.tuning_note,
            "outer": self.primary.describe(),
            "inner": self.secondary.describe(),
        }

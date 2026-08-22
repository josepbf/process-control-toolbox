"""Controller base class.

Any control law -- ON/OFF, PID, LQR, MPC -- implements this interface, so the
harness can swap one for another without changing a line of simulation code.

A controller is allowed (and, from phase 2 on, encouraged) to carry its own
*internal model* of the process. That model is not required to match the plant,
and the interesting results in this project come from the gap between them.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class Controller(ABC):
    """Abstract control law evaluated once per sample."""

    #: Human-readable label used in plots and metric tables.
    name: str = "controller"

    #: Set True by controllers that read a *measured* disturbance (feedforward).
    #: The harness then passes ``d=`` to :meth:`compute`. Declaring this is a
    #: modelling claim: it asserts the disturbance is instrumented on the real
    #: plant. Feedback-only controllers must leave it False, so no controller
    #: can quietly benefit from information the others do not have.
    uses_measured_disturbance: bool = False

    #: How this controller was tuned, e.g. "SIMC (Skogestad 2003), tau_c = theta".
    #: Fairness rule 1: every baseline must carry a named, cited tuning rule.
    tuning_note: str = "unspecified"

    @abstractmethod
    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        """Return the manipulated variable for this sample."""

    @abstractmethod
    def reset(self) -> None:
        """Clear all internal state (integrators, histories, warm starts)."""

    def diagnostics(self) -> dict[str, float]:
        """Internal signals worth logging alongside ``u``, as scalars.

        A control law usually computes quantities that are not its output but
        that explain its output: the setpoint a cascade's outer loop is asking
        of its inner loop, the share of the move a feedforward compensator
        contributed, the duty cycle behind a pulsed actuator. The harness logs
        whatever is returned here under a ``diag_`` prefix, one column per key,
        so those signals can be plotted and inspected instead of being
        recomputed or guessed at.

        Scalars only -- the log is one row per sample. Return ``{}`` (the
        default) if the controller has nothing to add.
        """
        return {}

    def describe(self) -> dict:
        """Tuning record written alongside the metrics of every run."""
        return {"controller": self.name, "tuning": self.tuning_note}

"""Feedforward from a measured disturbance, added to a feedback controller.

Feedback is fundamentally reactive: it cannot act until the error exists. If
the disturbance is *measured*, that limitation disappears for that disturbance
-- the controller can move the valve at the moment the upset is detected,
before the controlled variable has responded at all.

The ideal feedforward compensator is

    u_ff(s) = - G_d(s) / G_p(s) * d(s)

which, for two FOPDT models, is a lead-lag with a gain and a dead time:

    u_ff(s) = -(K_d/K_p) * (tau_p*s + 1)/(tau_d*s + 1) * exp(-(theta_d - theta_p)*s)

**Realisability is the whole story.** The exponent needs ``theta_d >= theta_p``:
the disturbance must reach the output *later* than the valve does, or the
compensator would have to act before the disturbance is measured. When that
fails, the delay term is dropped and the compensation is late by the
difference -- feedforward still helps, but it cannot be perfect, and saying so
is more useful than pretending otherwise.

Feedforward is also open loop: its accuracy is the accuracy of the ratio
``K_d/K_p``. It is never used alone. The pairing here is the industrial
standard -- feedforward for speed, feedback for the truth.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .base import Controller
from .pid import PIDController


class LeadLag:
    """Discrete lead-lag filter ``(tau_lead*s + 1)/(tau_lag*s + 1)``, Tustin."""

    def __init__(self, tau_lead: float, tau_lag: float, dt: float):
        self.tau_lead = float(tau_lead)
        # A pure lead is improper (infinite high-frequency gain); a small lag
        # is always present in practice, so one is enforced here.
        self.tau_lag = max(float(tau_lag), dt / 2.0)
        self.dt = float(dt)

        den = 2.0 * self.tau_lag + dt
        self.a = (2.0 * self.tau_lag - dt) / den
        self.b0 = (2.0 * self.tau_lead + dt) / den
        self.b1 = (dt - 2.0 * self.tau_lead) / den
        self.reset()

    def reset(self) -> None:
        self._x_prev = 0.0
        self._y_prev = 0.0

    def __call__(self, x: float) -> float:
        y = self.a * self._y_prev + self.b0 * x + self.b1 * self._x_prev
        self._x_prev, self._y_prev = x, y
        return y


class FeedforwardPID(Controller):
    """Feedback PID plus dynamic feedforward from a measured disturbance."""

    uses_measured_disturbance = True

    def __init__(
        self,
        feedback: PIDController,
        K_p: float,
        K_d: float,
        tau_p: float,
        tau_d: float,
        theta_p: float = 0.0,
        theta_d: float = 0.0,
        dt: float = 1.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        static_only: bool = False,
        name: str = "PI + feedforward",
        tuning_note: str = "",
    ):
        self.feedback = feedback
        self.dt = float(dt)
        self.u_min, self.u_max = float(u_min), float(u_max)
        self.gain = -K_d / K_p
        self.static_only = bool(static_only)

        self.lead_lag = LeadLag(tau_lead=tau_p, tau_lag=tau_d, dt=dt)

        # Delay the feedforward action when the disturbance path is slower than
        # the manipulated path; if it is faster, no amount of design recovers
        # the difference and the compensation is simply late.
        self.delay_samples = int(round(max(0.0, theta_d - theta_p) / dt))
        self.realisable = theta_d >= theta_p
        self.name = name
        self.tuning_note = tuning_note or (
            f"static gain {self.gain:.3f}"
            + ("" if static_only else f", lead-lag ({tau_p:g}s / {tau_d:g}s)")
            + (f", delayed {self.delay_samples * dt:g}s" if self.delay_samples else "")
            + ("" if self.realisable else "; NOT realisable (theta_d < theta_p), acts late")
        )
        self.reset()

    def reset(self) -> None:
        self.feedback.reset()
        self.lead_lag.reset()
        self._delay = deque([0.0] * (self.delay_samples + 1), maxlen=self.delay_samples + 1)
        self.last_ff = 0.0

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float, d=None) -> np.ndarray:
        d0 = 0.0 if d is None else float(np.atleast_1d(d)[0])

        ff = self.gain * d0 if self.static_only else self.gain * self.lead_lag(d0)
        self._delay.append(ff)
        ff = self._delay[0]
        self.last_ff = ff

        # Give the feedback controller the range that is actually left to it
        # once feedforward has taken its share, so its anti-windup stays honest.
        self.feedback.u_min = self.u_min - ff
        self.feedback.u_max = self.u_max - ff
        u_fb = float(self.feedback.compute(y, setpoint, t)[0])

        return np.array([float(np.clip(u_fb + ff, self.u_min, self.u_max))])

    def diagnostics(self) -> dict[str, float]:
        """Splitting u into its feedforward and feedback shares is the whole
        point of the structure: it shows how much of the correction was
        anticipated and how much had to be cleaned up after the fact."""
        return {"u_feedforward": float(self.last_ff)}

    def describe(self) -> dict:
        return {
            "controller": self.name,
            "tuning": self.tuning_note,
            "ff_gain": self.gain,
            "ff_realisable": self.realisable,
            "feedback": self.feedback.describe(),
        }

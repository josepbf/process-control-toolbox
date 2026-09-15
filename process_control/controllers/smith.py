"""Smith predictor: the classical answer to dead time.

Feedback degrades with dead time because the controller is always reacting to
news that is already ``theta`` seconds old. Article 5 measures the damage. The
Smith predictor (Smith 1957) is the classical structure that addresses it, and
the idea is one line: if you have a model, close the loop around the *model's*
undelayed prediction instead of the measurement, and use the measurement only
to correct the model.

Writing ``G(s) = G0(s) e^{-theta s}`` with ``G0`` the dead-time-free part, the
signal handed to the PI is::

    y_feedback = y0_model  +  (y_measured - y_delayed_model)
                 ^^^^^^^^      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                 what the      what the model got wrong, which is
                 model says    the only thing the measurement can
                 is happening  still tell you that is news
                 *now*

If the model were perfect and nothing else touched the process, the correction
term would be identically zero and the PI would be controlling ``G0`` -- a loop
with no dead time at all, which can be tuned far more aggressively. It is not
perfect, and that is the whole story of the structure:

- **The correction term is the model error**, published as a diagnostic. On a
  perfect model with no disturbance it is zero; anything else is the plant
  disagreeing, and watching it is how you tell a Smith predictor that is helping
  from one that is quietly falling apart.
- **Mismatch in ``theta`` is what hurts**, not mismatch in ``K`` or ``tau``. An
  aggressively tuned inner loop is being protected by a cancellation that only
  holds if the delay is right, and a Smith predictor tuned as though it had no
  dead time, on a plant whose dead time it has underestimated, can be worse than
  the PI it replaced. That is a result worth having, not a caveat to hide.
- **No integral action is added.** The PI inside supplies it, and the predictor
  supplies the timing. The controller is still only as offset-free as its inner
  loop.

This is also the first controller in the project to carry an internal model, so
it is what puts `models.discrete` under load before anything harder depends on
it.

References
----------
Smith, O.J.M. (1957). "Closer control of loops with dead time."
    Chemical Engineering Progress 53(5), 217-219.
Normey-Rico, J.E., Camacho, E.F. (2007). *Control of Dead-time Processes.*
    Springer.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ..models.discrete import DiscreteModel
from .base import Controller
from .pid import PIDController


class SmithPredictor(Controller):
    """A PI (or PID) closed around a dead-time-free internal model.

    Parameters
    ----------
    feedback : PIDController
        The inner law. Tune it on the **dead-time-free** model -- that is the
        point of the structure, and tuning it on the full FOPDT throws the
        benefit away. A SIMC PI on ``(K, tau, theta = dt)`` is the honest
        version of "as if there were no dead time": some dead time always
        remains, because the sample time itself is dead time.
    model : DiscreteModel
        The **delay-free** model, i.e. built with ``delay_states=False``. The
        predictor adds the delay itself, because it needs the undelayed and
        delayed predictions separately and an augmented model only gives the
        delayed one.
    theta : float
        Model dead time, seconds. Rounded to whole samples the way the plant
        rounds, and the residual is reported in ``tuning_note``.
    u_min, u_max : float
        The actuator range, passed to the inner controller's anti-windup. As
        everywhere else, the plant still owns the real limit; this only stops
        the inner integrator winding up against one it cannot see.
    """

    def __init__(
        self,
        feedback: PIDController,
        model: DiscreteModel,
        theta: float,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        name: str = "Smith predictor",
        tuning_note: str = "",
    ):
        if model.n_delay and any(n > 0 for n in model.n_delay):
            raise ValueError(
                "the Smith predictor needs the dead-time-free model: build it "
                "with fopdt_model(..., delay_states=False) and pass theta "
                "separately, because the structure needs the undelayed and the "
                "delayed prediction as two different signals"
            )
        self.feedback = feedback
        self.model = model
        self.dt = model.dt
        self.theta = float(theta)
        self.u_min = float(u_min)
        self.u_max = float(u_max)

        self.delay_samples = int(np.round(self.theta / self.dt))
        residual = self.theta - self.delay_samples * self.dt

        self.name = name
        self.tuning_note = tuning_note or (
            f"Smith predictor (Smith 1957) on {model.note or 'an internal model'}; "
            f"delay {self.delay_samples * self.dt:g}s"
            + (f" (requested {self.theta:g}s, residual {residual:+.3g}s)" if abs(residual) > 1e-12 else "")
            + f"; inner law: {feedback.tuning_note}"
        )
        self.reset()

    def reset(self) -> None:
        self.feedback.reset()
        self._x = np.zeros(self.model.n_states)
        # One slot per delayed sample plus the current one, exactly as the
        # plant's own dead-time buffers and FeedforwardPID's delay line.
        self._delay = deque([0.0] * (self.delay_samples + 1), maxlen=self.delay_samples + 1)
        self._y_undelayed = 0.0
        self._y_delayed = 0.0
        self._model_error = 0.0
        self._initialised = False

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        y0 = float(np.atleast_1d(y)[0])

        if not self._initialised:
            # Bumpless start: begin from a model state consistent with the first
            # reading, so the correction term starts at zero rather than at the
            # whole operating point. Without this the predictor's first move is
            # a kick, and every settling-time metric in the run is contaminated.
            self._seed(y0)
            self._initialised = True

        self._y_undelayed = float((self.model.C @ self._x)[0])
        self._y_delayed = float(self._delay[0])

        # The only thing the measurement still has to say that the model does
        # not already know. Zero on a perfect model with no disturbance.
        self._model_error = y0 - self._y_delayed
        y_feedback = np.array([self._y_undelayed + self._model_error])

        self.feedback.u_min = self.u_min
        self.feedback.u_max = self.u_max
        u = self.feedback.compute(y_feedback, setpoint, t)
        u_sat = float(np.clip(float(np.atleast_1d(u)[0]), self.u_min, self.u_max))

        # Advance the internal model under the move that is about to be held,
        # and push its output into the delay line. Saturated, because the model
        # must be driven by the move the plant will actually see.
        self._x = self.model.step(self._x, np.array([u_sat]))
        self._delay.append(float((self.model.C @ self._x)[0]))

        return np.array([u_sat])

    def _seed(self, y0: float) -> None:
        """Start the model and its delay line where the measurement says we are."""
        C = self.model.C
        self._x = np.linalg.lstsq(C, np.array([y0]), rcond=None)[0]
        self._delay = deque(
            [y0] * (self.delay_samples + 1), maxlen=self.delay_samples + 1
        )

    def diagnostics(self) -> dict[str, float]:
        """``model_error`` is the signal that explains this controller.

        It is the measurement minus what the model said the measurement would
        be, and it is the only information the feedback path still carries once
        the predictor has done its work. Flat and near zero means the model is
        good and the aggressive inner tuning is safe. Large or persistent means
        the cancellation the structure depends on is not happening, and the
        inner loop is being tuned for a plant that is not there.
        """
        return {
            "model_error": self._model_error,
            "y_undelayed": self._y_undelayed,
        }

    def describe(self) -> dict:
        return {
            "controller": self.name,
            "tuning": self.tuning_note,
            "theta_model": self.delay_samples * self.dt,
            "model": self.model.describe(),
            "feedback": self.feedback.describe(),
        }

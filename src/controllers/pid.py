"""PID with anti-windup and a filtered derivative.

Textbook "ideal" PID is unusable on a real plant for two reasons, and both
fixes are implemented here explicitly rather than hidden in a library:

**Integral windup.** When the valve is at a limit, the loop is open: the error
persists but the actuator cannot answer it. A naive integrator keeps
accumulating, so when the error finally reverses the controller must first
*unwind* a large integral before it starts backing off -- producing a huge
overshoot. The fix used here is back-calculation (Astrom & Hagglund,
*Advanced PID Control*, 2006): the integrator is fed the amount by which the
requested move was clipped, divided by a tracking time constant ``Tt``, which
bleeds the integral back to a feasible value.

**Derivative kick and noise.** Pure derivative action differentiates the error,
so a step in setpoint produces an impulse in ``u``; and it amplifies
measurement noise without bound. The fixes: differentiate the *measurement*
rather than the error, and pass it through a first-order filter with time
constant ``Td / N`` (``N`` typically 5-20).

Form implemented (parallel / ISA "ideal"):

    u = Kc * ( beta*sp - y  +  (1/Ti) * INT e dt  -  Td * d(y_filtered)/dt )
"""

from __future__ import annotations

import numpy as np

from .base import Controller


class PIDController(Controller):
    """Discrete PID, positional form, derivative on filtered measurement."""

    def __init__(
        self,
        Kc: float,
        Ti: float | None = None,
        Td: float = 0.0,
        dt: float = 1.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        N: float = 10.0,
        Tt: float | None = None,
        beta: float = 1.0,
        u0: float | None = None,
        name: str = "PID",
        tuning_note: str = "unspecified",
    ):
        self.Kc = float(Kc)
        self.Ti = None if Ti is None else float(Ti)
        self.Td = float(Td)
        self.dt = float(dt)
        self.u_min = float(u_min)
        self.u_max = float(u_max)
        self.N = float(N)
        self.beta = float(beta)
        self.u0 = u0
        self.name = name
        self.tuning_note = tuning_note

        # Tracking time constant for back-calculation. Astrom's rule of thumb is
        # Tt = sqrt(Ti*Td) for PID, Tt = Ti for PI: fast enough to unwind
        # promptly, slow enough not to cripple integral action.
        if Tt is not None:
            self.Tt = float(Tt)
        elif self.Ti is not None and self.Td > 0:
            self.Tt = float(np.sqrt(self.Ti * self.Td))
        elif self.Ti is not None:
            self.Tt = self.Ti
        else:
            self.Tt = np.inf

        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._y_prev = None
        self._deriv = 0.0
        self._initialised = False

    # -- individual terms ------------------------------------------------
    def _derivative(self, y: float) -> float:
        """Filtered derivative action, -Kc*Td*dy/dt through a lag Td/N."""
        if self.Td <= 0:
            return 0.0
        if self._y_prev is None:
            self._y_prev = y
            return 0.0
        a = self.Td / (self.Td + self.N * self.dt)
        b = self.Kc * self.Td * self.N / (self.Td + self.N * self.dt)
        self._deriv = a * self._deriv - b * (y - self._y_prev)
        self._y_prev = y
        return self._deriv

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        y0 = float(np.atleast_1d(y)[0])
        sp = float(np.atleast_1d(setpoint)[0])
        e = sp - y0

        p_term = self.Kc * (self.beta * sp - y0)
        d_term = self._derivative(y0)

        if not self._initialised:
            # Bumpless start: choose the initial integral so the first output is
            # the nominal valve position, instead of stepping the actuator at t=0.
            if self.u0 is not None:
                self._integral = float(self.u0) - p_term - d_term
            self._initialised = True

        u_unsat = p_term + self._integral + d_term
        u_sat = float(np.clip(u_unsat, self.u_min, self.u_max))

        # Integrate *after* forming the output, then apply back-calculation, so
        # the windup correction uses the clipping that actually happened.
        if self.Ti is not None and self.Ti > 0:
            self._integral += self.Kc * (self.dt / self.Ti) * e
            self._integral += (self.dt / self.Tt) * (u_sat - u_unsat)

        return np.array([u_sat])

    def describe(self) -> dict:
        return {
            "controller": self.name,
            "tuning": self.tuning_note,
            "Kc": self.Kc,
            "Ti": self.Ti,
            "Td": self.Td,
        }


class VelocityPIDController(Controller):
    """Incremental ("velocity") PID: computes the *change* in output.

        du_k = Kc * [ (e_k - e_{k-1}) + (dt/Ti)*e_k - Td*(filtered dy/dt term) ]
        u_k  = clip(u_{k-1} + du_k)

    This is the form most DCS and PLC function blocks actually implement, for
    two practical reasons:

    * **Windup is structurally limited.** There is no integral state to run
      away: the integral lives in ``u_{k-1}``, which is clipped every sample,
      so the controller can never accumulate a demand the actuator cannot
      deliver. Anti-windup is a property of the form rather than a bolt-on.
    * **Bumpless transfer is free.** On switching from manual to automatic the
      controller starts from whatever the operator left the valve at, because
      it only ever adds increments to the current position.

    The trade-off: derivative action is more noise-sensitive in this form
    (it becomes a second difference), and a P-only velocity controller is not
    possible -- without an integral term the output would drift.
    """

    def __init__(
        self,
        Kc: float,
        Ti: float,
        Td: float = 0.0,
        dt: float = 1.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        N: float = 10.0,
        u0: float = 0.0,
        name: str = "velocity PID",
        tuning_note: str = "unspecified",
    ):
        if Ti is None or Ti <= 0:
            raise ValueError("the velocity form requires integral action")
        self.Kc, self.Ti, self.Td = float(Kc), float(Ti), float(Td)
        self.dt, self.N = float(dt), float(N)
        self.u_min, self.u_max = float(u_min), float(u_max)
        self.u0 = float(u0)
        self.name = name
        self.tuning_note = tuning_note
        self.reset()

    def reset(self) -> None:
        self._u = float(np.clip(self.u0, self.u_min, self.u_max))
        self._e_prev = None
        self._y_prev = None
        self._deriv = 0.0

    def compute(self, y: np.ndarray, setpoint: np.ndarray, t: float) -> np.ndarray:
        y0 = float(np.atleast_1d(y)[0])
        sp = float(np.atleast_1d(setpoint)[0])
        e = sp - y0

        if self._e_prev is None:
            self._e_prev, self._y_prev = e, y0
            return np.array([self._u])

        du = self.Kc * ((e - self._e_prev) + (self.dt / self.Ti) * e)

        if self.Td > 0:
            a = self.Td / (self.Td + self.N * self.dt)
            b = self.Kc * self.Td * self.N / (self.Td + self.N * self.dt)
            deriv = a * self._deriv - b * (y0 - self._y_prev)
            du += deriv - self._deriv          # increment of the derivative term
            self._deriv = deriv

        self._u = float(np.clip(self._u + du, self.u_min, self.u_max))
        self._e_prev, self._y_prev = e, y0
        return np.array([self._u])

    def describe(self) -> dict:
        return {
            "controller": self.name, "tuning": self.tuning_note,
            "Kc": self.Kc, "Ti": self.Ti, "Td": self.Td, "form": "velocity",
        }

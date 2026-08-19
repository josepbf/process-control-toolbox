"""Boiler drum level: shrink and swell, i.e. inverse response.

Two parallel paths from the same input, with opposite signs and different
speeds:

    tau_s * dx_s/dt = -x_s + K_s * u(t-theta)     slow, positive: added mass
    tau_f * dx_f/dt = -x_f + K_f * u(t-theta)     fast, negative: bubble collapse
    y = x_s + x_f

Physically: opening feedwater adds mass to the drum, so the level must
eventually rise -- but the incoming water is colder, it collapses the steam
bubbles below the surface, and the level *falls first*. Equivalently, the
transfer function has a zero in the right half plane.

This is the process that punishes feedback hardest, and for a reason worth
stating precisely: for the first few seconds the measurement moves the *wrong
way*, so a controller that responds to what it sees makes things worse, and
increasing the gain makes it worse faster. An RHP zero puts a hard ceiling on
achievable bandwidth that no amount of tuning can lift -- the only escape is to
know in advance that the dip is coming, which is what a model-based controller
has and a PID does not.

Configured by default in the drum-level regime: net gain positive, initial
response negative. It is also the phase-3 non-minimum-phase story in miniature,
on a SISO loop where it can be isolated.
"""

from __future__ import annotations

import numpy as np

from .base import Plant
from ..tuning.rules import half_rule


class InverseResponseTank(Plant):
    """Two opposing first-order paths; inverse response when the fast path wins
    initially. The RHP zero sits at ``1/T_z`` with ``T_z`` from :meth:`zero`."""

    def __init__(
        self,
        K_slow: float = 1.5,
        tau_slow: float = 60.0,
        K_fast: float = -0.5,
        tau_fast: float = 5.0,
        theta: float = 2.0,
        Kd: float = 1.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        noise_std: float = 0.0,
        y_min: float | None = None,
        y_max: float | None = None,
        seed: int = 0,
    ):
        self.K_slow, self.tau_slow = float(K_slow), float(tau_slow)
        self.K_fast, self.tau_fast = float(K_fast), float(tau_fast)
        self.theta, self.Kd = float(theta), float(Kd)

        super().__init__(
            n_states=2, n_inputs=1, n_outputs=1,
            u_min=u_min, u_max=u_max, x0=np.zeros(2), dead_time=theta,
            noise_std=noise_std, y_min=y_min, y_max=y_max, seed=seed,
        )

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        # The disturbance is in input-equivalent units and travels both paths:
        # a steam-demand change on a real drum causes shrink and swell too.
        drive = u[0] + self.Kd * d[0]
        return np.array(
            [
                (-x[0] + self.K_slow * drive) / self.tau_slow,
                (-x[1] + self.K_fast * drive) / self.tau_fast,
            ]
        )

    def output(self, x: np.ndarray) -> np.ndarray:
        return np.array([x[0] + x[1]])

    @property
    def K(self) -> float:
        """Net steady-state gain."""
        return self.K_slow + self.K_fast

    def steady_input(self, y_target: float) -> float:
        return y_target / self.K

    @property
    def has_inverse_response(self) -> bool:
        """True when the initial slope opposes the final direction.

        Initial slope of the step response is ``K_s/tau_s + K_f/tau_f``; the
        final value is ``K_s + K_f``. Inverse response is exactly the case
        where those two disagree in sign.
        """
        initial_slope = self.K_slow / self.tau_slow + self.K_fast / self.tau_fast
        return np.sign(initial_slope) != np.sign(self.K)

    @property
    def zero(self) -> float:
        """Time constant ``T_z`` of the numerator zero, in seconds.

        Summing the two paths gives numerator
        ``K_s*(tau_f s + 1) + K_f*(tau_s s + 1)``, so the zero is at
        ``s = -(K_s + K_f) / (K_s*tau_f + K_f*tau_s)``. A *positive* ``T_z``
        here means a right-half-plane zero.
        """
        return -(self.K_slow * self.tau_fast + self.K_fast * self.tau_slow) / self.K

    def fopdt_half_rule(self, dt: float = 0.0) -> dict:
        """FOPDT approximation for classical tuning.

        The half rule sends a right-half-plane zero entirely into the effective
        dead time -- which is the correct instinct: an inverse response costs
        you the same thing dead time costs you, the inability to act on what
        you have not yet seen.
        """
        tau_eff, theta_eff = half_rule(
            [self.tau_slow, self.tau_fast],
            theta=self.theta,
            inverse_zeros=[self.zero] if self.zero > 0 else [],
            dt=dt,
        )
        return {"K": self.K, "tau": tau_eff, "theta": theta_eff}

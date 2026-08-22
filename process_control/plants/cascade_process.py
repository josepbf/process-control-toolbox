"""Two-stage process with an inner disturbance: the case cascade exists for.

    inner (fast):  tau_1 * dx_1/dt = -x_1 + K_1 * u(t-theta_1) + Kd_1 * d_inner
    outer (slow):  tau_2 * dx_2/dt = -x_2 + K_2 * x_1          + Kd_2 * d_outer

    y = [x_2 (primary, controlled), x_1 (secondary, measured)]

Concretely: ``u`` is a valve position, ``x_1`` the fuel or water *flow* it
produces, and ``x_2`` the temperature that flow drives. ``d_inner`` is a header
pressure change -- the same valve position now delivers a different flow.

The structural point: ``d_inner`` enters *before* the slow lag. A single loop
measuring only ``x_2`` cannot know anything has happened until the upset has
worked through ``tau_2``, by which time the temperature has already moved. A
secondary controller on ``x_1`` sees it within ``tau_1`` and corrects it before
the primary variable is affected at all. Cascade is not a tuning trick; it buys
information, by adding a measurement.

The rule of thumb it depends on: the inner loop must be substantially faster
than the outer one (a factor of 3-5 in time constant is the usual guidance),
otherwise the two loops interact and the cascade is worse than useless.
"""

from __future__ import annotations

import numpy as np

from .base import Plant


class CascadeProcess(Plant):
    """Fast inner stage feeding a slow outer stage; disturbances on both."""

    def __init__(
        self,
        K1: float = 1.0,
        tau1: float = 5.0,
        K2: float = 1.2,
        tau2: float = 60.0,
        theta: float = 2.0,
        Kd_inner: float = 1.0,
        Kd_outer: float = 1.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        y_dead_time: tuple[float, float] = (20.0, 0.0),
        noise_std: tuple[float, float] = (0.0, 0.0),
        x0: np.ndarray | None = None,
        seed: int = 0,
    ):
        self.K1, self.tau1 = float(K1), float(tau1)
        self.K2, self.tau2 = float(K2), float(tau2)
        self.theta = float(theta)
        self.Kd_inner, self.Kd_outer = float(Kd_inner), float(Kd_outer)
        # The primary (temperature, quality) is measured with a lag; the
        # secondary (flow) is not. That asymmetry is the practical reason a
        # cascade pays for itself.
        self.y_theta = tuple(float(v) for v in y_dead_time)

        super().__init__(
            n_states=2, n_inputs=1, n_outputs=2, n_disturbances=2,
            u_min=u_min, u_max=u_max,
            x0=np.zeros(2) if x0 is None else x0,
            dead_time=theta, y_dead_time=y_dead_time, noise_std=noise_std, seed=seed,
        )

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        d_inner = d[0]
        d_outer = d[1] if d.size > 1 else 0.0
        return np.array(
            [
                (-x[0] + self.K1 * u[0] + self.Kd_inner * d_inner) / self.tau1,
                (-x[1] + self.K2 * x[0] + self.Kd_outer * d_outer) / self.tau2,
            ]
        )

    def output(self, x: np.ndarray) -> np.ndarray:
        """Primary first, secondary second: ``y = [x_2, x_1]``."""
        return np.array([x[1], x[0]])

    def steady_input(self, y_target: float) -> float:
        return y_target / (self.K1 * self.K2)

    @property
    def inner_fopdt(self) -> dict:
        """What the secondary controller sees: valve -> flow, measured now."""
        return {"K": self.K1, "tau": self.tau1, "theta": self.theta + self.y_theta[1]}

    @property
    def outer_fopdt(self) -> dict:
        """What the primary controller sees *once the inner loop is closed and
        fast*: flow setpoint -> temperature. The inner loop is approximated as
        unity gain with its closed-loop lag folded into the dead time, which is
        the standard cascade design assumption."""
        return {
            "K": self.K2, "tau": self.tau2,
            "theta": self.theta + self.tau1 + self.y_theta[0],
        }

    @property
    def single_loop_fopdt(self) -> dict:
        """What a single loop sees: valve -> temperature, both lags in series,
        reduced by the half rule."""
        from ..tuning.rules import half_rule

        tau_eff, theta_eff = half_rule(
            [self.tau2, self.tau1], theta=self.theta + self.y_theta[0]
        )
        return {"K": self.K1 * self.K2, "tau": tau_eff, "theta": theta_eff}

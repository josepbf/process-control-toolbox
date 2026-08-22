"""N first-order lags in series -- a high-order, self-regulating process.

    tau_i * dx_i/dt = -x_i + x_{i-1},     x_0 = K * u(t - theta)

Three tanks draining into each other, a multi-pass heat exchanger, a bank of
cyclones, or the thermal mass chain of a kiln shell: real processes are rarely
first order, and the extra lags behave, from the controller's point of view,
very much like additional dead time.

That is what makes this plant useful here: it is the honest test of Skogestad's
half rule (``tuning.rules.half_rule``), which reduces the true high-order model
to the two-parameter FOPDT form that every classical tuning rule needs. The
reduction is not a numerical convenience -- it is the moment where structure
the controller cannot represent gets swept into an effective dead time, and
dead time is exactly what feedback handles worst.
"""

from __future__ import annotations

import numpy as np

from .base import Plant
from ..tuning.rules import half_rule


class SeriesTanks(Plant):
    """A chain of first-order lags, optionally with transport delay."""

    def __init__(
        self,
        K: float = 1.0,
        taus: tuple[float, ...] = (40.0, 20.0, 10.0),
        theta: float = 0.0,
        Kd: float = 1.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        noise_std: float = 0.0,
        y_min: float | None = None,
        y_max: float | None = None,
        x0: np.ndarray | None = None,
        seed: int = 0,
    ):
        self.K = float(K)
        self.taus = np.array([float(t) for t in taus])
        self.theta = float(theta)
        self.Kd = float(Kd)
        n = len(self.taus)

        super().__init__(
            n_states=n, n_inputs=1, n_outputs=1,
            u_min=u_min, u_max=u_max,
            x0=np.zeros(n) if x0 is None else x0,
            dead_time=theta, noise_std=noise_std,
            y_min=y_min, y_max=y_max, seed=seed,
        )

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        driving = np.empty_like(x)
        driving[0] = self.K * u[0] + self.Kd * d[0]
        driving[1:] = x[:-1]
        return (driving - x) / self.taus

    def output(self, x: np.ndarray) -> np.ndarray:
        return np.array([x[-1]])

    def steady_input(self, y_target: float) -> float:
        return y_target / self.K

    @property
    def fopdt_true(self) -> dict:
        """The *exact* high-order description, for reference."""
        return {"K": self.K, "taus": self.taus.tolist(), "theta": self.theta}

    def fopdt_half_rule(self, dt: float = 0.0) -> dict:
        """FOPDT approximation by Skogestad's half rule -- what a classical
        tuning rule is actually given when it is applied to this plant."""
        tau_eff, theta_eff = half_rule(self.taus.tolist(), theta=self.theta, dt=dt)
        return {"K": self.K, "tau": tau_eff, "theta": theta_eff}

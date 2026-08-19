"""SISO buffer tank: first order plus dead time (FOPDT).

This is the workhorse test process of classical control. The level ``h`` [%] is
driven by an inlet valve ``u`` [%] whose effect arrives ``theta`` seconds late
(pipe run / conveyor transport), and is pushed around by a load disturbance
``d`` (an unmeasured outlet demand).

    tau * dh/dt = -h + K * u(t - theta) + Kd * d(t)

The FOPDT form matters because nearly every classical tuning rule (SIMC, IMC,
Ziegler-Nichols, Cohen-Coon) is stated in terms of exactly these three numbers:
gain ``K``, time constant ``tau``, dead time ``theta``. Their ratio
``theta / tau`` is the single best predictor of how hard a loop is to control:
below ~0.2 almost anything works, above ~1 feedback alone struggles and
deadtime compensation (Smith predictor, MPC) starts to pay.
"""

from __future__ import annotations

import numpy as np

from .base import Plant


class Tank(Plant):
    """First-order-plus-dead-time level process."""

    def __init__(
        self,
        K: float = 1.5,
        tau: float = 60.0,
        theta: float = 15.0,
        Kd: float = 1.0,
        h0: float = 30.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        noise_std: float = 0.0,
        y_min: float | None = None,
        y_max: float | None = None,
        seed: int = 0,
    ):
        self.K = float(K)
        self.tau = float(tau)
        self.theta = float(theta)
        self.Kd = float(Kd)

        super().__init__(
            n_states=1,
            n_inputs=1,
            n_outputs=1,
            u_min=u_min,
            u_max=u_max,
            x0=[h0],
            dead_time=theta,
            noise_std=noise_std,
            y_min=y_min,
            y_max=y_max,
            seed=seed,
        )
        # Prime the delay line with the input that holds the initial level, so
        # the run does not start with a hidden step buried in the pipeline.
        self.u_init = self.saturate(np.array([h0 / self.K]))

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        return np.array([(-x[0] + self.K * u[0] + self.Kd * d[0]) / self.tau])

    def steady_input(self, y_target: float) -> float:
        """Valve position that holds ``y_target`` with no disturbance."""
        return y_target / self.K

    @property
    def fopdt(self) -> dict:
        """Parameters a tuning rule needs. This is the *true* model; a
        controller is only entitled to it when we deliberately grant a
        no-mismatch upper bound (see fairness rule 3)."""
        return {"K": self.K, "tau": self.tau, "theta": self.theta}

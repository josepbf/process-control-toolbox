"""SISO buffer tank: first order plus dead time (FOPDT).

This is the workhorse test process of classical control. The level ``h`` [%] is
driven by an inlet valve ``u`` [%] whose effect arrives ``theta`` seconds late
(pipe run / conveyor transport), and is pushed around by a load disturbance
``d`` (an unmeasured outlet demand).

    tau   * dh/dt  = -h  + K  * u(t - theta)
    tau_d * dx_d/dt = -x_d + Kd * d(t - theta_d)
    y = h + x_d

The disturbance travels its *own* first-order path with its own gain, time
constant and transport delay. That separation is not cosmetic: whether
feedforward compensation is even realisable depends on the relationship between
the two paths (``theta_d >= theta``), and the compensator itself is the ratio
of the two transfer functions. With the defaults (``tau_d = tau``,
``theta_d = 0``) the two-path model is identical to the textbook single-state
form.

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
        tau_d: float | None = None,
        theta_d: float = 0.0,
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
        self.tau_d = float(tau) if tau_d is None else float(tau_d)
        self.theta_d = float(theta_d)

        super().__init__(
            n_states=2,
            n_inputs=1,
            n_outputs=1,
            u_min=u_min,
            u_max=u_max,
            x0=[h0, 0.0],
            dead_time=theta,
            d_dead_time=theta_d,
            noise_std=noise_std,
            y_min=y_min,
            y_max=y_max,
            seed=seed,
        )
        # Prime the delay line with the input that holds the initial level, so
        # the run does not start with a hidden step buried in the pipeline.
        self.u_init = self.saturate(np.array([h0 / self.K]))

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        return np.array(
            [
                (-x[0] + self.K * u[0]) / self.tau,
                (-x[1] + self.Kd * d[0]) / self.tau_d,
            ]
        )

    def output(self, x: np.ndarray) -> np.ndarray:
        """Level = the manipulated contribution plus the disturbance contribution."""
        return np.array([x[0] + x[1]])

    def steady_input(self, y_target: float) -> float:
        """Valve position that holds ``y_target`` with no disturbance."""
        return y_target / self.K

    @property
    def fopdt_disturbance(self) -> dict:
        """The disturbance path ``d -> y``, in the same FOPDT form. This is the
        model a feedforward compensator needs, and it is a *different* model
        from the one the feedback controller is tuned on."""
        return {"K": self.Kd, "tau": self.tau_d, "theta": self.theta_d}

    @property
    def fopdt(self) -> dict:
        """Parameters a tuning rule needs. This is the *true* model; a
        controller is only entitled to it when we deliberately grant a
        no-mismatch upper bound (see fairness rule 3)."""
        return {"K": self.K, "tau": self.tau, "theta": self.theta}

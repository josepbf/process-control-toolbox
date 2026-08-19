"""Surge tank: an *integrating* level process with no self-regulation.

    dh/dt = k' * (u(t - theta) - u_bias) + Kd * d(t)

The distinction from `Tank` is the whole point. A self-regulating process
settles somewhere on its own when you leave the valve alone; an integrating one
does not -- the level ramps until something stops it. Surge tanks, drum levels,
silos, mill hold-ups and kiln beds all behave this way, and they are the most
common loop in a plant.

Two consequences that drive the experiments:

* There is no ``tau`` to cancel, so any tuning rule that sets ``Ti`` from a
  process lag is meaningless here. Applied anyway, it produces an integral time
  far too long and the loop rolls slowly around setpoint for hours -- the
  classic badly tuned level loop, seen in nearly every plant.
* Tight control is often the *wrong objective*. A surge tank exists to absorb
  flow variation so the downstream unit sees a steady feed; a controller that
  holds level perfectly passes every inlet upset straight through and defeats
  the equipment. See ``tuning.rules.averaging_level_pi``.
"""

from __future__ import annotations

import numpy as np

from .base import Plant


class IntegratingTank(Plant):
    """Level that integrates net flow, with transport delay on the inlet."""

    def __init__(
        self,
        k_prime: float = 0.02,
        u_bias: float = 50.0,
        theta: float = 10.0,
        Kd: float = -0.02,
        h0: float = 50.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        noise_std: float = 0.0,
        y_min: float | None = None,
        y_max: float | None = None,
        seed: int = 0,
    ):
        self.k_prime = float(k_prime)   # %% level per second per %% valve
        self.u_bias = float(u_bias)     # valve position at which net flow is zero
        self.theta = float(theta)
        self.Kd = float(Kd)

        super().__init__(
            n_states=1, n_inputs=1, n_outputs=1,
            u_min=u_min, u_max=u_max, x0=[h0], dead_time=theta,
            noise_std=noise_std, y_min=y_min, y_max=y_max, seed=seed,
        )
        self.u_init = self.saturate(np.array([self.u_bias]))

    def dynamics(self, x: np.ndarray, u: np.ndarray, d: np.ndarray) -> np.ndarray:
        return np.array([self.k_prime * (u[0] - self.u_bias) + self.Kd * d[0]])

    def steady_input(self, y_target: float) -> float:
        """Any level is held by the same valve position: the balance point."""
        return self.u_bias

    @property
    def integrating(self) -> dict:
        """Parameters for ``simc_integrating`` / ``averaging_level_pi``."""
        return {"k_prime": self.k_prime, "theta": self.theta}

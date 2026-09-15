"""Named, cited tuning for MPC -- because a knob somebody turned is not a rule.

Fairness rule 1 applies to model predictive control exactly as it applies to a
PI: a controller whose weights were adjusted until the plot looked good is not
admissible as a result. An MPC has more knobs than a PI, not fewer, so it needs
this more.

The rule here is Shridhar and Cooper's analytical tuning for unconstrained DMC,
which is stated -- like SIMC, IMC and Cohen-Coon before it -- in terms of the
same three FOPDT numbers every classical rule uses. That is the point of it:
the MPC and the PI it is being compared against are then tuned from *the same
model*, so a difference between them is a difference of structure rather than of
how much somebody knew about the plant.

References
----------
Shridhar, R., Cooper, D.J. (1997). "A tuning strategy for unconstrained SISO
    model predictive control." Ind. Eng. Chem. Res. 36(3), 729-746.
    doi:10.1021/ie9604280
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class MPCTuning:
    """Horizons and weights from a named rule, ready to splat into `LinearMPC`.

    Mirrors `tuning.rules.PIDTuning`, which exists so that an experiment reads
    ``LinearMPC(model, **shridhar_cooper(**plant.fopdt).as_kwargs())`` in the
    same shape as ``PIDController(**simc_pi(**plant.fopdt).as_kwargs())``.
    """

    N: int
    M: int
    Q: float
    R: float
    rule: str

    def as_kwargs(self) -> dict:
        return {"N": self.N, "M": self.M, "Q": self.Q, "R": self.R}

    def to_dict(self) -> dict:
        return {"N": self.N, "M": self.M, "Q": self.Q, "R": self.R, "rule": self.rule}


def shridhar_cooper(
    K: float,
    tau: float,
    theta: float = 0.0,
    dt: float = 1.0,
    M: int = 1,
) -> MPCTuning:
    """Analytical DMC tuning from an FOPDT model (Shridhar & Cooper 1997).

    The rule, in their notation with sample time ``T``:

    - discrete dead time      ``k = theta/T + 1``
    - horizons                ``P = N = 5 tau/T + k``, rounded up -- the process
      settling time in samples, which is the same "cover the settling" argument
      `LinearMPC` makes for its prediction horizon
    - move suppression        ``f = (M/500)(3.5 tau/T + 2 - (M-1)/2)``, then
      ``lambda = f K^2``, and ``lambda = 0`` for ``M = 1``

    The ``K^2`` is the part worth noticing: move suppression has to scale with
    the square of the process gain because the cost compares an output error
    against an input move, and those are in different units. A weight that
    worked on one loop is meaningless on another without it -- which is exactly
    why a raw ``R`` copied between experiments is not a tuning rule.

    ``M = 1`` gives ``lambda = 0``, which `LinearMPC` will refuse: a zero
    move-suppression weight makes the QP non-strictly-convex. The rule is
    derived for the unconstrained case where that is benign; here it is not, so
    ``M = 1`` is reported with a small positive floor and the substitution is
    named in ``rule``.

    Parameters
    ----------
    K, tau, theta : float
        The FOPDT model, in the same form every other rule in this package
        takes -- so ``shridhar_cooper(**plant.fopdt)`` works.
    dt : float
        Sample time, seconds.
    M : int
        Control horizon, the user's choice. The rule tunes the weight *given* M
        rather than choosing it.
    """
    if tau <= 0:
        raise ValueError(f"tau must be positive, got {tau}")
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")
    if M < 1:
        raise ValueError(f"the control horizon must be at least one move, got {M}")

    k = theta / dt + 1.0
    N = int(math.ceil(5.0 * tau / dt + k))

    note = ""
    if M == 1:
        # The published rule gives exactly zero here. A strictly convex QP needs
        # more than that, so the substitution is made explicit rather than
        # hidden: it is a departure from the cited rule, and it says so.
        R = 1e-6 * K * K
        note = "; lambda = 0 for M = 1 in the rule, floored here to keep the QP strictly convex"
    else:
        f = (M / 500.0) * (3.5 * tau / dt + 2.0 - (M - 1) / 2.0)
        R = f * K * K
        if R <= 0.0:
            # f goes negative for a control horizon long enough to swamp the
            # 3.5 tau/T term. That is outside the rule's range, not a weight.
            raise ValueError(
                f"the move-suppression coefficient came out non-positive (f = {f:.4g}) "
                f"for M = {M} at tau/dt = {tau / dt:g}. The rule is derived for a "
                "control horizon short relative to the settling time; shorten M."
            )

    return MPCTuning(
        N=N,
        M=M,
        Q=1.0,
        R=R,
        rule=(
            f"Shridhar & Cooper (1997) analytical DMC tuning, "
            f"K={K:g}, tau={tau:g}, theta={theta:g}, dt={dt:g}, M={M}"
            f" -> N={N}, R={R:.4g}{note}"
        ),
    )


def settling_horizon(tau: float, theta: float = 0.0, dt: float = 1.0, n_tau: float = 5.0) -> int:
    """Prediction horizon covering the open-loop settling time, in samples.

    ``LinearMPC`` has no terminal cost and no terminal constraint: its stability
    argument is that the horizon is long enough to see the process settle. This
    is that number, and it is the same one the Shridhar-Cooper rule uses.

    A horizon materially shorter than this is not a cheaper controller, it is a
    different and worse one -- it optimises over a window in which the
    consequences of its own moves have not yet arrived, which on a dead-time
    dominant loop is most of them.
    """
    return int(math.ceil(n_tau * tau / dt + theta / dt + 1.0))

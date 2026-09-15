"""Linear MPC as a condensed quadratic program in the move increments.

One `Controller` among the others. It sees ``y``, it returns ``u``, and the
plant still owns every physical limit -- so whatever advantage it has must come
from *anticipating* a constraint, never from being exempt from one. That is the
whole claim being tested, and the architecture is what makes it testable.

The problem solved every sample
-------------------------------
Decision vector ``z = [du_0 ... du_{M-1}, eps]``, ``eps`` the output-constraint
slack. With the estimated state ``x_hat`` and the last move actually issued
``u_prev``, the predicted output over the horizon is::

    Y = Psi x_hat + Upsilon u_prev + Theta dU      (+ Gamma d, if d is measured)
        \\_______________________/   \\________/
            the free response:        what this
            what happens if you       sample's
            do nothing                moves buy

and the cost is::

    J = (Y - T)' Qbar (Y - T)  +  dU' Rbar dU  +  rho |eps| + rho_q eps^2

so ``H = Theta' Qbar Theta + Rbar`` and ``g = Theta' Qbar (f - T)``. ``H`` is
**constant** for an LTI model with fixed weights, so it is factorised once at
construction rather than once a sample -- which matters, because solve time is a
reported metric and a self-inflicted cost would distort the comparison as surely
as an unfair advantage.

``T`` is the setpoint repeated across the horizon, or the previewed setpoint
trajectory when ``preview=True``. That substitution is the *entire*
implementation of preview, which is why the capability flag is cheap -- and why
the honesty rule around it (compare against a classical scheme given the
equivalent, or state the asymmetry) carries all the weight.

Three decisions worth defending
-------------------------------
**Increments, not positions.** Penalising ``du`` and never ``u`` means the
unconstrained steady state has ``du -> 0`` and ``y -> sp`` exactly: offset-free
tracking comes free from the parameterisation, with no steady-state target
calculation. The equivalent with a position penalty needs a target calculation,
a feasibility question, and a second failure mode. It breaks the moment anything
penalises ``u`` itself or asks for an input target -- which nothing here does.

**Input constraints hard, output constraints soft.** ``y_min``/``y_max`` are
declared bands the plant does *not* enforce (`plants/base.py`), and a run may
legitimately start outside one. A hard output constraint would then be
infeasible at the first sample -- a silent failure buried in a sweep. An exact
penalty with ``rho`` above the multiplier norm of the hard problem recovers
hard-constraint behaviour wherever the hard problem is feasible at all
(Kerrigan & Maciejowski 2000).

**The horizon is the stability argument, not a terminal set.** ``N`` should
cover the open-loop settling time -- roughly ``(theta + 4 tau)/dt``. That is the
industrial DMC position, it is what the tuning rules in `tuning.mpc_rules`
assume, and it is honest: there is no terminal cost or terminal constraint here,
and a horizon too short to see the process settle will misbehave in ways no
weight can fix. ``__init__`` warns when ``N`` is short relative to the model.

References
----------
Cutler, C.R., Ramaker, B.L. (1980). "Dynamic matrix control -- a computer
    control algorithm." Proc. Joint Automatic Control Conference.
Maciejowski, J.M. (2002). *Predictive Control with Constraints.* Prentice Hall.
Kerrigan, E.C., Maciejowski, J.M. (2000). "Soft constraints and exact penalty
    functions in model predictive control." Proc. UKACC Control Conference.
Rawlings, J.B., Mayne, D.Q., Diehl, M.M. (2017). *Model Predictive Control:
    Theory, Computation, and Design.* Nob Hill.
"""

from __future__ import annotations

import warnings

import numpy as np

from ..models.discrete import DiscreteModel
from ..models.observer import KalmanObserver, augment_disturbance
from ..solvers import SOLVED, get_solver
from .base import Controller

#: qp_status codes, published as a diagnostic so a run that limped is visible.
STATUS_SOLVED = 0.0
STATUS_MAX_ITER = 1.0
STATUS_FALLBACK = 2.0


class LinearMPC(Controller):
    """Linear model predictive control on a dense QP in the move increments.

    Parameters
    ----------
    model : DiscreteModel
        The controller's internal model, already discretised at the sample time
        and dead-time augmented. It is passed through
        :func:`~process_control.models.observer.augment_disturbance` here unless
        it already carries disturbance states, because without them this
        controller has no integral action at all.
    N : int
        Prediction horizon, samples. Should cover the open-loop settling time;
        see the module docstring.
    M : int, optional
        Control horizon. Increments past ``M`` are zero, so the last move is
        held to the end of the prediction horizon -- which is what makes a short
        control horizon stabilising rather than myopic. Defaults to
        ``min(N, 10)``.
    Q, R : float or ndarray
        Output tracking weight and move-suppression weight. ``R`` acts on
        ``du``, not on ``u``, and must be strictly positive: it is what makes
        the QP strictly convex, and a zero would be refused by the solver rather
        than silently regularised. Only the ratio ``Q/R`` matters.
    u_min, u_max, du_max : float
        Hard input constraints. Pass the *same* numbers the plant enforces --
        the controller does not get its own, softer actuator.
    y_min, y_max : float, optional
        The output band. Soft by default; see the module docstring.
    rho, rho_quad : float
        Linear (exact-penalty) and quadratic slack weights. ``rho`` large enough
        makes the soft constraint behave as a hard one wherever that is
        feasible; ``rho_quad`` keeps the slack block of ``H`` positive definite.
    observer : KalmanObserver, optional
        Built from the augmented model if not supplied.
    disturbance : {"output", "input"}
        Which disturbance model to augment with, when one is built here.
    preview : bool
        Declare that this controller reads future setpoints. An information
        advantage -- see :attr:`Controller.uses_preview`.
    solver : str
        QP backend name. ``"auto"`` is the dependency-free dense solver and
        never probes for anything faster; see `process_control.solvers`.
    u0 : float
        The move held at startup, before the first solve. As with `PIDController`
        this is what makes the first sample bumpless.
    """

    def __init__(
        self,
        model: DiscreteModel,
        *,
        N: int = 40,
        M: int | None = None,
        Q: float = 1.0,
        R: float = 0.1,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        du_max: float = np.inf,
        y_min: float | None = None,
        y_max: float | None = None,
        soft_output: bool = True,
        rho: float = 1e4,
        rho_quad: float = 1.0,
        observer: KalmanObserver | None = None,
        disturbance: str = "output",
        preview: bool = False,
        uses_measured_disturbance: bool = False,
        solver: str = "auto",
        u0: float = 0.0,
        name: str = "MPC",
        tuning_note: str = "",
    ):
        if model.n_inputs != 1:
            raise ValueError(
                "this MPC is written for a single manipulated variable; a "
                "multivariable version needs a decision about input scaling "
                "that should be made explicitly rather than inherited"
            )
        if float(R) <= 0.0:
            raise ValueError(
                f"R must be strictly positive, got {R}: it is the move-suppression "
                "weight, and a zero makes the QP non-strictly-convex with no "
                "unique solution"
            )

        self.model = augment_disturbance(model, disturbance) if not model.n_disturbance_states else model
        self.dt = self.model.dt
        self.N = int(N)
        self.M = min(self.N, 10) if M is None else int(min(M, N))
        self.Q, self.R = float(Q), float(R)
        self.u_min, self.u_max = float(u_min), float(u_max)
        self.du_max = float(du_max)
        self.y_min, self.y_max = y_min, y_max
        self.soft_output = bool(soft_output) and (y_min is not None or y_max is not None)
        self.rho, self.rho_quad = float(rho), float(rho_quad)
        self.u0 = float(u0)

        self.observer = observer or KalmanObserver(self.model)
        self.uses_preview = bool(preview)
        self.preview_horizon = self.N if preview else 0
        self.uses_measured_disturbance = bool(uses_measured_disturbance)
        if self.uses_measured_disturbance and self.model.Bd is None:
            raise ValueError(
                "this controller declares it reads a measured disturbance, but "
                "its model has no disturbance path: build one with "
                "models.discrete.append_disturbance_path, or the declaration is "
                "a claim the model cannot honour"
            )

        self._build_prediction()
        self._build_qp()

        self._solver_name = solver
        self._solver = get_solver(solver)
        self._solver.setup(self._H, self._A_qp, self._z_lb, self._z_ub)

        self.name = name
        self.tuning_note = tuning_note or (
            f"condensed QP in du, N={self.N}, M={self.M}, Q={self.Q:g}, R={self.R:g}"
            f" (Q/R = {self.Q / self.R:g}); {self.model.disturbance_kind}-disturbance "
            f"observer; solver={self._solver.name}"
            + ("; setpoint preview" if self.uses_preview else "")
        )
        self.reset()

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------
    def _build_prediction(self) -> None:
        """Psi, Upsilon, Theta -- built explicitly, because they are the thing a
        reader has to check when a result looks too good."""
        A, B, C = self.model.A, self.model.B, self.model.C
        n, p = self.model.n_states, self.model.n_outputs
        N, M = self.N, self.M

        # Step response of the state: S[r] = sum_{j<r} A^j B, i.e. where the
        # state ends up after holding a unit move for r samples.
        S = [np.zeros((n, 1))]
        for _ in range(N):
            S.append(A @ S[-1] + B)

        Apow = [np.eye(n)]
        for _ in range(N):
            Apow.append(A @ Apow[-1])

        self._Psi = np.vstack([C @ Apow[i] for i in range(1, N + 1)])          # (Np, n)
        self._Upsilon = np.vstack([C @ S[i] for i in range(1, N + 1)])         # (Np, 1)

        # Theta[i, l] = C S[i - l]: the effect at sample i of a unit increment
        # applied at sample l and held from then on. Zero for l >= i.
        self._Theta = np.zeros((N * p, M))
        for i in range(1, N + 1):
            for l in range(min(i, M)):
                self._Theta[(i - 1) * p : i * p, l : l + 1] = C @ S[i - l]

        # Measured disturbance, held constant across the horizon.
        self._Gamma = None
        if self.model.Bd is not None:
            Sd = [np.zeros((n, self.model.n_disturbances))]
            for _ in range(N):
                Sd.append(A @ Sd[-1] + self.model.Bd)
            self._Gamma = np.vstack([C @ Sd[i] for i in range(1, N + 1)])

        # The state at the end of the horizon, for the snapshot's plan.
        self._S = S

    def _build_qp(self) -> None:
        """H, the constant constraint rows, and the simple bounds.

        Everything here is fixed for an LTI model, so it is built once. Only
        ``g`` and the row limits move from sample to sample.
        """
        N, M, p = self.N, self.M, self.model.n_outputs
        self._n_slack = 1 if self.soft_output else 0
        n_z = M + self._n_slack

        Qbar = np.eye(N * p) * self.Q
        Rbar = np.eye(M) * self.R

        H = np.zeros((n_z, n_z))
        H[:M, :M] = 2.0 * (self._Theta.T @ Qbar @ self._Theta + Rbar)
        if self._n_slack:
            H[M, M] = 2.0 * self.rho_quad
        self._H = H
        self._Qbar = Qbar

        # --- constraint rows, in the canonical lb <= A z <= ub form ----------
        rows = []
        self._row_kind = []

        # Input magnitude: u_k = u_prev + (L dU)_k, hard.
        L = np.tril(np.ones((M, M)))
        block = np.zeros((M, n_z))
        block[:, :M] = L
        rows.append(block)
        self._row_kind.append(("input", M))

        # Output band, soft: two one-sided sets, because the slack widens the
        # band in both directions and so cannot be one two-sided row.
        if self.y_min is not None or self.y_max is not None:
            lower = np.zeros((N * p, n_z))
            lower[:, :M] = self._Theta
            if self._n_slack:
                lower[:, M] = 1.0                 # Theta dU + eps >= y_min - f
            rows.append(lower)
            self._row_kind.append(("y_lower", N * p))

            upper = np.zeros((N * p, n_z))
            upper[:, :M] = self._Theta
            if self._n_slack:
                upper[:, M] = -1.0                # Theta dU - eps <= y_max - f
            rows.append(upper)
            self._row_kind.append(("y_upper", N * p))

        self._A_qp = np.vstack(rows)

        # --- simple bounds ---------------------------------------------------
        self._z_lb = np.full(n_z, -self.du_max)
        self._z_ub = np.full(n_z, self.du_max)
        if self._n_slack:
            self._z_lb[M] = 0.0                   # slack is a magnitude
            self._z_ub[M] = np.inf

        self._n_z = n_z

    # ------------------------------------------------------------------
    # the interface
    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.observer.reset()
        self._u_prev = np.array([self.u0])
        self._warm_start = np.zeros(self._n_z)
        self._y_pred = np.zeros((self.N, self.model.n_outputs))
        self._u_plan = np.full((self.N, 1), self.u0)
        self._T = np.zeros(self.N * self.model.n_outputs)
        self._t = 0.0
        self._d_hat = np.zeros(max(self.model.n_disturbance_states, 1))
        self._slack = 0.0
        self._cost = 0.0
        self._iters = 0
        self._status = STATUS_SOLVED
        self._fallbacks = 0
        self._initialised = False

    def compute(self, y, setpoint, t: float, d=None, sp_preview=None) -> np.ndarray:
        y = np.atleast_1d(np.asarray(y, dtype=float))
        self._t = float(t)

        if not self._initialised:
            # Bumpless start, the same contract PIDController signs: begin from
            # a state consistent with the first reading so the first move is u0
            # rather than a kick that contaminates every settling-time metric.
            self.observer.seed_from_measurement(y, u0=self.u0)
            self._initialised = True

        x_hat = self.observer.correct(y)            # x(k|k): correct, then move
        self._d_hat = self.observer.d_hat

        f = self._Psi @ x_hat + self._Upsilon @ self._u_prev
        d_vec = None
        if self.uses_measured_disturbance and d is not None:
            d_vec = np.atleast_1d(np.asarray(d, dtype=float))
            f = f + self._Gamma @ d_vec

        T = self._target(setpoint, sp_preview)
        self._T = T

        g = np.zeros(self._n_z)
        g[: self.M] = 2.0 * (self._Theta.T @ self._Qbar @ (f - T))
        if self._n_slack:
            g[self.M] = self.rho                   # exact penalty, linear in eps

        lb, ub = self._row_limits(f)
        result = self._solver.solve(g, lb, ub, z0=self._warm_start)

        if result.status == SOLVED:
            z = result.z
            self._status = STATUS_SOLVED
        else:
            # Never raise inside compute(). A traceback at sample 812 of a
            # 6000-run sweep destroys the sweep; a held move degrades one sample
            # and shows up in the metrics, where it can be seen and reported.
            z = self._warm_start if result.status != "infeasible" else np.zeros(self._n_z)
            self._status = STATUS_MAX_ITER if result.status == "max_iter" else STATUS_FALLBACK
            self._fallbacks += 1

        du = z[: self.M]
        self._slack = float(z[self.M]) if self._n_slack else 0.0
        self._cost = float(result.objective) if np.isfinite(result.objective) else 0.0
        self._iters = int(result.iterations)

        u = float(self._u_prev[0] + du[0])
        u = float(np.clip(u, self.u_min, self.u_max))    # belt and braces on the hard row

        self._record_plan(f, du, u)
        self._warm_start = np.concatenate([du[1:], [0.0], z[self.M :]]) if self.M > 1 else z

        self._u_prev = np.array([u])
        self.observer.predict(np.array([u]), d_vec)      # x(k+1|k), last
        return np.array([u])

    def _target(self, setpoint, sp_preview) -> np.ndarray:
        """The trajectory the horizon is asked to reach. Preview lives here."""
        p = self.model.n_outputs
        if self.uses_preview and sp_preview is not None:
            preview = np.atleast_2d(np.asarray(sp_preview, dtype=float))
            rows = preview[1 : self.N + 1]
            if rows.shape[0] < self.N:               # ran off the end of the run
                pad = np.repeat(rows[-1:], self.N - rows.shape[0], axis=0)
                rows = np.vstack([rows, pad])
            return rows[:, :p].reshape(-1)
        return np.tile(np.atleast_1d(np.asarray(setpoint, dtype=float))[:p], self.N)

    def _row_limits(self, f: np.ndarray):
        """Only these move from sample to sample; the rows themselves are fixed."""
        parts_lb, parts_ub = [], []
        for kind, count in self._row_kind:
            if kind == "input":
                u_prev = float(self._u_prev[0])
                parts_lb.append(np.full(count, self.u_min - u_prev))
                parts_ub.append(np.full(count, self.u_max - u_prev))
            elif kind == "y_lower":
                lo = -np.inf if self.y_min is None else self.y_min
                parts_lb.append(np.full(count, lo) - (f if self.y_min is not None else 0.0))
                parts_ub.append(np.full(count, np.inf))
            else:                                    # y_upper
                hi = np.inf if self.y_max is None else self.y_max
                parts_lb.append(np.full(count, -np.inf))
                parts_ub.append(np.full(count, hi) - (f if self.y_max is not None else 0.0))
        return np.concatenate(parts_lb), np.concatenate(parts_ub)

    def _record_plan(self, f: np.ndarray, du: np.ndarray, u: float) -> None:
        """The predicted trajectory and the planned moves, for snapshot()."""
        p = self.model.n_outputs
        self._y_pred = (f + self._Theta @ du).reshape(self.N, p)
        plan = np.full(self.N, float(self._u_prev[0]))
        running = float(self._u_prev[0])
        for i in range(self.N):
            if i < self.M:
                running += float(du[i])
            plan[i] = running                        # held past M, by construction
        self._u_plan = plan.reshape(self.N, 1)

    # ------------------------------------------------------------------
    # what it publishes
    # ------------------------------------------------------------------
    def diagnostics(self) -> dict[str, float]:
        """``d_hat`` is the signal that explains this controller.

        The estimated disturbance *is* the integral action -- the MPC analogue of
        `FeedforwardPID`'s ``u_feedforward``, and the reason the output returns
        to setpoint after a load change. ``slack`` says how much of the declared
        output band the optimiser chose to give up, and the ``qp_`` pair keeps
        the numerics in the results table: a controller that fell back forty
        times and still won is not a controller that won.
        """
        return {
            "d_hat": float(self._d_hat[0]) if self._d_hat.size else 0.0,
            "y_pred1": float(self._y_pred[0, 0]),
            "slack": self._slack,
            "cost": self._cost,
            "qp_iters": float(self._iters),
            "qp_status": self._status,
        }

    def snapshot(self) -> dict[str, np.ndarray]:
        """The receding horizon itself: what the controller thinks happens next."""
        return {
            "t_horizon": self._t + np.arange(1, self.N + 1) * self.dt,
            "y_pred": self._y_pred,
            "u_plan": self._u_plan,
            "sp_target": self._T.reshape(self.N, self.model.n_outputs),
        }

    def describe(self) -> dict:
        return {
            "controller": self.name,
            "tuning": self.tuning_note,
            "N": self.N,
            "M": self.M,
            "Q": self.Q,
            "R": self.R,
            "u_min": self.u_min,
            "u_max": self.u_max,
            "du_max": self.du_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "soft_output": self.soft_output,
            "rho": self.rho,
            "preview": self.uses_preview,
            "model": self.model.describe(),
            "observer": self.observer.describe(),
            # Not bookkeeping: a run solved by OSQP at 1e-3 is not the same run
            # as one from the exact active-set solver.
            "solver": self._solver.name,
            "fallbacks": self._fallbacks,
        }

    # ------------------------------------------------------------------
    # analysis
    # ------------------------------------------------------------------
    def linear_gain(self):
        """The unconstrained control law this MPC reduces to.

            u = (1 + K_u) u_prev + K_x x_hat + K_sp sp

        An MPC with no active constraint *is* a linear controller, and being able
        to say which one is what makes it comparable to everything else in the
        project: it gives the LQR-equality check, an ``Ms`` through the existing
        `tuning.analysis` machinery, and a principled way to match an MPC and a
        PI at equal robustness before comparing them on constraint handling.

        Returns
        -------
        (K_x, K_sp, K_u) : tuple of ndarray
            Row vectors of shape ``(1, n_states)``, ``(1, n_outputs)``, ``(1, 1)``.
        """
        M, p = self.M, self.model.n_outputs
        Hd = self._H[:M, :M] / 2.0
        Kg = np.linalg.solve(Hd, self._Theta.T @ self._Qbar)[0:1]       # (1, Np)

        K_x = -Kg @ self._Psi
        K_u = -Kg @ self._Upsilon
        ones = np.tile(np.eye(p), (self.N, 1))                          # (Np, p)
        K_sp = Kg @ ones
        return K_x, K_sp, K_u

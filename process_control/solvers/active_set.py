"""A dense dual active-set QP solver (Goldfarb & Idnani 1983).

Solves::

    minimise   0.5 z' H z + g' z
    subject to lb <= A z <= ub,   z_lb <= z <= z_ub

with ``H`` symmetric positive definite.

Why this algorithm rather than an ADMM-in-eighty-lines, which would have been
shorter: it starts from the **unconstrained minimiser** and admits constraints
only as they are found to be violated. Three consequences, all of which matter
more here than raw speed:

1. With no constraint active it terminates immediately at the exact answer. So
   "an unconstrained MPC is a linear controller, and here is which one" is
   testable to machine precision rather than to a solver tolerance.
2. It terminates finitely at the exact solution, which makes it the *reference*
   the optional accelerators are checked against -- not the other way round.
3. The iteration count means something readable: how many limits the process
   forced on the controller this sample. That is worth publishing as a
   diagnostic, and an ADMM iteration count is not.

The problems here are small and dense -- twenty to sixty variables, a hundred or
so rows -- so dense linear algebra is correct and sparsity machinery would be
dead code.

References
----------
Goldfarb, D., Idnani, A. (1983). "A numerically stable dual method for solving
    strictly convex quadratic programs." Mathematical Programming 27, 1-33.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import cho_factor, cho_solve

# Termination statuses, shared with the other backends.
SOLVED = "solved"
MAX_ITER = "max_iter"
INFEASIBLE = "infeasible"


def _as_rows(A, lb, ub, z_lb, z_ub, n):
    """Fold two-sided rows and simple bounds into one-sided rows ``a'z >= b``.

    The solver works with a single list of inequalities; keeping the public
    interface two-sided is what lets OSQP and qpsolvers take the same problem
    without a translation layer on the caller's side.
    """
    rows, rhs = [], []

    if A is not None and np.size(A):
        A = np.atleast_2d(np.asarray(A, dtype=float))
        if lb is not None:
            lb = np.atleast_1d(np.asarray(lb, dtype=float))
            for i in range(A.shape[0]):
                if np.isfinite(lb[i]):
                    rows.append(A[i])
                    rhs.append(lb[i])
        if ub is not None:
            ub = np.atleast_1d(np.asarray(ub, dtype=float))
            for i in range(A.shape[0]):
                if np.isfinite(ub[i]):
                    rows.append(-A[i])
                    rhs.append(-ub[i])

    eye = np.eye(n)
    if z_lb is not None:
        z_lb = np.broadcast_to(np.asarray(z_lb, dtype=float), (n,))
        for i in range(n):
            if np.isfinite(z_lb[i]):
                rows.append(eye[i])
                rhs.append(z_lb[i])
    if z_ub is not None:
        z_ub = np.broadcast_to(np.asarray(z_ub, dtype=float), (n,))
        for i in range(n):
            if np.isfinite(z_ub[i]):
                rows.append(-eye[i])
                rhs.append(-z_ub[i])

    if not rows:
        return np.zeros((0, n)), np.zeros(0)
    return np.asarray(rows, dtype=float), np.asarray(rhs, dtype=float)


def solve_dense_qp(
    H: np.ndarray,
    g: np.ndarray,
    A=None,
    lb=None,
    ub=None,
    z_lb=None,
    z_ub=None,
    *,
    tol: float = 1e-9,
    max_iter: int = 200,
    chol=None,
):
    """Solve the QP. Returns ``(z, status, iterations, objective)``.

    ``chol`` is a cached ``cho_factor(H)``; ``LinearMPC`` passes one because
    ``H`` is constant for an LTI model with fixed weights, and refactorising it
    every sample would show up in the reported solve time as work the algorithm
    does not actually have to do.
    """
    H = np.atleast_2d(np.asarray(H, dtype=float))
    g = np.atleast_1d(np.asarray(g, dtype=float))
    n = H.shape[0]

    if chol is None:
        chol = cho_factor(H)

    C, b = _as_rows(A, lb, ub, z_lb, z_ub, n)
    m = C.shape[0]

    def objective(z):
        return float(0.5 * z @ H @ z + g @ z)

    # Step 0: the unconstrained minimiser. If it is feasible we are done, and on
    # a quiet loop that is every sample -- which is why the iteration count is
    # readable as "how many limits the process forced on me".
    z = cho_solve(chol, -g)
    if m == 0:
        return z, SOLVED, 0, objective(z)

    active: list[int] = []
    lam = np.zeros(0)

    # H^-1 c_i for each constraint row, built lazily and reused: H is fixed.
    _hinv: dict[int, np.ndarray] = {}

    def hinv_row(i: int) -> np.ndarray:
        if i not in _hinv:
            _hinv[i] = cho_solve(chol, C[i])
        return _hinv[i]

    iterations = 0
    for iterations in range(1, max_iter + 1):
        violation = C @ z - b
        candidates = np.where(violation < -tol)[0]
        if candidates.size == 0:
            return z, SOLVED, iterations - 1, objective(z)
        p = int(candidates[np.argmin(violation[candidates])])

        # One constraint is added per outer iteration, but the inner loop may
        # have to drop several already-active ones on the way there -- that is
        # what keeps the method from falsely reporting infeasibility.
        for _ in range(m + 1):
            n_p = hinv_row(p)                        # H^-1 c_p
            if active:
                N = C[active]                        # (|A|, n)
                Minv = N @ np.column_stack([hinv_row(i) for i in active])
                try:
                    r = np.linalg.solve(Minv, N @ n_p)
                except np.linalg.LinAlgError:
                    r = np.linalg.lstsq(Minv, N @ n_p, rcond=None)[0]
                z_step = n_p - np.column_stack([hinv_row(i) for i in active]) @ r
            else:
                r = np.zeros(0)
                z_step = n_p

            moves = float(C[p] @ z_step)
            # Dual blocking: the active constraint whose multiplier hits zero
            # first as we push along this direction.
            t_dual, blocking = np.inf, None
            if active:
                pushing = np.where(r > tol)[0]
                if pushing.size:
                    ratios = lam[pushing] / r[pushing]
                    j = int(np.argmin(ratios))
                    t_dual, blocking = float(ratios[j]), int(pushing[j])

            if moves <= tol:
                # No primal progress available along this direction.
                if blocking is None:
                    return z, INFEASIBLE, iterations, float("inf")
                lam = lam - t_dual * r
                lam = np.delete(lam, blocking)
                active = active[:blocking] + active[blocking + 1 :]
                continue

            t_primal = float((b[p] - C[p] @ z) / moves)
            t = min(t_primal, t_dual)

            z = z + t * z_step
            if active:
                lam = lam - t * r
            if t_dual < t_primal:
                # Partial step: drop the blocking constraint and keep going
                # toward p rather than adding it prematurely.
                lam = np.delete(lam, blocking)
                active = active[:blocking] + active[blocking + 1 :]
                continue

            active = active + [p]
            lam = np.append(lam, t)
            break
        else:                                                   # pragma: no cover
            return z, INFEASIBLE, iterations, float("inf")

    return z, MAX_ITER, max_iter, objective(z)

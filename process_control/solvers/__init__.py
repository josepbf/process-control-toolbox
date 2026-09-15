"""The one place this toolbox calls out to somebody else's numerics.

Building an MPC's prediction matrices, disturbance model and constraint rows is
toolbox code, deliberately: that is what has to be read when a result looks too
good, and a controller you cannot open is a controller you cannot check. But
nobody learns anything from the iteration bookkeeping inside a QP solver, and a
good one is real numerical-analysis work with failure modes that have nothing to
do with process control. So the *numerical solve* is delegated, behind one
function.

The canonical form is::

    minimise   0.5 z' H z + g' z
    subject to lb <= A z <= ub,   z_lb <= z <= z_ub

which is OSQP's form, and maps to `qpsolvers` and cvxpy without loss. The
adaptation cost therefore lands on our own backend, which is where we want it.

Two rules about backends, both of which exist because this is a benchmarking
toolbox rather than a production controller:

**``"auto"`` resolves to the dependency-free solver, always. It never probes.**
A controller whose numerics change depending on what happens to be pip installed
is a controller whose results are not comparable across machines, and fairness
rule 2 is that every controller in a comparison runs under identical conditions.
An accelerator is opt-in, by name.

**A missing optional dependency is loud, never a silent fallback.** Somebody who
asked for OSQP and got the dense solver instead has a differently-solved run and
no way to know it. The backend that actually solved a run is recorded in
``df.attrs["controller_info"]["solver"]``.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from scipy.linalg import cho_factor

from .active_set import INFEASIBLE, MAX_ITER, SOLVED, solve_dense_qp

#: ``"auto"`` means this, and only this. See the module docstring.
DEFAULT_BACKEND = "dense"

#: Optional backends, and the extra that installs each one.
_OPTIONAL = {"osqp": ("osqp", "osqp"), "qpsolvers": ("qpsolvers", "qp"), "cvxpy": ("cvxpy", "cvxpy")}


@dataclass(frozen=True)
class QPResult:
    """One solve, and enough about it to put in a results table."""

    z: np.ndarray
    status: str            # "solved" | "max_iter" | "infeasible" | "failed"
    iterations: int
    objective: float
    backend: str

    @property
    def solved(self) -> bool:
        return self.status == SOLVED


def _require(module: str, extra: str):
    try:
        return importlib.import_module(module)
    except ImportError as exc:                                  # pragma: no cover
        raise ImportError(
            f"backend {extra!r} needs {module}, which is not installed. Install it "
            f"with:  pip install 'process-control-toolbox[{extra}]'  -- or leave "
            f"backend='auto', which uses the dependency-free solver and needs "
            f"nothing."
        ) from exc


class QPSolver(Protocol):
    """A solver configured once and held across samples.

    A bare ``solve_qp(H, g, ...)`` throws away the two things that make an MPC
    fast: the factorisation of a constant ``H``, and the warm start. Since
    ``solve_ms_mean`` is a *reported* metric, a boundary that makes MPC
    artificially slow distorts a comparison as surely as one that flatters it.
    So the stateful protocol is the one `LinearMPC` uses; :func:`solve_qp` is the
    one-shot entry point for everything else.
    """

    name: str

    def setup(self, H, A=None, z_lb=None, z_ub=None, *, tol: float = 1e-9,
              max_iter: int = 200) -> None: ...

    def solve(self, g, lb=None, ub=None, *, z0=None) -> QPResult: ...


class DenseActiveSetSolver:
    """The default: Goldfarb-Idnani, dense, numpy and scipy only."""

    name = "dense"

    def __init__(self):
        self._H = self._chol = self._A = None
        self._z_lb = self._z_ub = None
        self._tol, self._max_iter = 1e-9, 200

    def setup(self, H, A=None, z_lb=None, z_ub=None, *, tol=1e-9, max_iter=200) -> None:
        H = np.atleast_2d(np.asarray(H, dtype=float))
        try:
            self._chol = cho_factor(H)
        except np.linalg.LinAlgError as exc:
            raise ValueError(
                "the QP is not strictly convex: H must be positive definite, "
                "which for an MPC means a strictly positive move-suppression "
                "weight R. Regularising silently would change the problem "
                "being solved without saying so."
            ) from exc
        self._H = H
        self._A = None if A is None else np.atleast_2d(np.asarray(A, dtype=float))
        self._z_lb, self._z_ub = z_lb, z_ub
        self._tol, self._max_iter = tol, max_iter

    def solve(self, g, lb=None, ub=None, *, z0=None) -> QPResult:
        # z0 is accepted and ignored: a dual active-set method starts from the
        # unconstrained minimiser by construction, which is a better starting
        # point than a warm start and costs nothing to compute.
        z, status, iters, obj = solve_dense_qp(
            self._H, g, self._A, lb, ub, self._z_lb, self._z_ub,
            tol=self._tol, max_iter=self._max_iter, chol=self._chol,
        )
        return QPResult(z=z, status=status, iterations=iters, objective=obj, backend=self.name)


class OSQPSolver:
    """ADMM, warm-started, first order. Faster on big problems, approximate."""

    name = "osqp"

    def __init__(self, eps: float = 1e-8):
        self._osqp = _require(*_OPTIONAL["osqp"])
        self._sp = _require("scipy.sparse", "osqp")
        self._eps = eps
        self._prob = None
        self._n = 0

    def setup(self, H, A=None, z_lb=None, z_ub=None, *, tol=1e-9, max_iter=200) -> None:
        H = np.atleast_2d(np.asarray(H, dtype=float))
        self._n = H.shape[0]
        rows = [] if A is None else [np.atleast_2d(np.asarray(A, dtype=float))]
        rows.append(np.eye(self._n))                 # simple bounds as rows
        self._A_full = np.vstack(rows)
        self._n_rows = 0 if A is None else self._A_full.shape[0] - self._n
        self._z_lb = (
            np.full(self._n, -np.inf) if z_lb is None
            else np.broadcast_to(np.asarray(z_lb, float), (self._n,))
        )
        self._z_ub = (
            np.full(self._n, np.inf) if z_ub is None
            else np.broadcast_to(np.asarray(z_ub, float), (self._n,))
        )
        self._prob = self._osqp.OSQP()
        self._prob.setup(
            P=self._sp.csc_matrix(H), q=np.zeros(self._n),
            A=self._sp.csc_matrix(self._A_full),
            l=np.concatenate([np.full(self._n_rows, -np.inf), self._z_lb]),
            u=np.concatenate([np.full(self._n_rows, np.inf), self._z_ub]),
            eps_abs=max(tol, self._eps), eps_rel=max(tol, self._eps),
            max_iter=max(max_iter, 4000), polish=True, verbose=False,
        )

    def solve(self, g, lb=None, ub=None, *, z0=None) -> QPResult:
        low = np.concatenate([
            np.full(self._n_rows, -np.inf) if lb is None else np.asarray(lb, float),
            self._z_lb,
        ])
        high = np.concatenate([
            np.full(self._n_rows, np.inf) if ub is None else np.asarray(ub, float),
            self._z_ub,
        ])
        self._prob.update(q=np.asarray(g, dtype=float), l=low, u=high)
        if z0 is not None:
            self._prob.warm_start(x=np.asarray(z0, dtype=float))
        res = self._prob.solve()
        status = {
            "solved": SOLVED,
            "solved inaccurate": SOLVED,
            "maximum iterations reached": MAX_ITER,
        }.get(res.info.status, INFEASIBLE if "infeasible" in res.info.status else "failed")
        z = np.zeros(self._n) if res.x is None or np.any(np.isnan(res.x)) else res.x
        return QPResult(
            z=z, status=status, iterations=int(res.info.iter),
            objective=float(res.info.obj_val), backend=self.name,
        )


class QPSolversSolver:
    """Whatever `qpsolvers` is configured with -- a third opinion, mainly."""

    name = "qpsolvers"

    def __init__(self, solver: str = "quadprog"):
        self._qp = _require(*_OPTIONAL["qpsolvers"])
        self._solver = solver
        self._H = self._A = None
        self._z_lb = self._z_ub = None

    def setup(self, H, A=None, z_lb=None, z_ub=None, *, tol=1e-9, max_iter=200) -> None:
        self._H = np.atleast_2d(np.asarray(H, dtype=float))
        self._A = None if A is None else np.atleast_2d(np.asarray(A, dtype=float))
        n = self._H.shape[0]
        self._z_lb = None if z_lb is None else np.broadcast_to(np.asarray(z_lb, float), (n,)).copy()
        self._z_ub = None if z_ub is None else np.broadcast_to(np.asarray(z_ub, float), (n,)).copy()

    def solve(self, g, lb=None, ub=None, *, z0=None) -> QPResult:
        G_rows, h_rows = [], []
        if self._A is not None:
            if ub is not None:
                finite = np.isfinite(np.asarray(ub, float))
                G_rows.append(self._A[finite])
                h_rows.append(np.asarray(ub, float)[finite])
            if lb is not None:
                finite = np.isfinite(np.asarray(lb, float))
                G_rows.append(-self._A[finite])
                h_rows.append(-np.asarray(lb, float)[finite])
        G = np.vstack(G_rows) if G_rows else None
        h = np.concatenate(h_rows) if h_rows else None

        z = self._qp.solve_qp(
            P=self._H, q=np.asarray(g, dtype=float), G=G, h=h,
            lb=self._z_lb, ub=self._z_ub, solver=self._solver,
        )
        if z is None:
            n = self._H.shape[0]
            return QPResult(np.zeros(n), INFEASIBLE, 0, float("inf"), self.name)
        obj = float(0.5 * z @ self._H @ z + np.asarray(g, float) @ z)
        return QPResult(z=z, status=SOLVED, iterations=0, objective=obj, backend=self.name)


_BACKENDS = {
    "dense": DenseActiveSetSolver,
    "osqp": OSQPSolver,
    "qpsolvers": QPSolversSolver,
}


def get_solver(backend: str = "auto", **kwargs) -> QPSolver:
    """Build a configured solver. ``"auto"`` is always the dense one."""
    if backend == "auto":
        backend = DEFAULT_BACKEND
    if backend not in _BACKENDS:
        raise ValueError(
            f"unknown QP backend {backend!r}; available names are "
            f"{sorted(_BACKENDS)} (and 'auto', which is {DEFAULT_BACKEND!r})"
        )
    return _BACKENDS[backend](**kwargs)


def available_backends() -> dict[str, bool]:
    """Which backends this installation can actually use.

    Used to parametrise the agreement tests, and to answer "is OSQP worth
    installing here" without a traceback.
    """
    found = {"dense": True}
    for name, (module, _extra) in _OPTIONAL.items():
        if name not in _BACKENDS:
            continue
        try:
            importlib.import_module(module)
            found[name] = True
        except ImportError:
            found[name] = False
    return found


def solve_qp(
    H,
    g,
    A=None,
    lb=None,
    ub=None,
    z_lb=None,
    z_ub=None,
    *,
    z0=None,
    backend: str = "auto",
    tol: float = 1e-9,
    max_iter: int = 200,
) -> QPResult:
    """Solve one QP.

        minimise   0.5 z' H z + g' z
        subject to lb <= A z <= ub,   z_lb <= z <= z_ub

    ``H`` must be symmetric positive definite -- for an MPC that means a
    strictly positive move-suppression weight. A singular ``H`` is refused
    rather than regularised, because quietly adding ``eps * I`` changes the
    problem being solved without saying so.

    This is the one-shot entry point. A controller solving the same-shaped
    problem every sample should hold a :class:`QPSolver` from :func:`get_solver`
    instead, so the factorisation of a constant ``H`` is not repeated.
    """
    solver = get_solver(backend)
    solver.setup(H, A, z_lb, z_ub, tol=tol, max_iter=max_iter)
    return solver.solve(g, lb, ub, z0=z0)


__all__ = [
    "QPResult",
    "QPSolver",
    "solve_qp",
    "get_solver",
    "available_backends",
    "DEFAULT_BACKEND",
    "SOLVED",
    "MAX_ITER",
    "INFEASIBLE",
]

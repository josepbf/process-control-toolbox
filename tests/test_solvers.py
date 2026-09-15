"""The one place the toolbox delegates numerics, and the rules that keep it honest."""

import numpy as np
import pytest

from process_control.solvers import (
    DEFAULT_BACKEND,
    available_backends,
    get_solver,
    solve_qp,
)

INSTALLED = [name for name, ok in available_backends().items() if ok]


def _random_qp(rng, n=6, m=4):
    M = rng.normal(size=(n, n))
    H = M @ M.T + n * np.eye(n)                 # symmetric positive definite
    g = rng.normal(size=n)
    A = rng.normal(size=(m, n))
    lb = -np.abs(rng.normal(size=m)) - 0.5
    ub = np.abs(rng.normal(size=m)) + 0.5
    z_lb = -np.abs(rng.normal(size=n)) - 1.0
    z_ub = np.abs(rng.normal(size=n)) + 1.0
    return H, g, A, lb, ub, z_lb, z_ub


# ----------------------------------------------------------------------
# correctness
# ----------------------------------------------------------------------
def test_an_unconstrained_qp_returns_the_analytic_minimiser():
    """And in zero iterations. A dual method starts from this point, which is
    what makes 'unconstrained MPC is a linear controller' exactly testable
    rather than testable to a solver tolerance."""
    H = np.array([[4.0, 1.0], [1.0, 3.0]])
    g = np.array([-1.0, -2.0])

    result = solve_qp(H, g)
    assert np.allclose(result.z, np.linalg.solve(H, -g), atol=1e-14)
    assert result.iterations == 0
    assert result.solved


def test_a_bound_that_binds_is_honoured_exactly_and_the_kkt_residual_is_zero():
    H = np.array([[2.0, 0.0], [0.0, 2.0]])
    g = np.array([2.0, 2.0])                    # unconstrained minimiser is (-1, -1)
    z_lb = np.array([0.0, 0.0])

    result = solve_qp(H, g, z_lb=z_lb)
    assert np.allclose(result.z, [0.0, 0.0], atol=1e-12)

    # Stationarity: H z + g must be a non-negative combination of the active
    # lower-bound normals, i.e. non-negative here.
    assert np.all(H @ result.z + g >= -1e-12)


def test_the_solver_agrees_with_scipy_on_a_hundred_random_problems():
    """An independent method (SLSQP, a sequential-QP method) on the same
    problems. The point is not that scipy is authoritative -- it is that two
    unrelated algorithms agreeing is evidence and one algorithm agreeing with
    itself is not."""
    minimize = pytest.importorskip("scipy.optimize").minimize
    rng = np.random.default_rng(0)

    for _ in range(100):
        H, g, A, lb, ub, z_lb, z_ub = _random_qp(rng)
        ours = solve_qp(H, g, A, lb, ub, z_lb, z_ub)
        reference = minimize(
            lambda z: 0.5 * z @ H @ z + g @ z, np.zeros(len(g)),
            jac=lambda z: H @ z + g,
            bounds=list(zip(z_lb, z_ub)),
            constraints=[
                {"type": "ineq", "fun": lambda z: A @ z - lb},
                {"type": "ineq", "fun": lambda z: ub - A @ z},
            ],
            method="SLSQP", options={"maxiter": 500, "ftol": 1e-12},
        )
        reference_objective = 0.5 * reference.x @ H @ reference.x + g @ reference.x
        assert ours.objective <= reference_objective + 1e-6
        assert np.all(A @ ours.z >= lb - 1e-8)
        assert np.all(A @ ours.z <= ub + 1e-8)


def test_the_returned_point_always_satisfies_the_constraints_it_reports_solving():
    rng = np.random.default_rng(7)
    for _ in range(50):
        H, g, A, lb, ub, z_lb, z_ub = _random_qp(rng, n=8, m=6)
        result = solve_qp(H, g, A, lb, ub, z_lb, z_ub)
        if not result.solved:
            continue
        assert np.all(result.z >= z_lb - 1e-8)
        assert np.all(result.z <= z_ub + 1e-8)


# ----------------------------------------------------------------------
# the awkward cases
# ----------------------------------------------------------------------
def test_the_solver_terminates_on_redundant_active_constraints():
    """Six copies of the same binding constraint. A naive active-set loop either
    cycles or declares the problem infeasible; neither is acceptable inside a
    controller running fourteen hundred times a run."""
    H = np.array([[4.0, 1.0], [1.0, 3.0]])
    g = np.array([-1.0, -2.0])
    A = np.tile(np.array([[1.0, 1.0]]), (6, 1))

    result = solve_qp(H, g, A=A, lb=np.full(6, 2.0), ub=np.full(6, np.inf), max_iter=50)
    assert result.solved
    assert float(result.z.sum()) == pytest.approx(2.0, abs=1e-9)


def test_a_genuinely_infeasible_problem_is_reported_not_guessed_at():
    result = solve_qp(np.eye(1), np.zeros(1), A=np.eye(1), lb=[1.0], ub=[-1.0])
    assert result.status == "infeasible"
    assert not result.solved


def test_a_singular_hessian_is_refused_rather_than_regularised():
    """Silently adding eps * I would change the problem being solved without
    saying so. For an MPC a singular H means a zero move-suppression weight,
    which is a design error worth hearing about."""
    solver = get_solver("auto")
    with pytest.raises(ValueError, match="positive definite"):
        solver.setup(np.zeros((2, 2)))


def test_infinite_bounds_are_simply_absent():
    H = np.array([[2.0]])
    g = np.array([2.0])
    result = solve_qp(H, g, z_lb=[-np.inf], z_ub=[np.inf])
    assert result.z[0] == pytest.approx(-1.0)
    assert result.iterations == 0


# ----------------------------------------------------------------------
# the backend rules
# ----------------------------------------------------------------------
def test_auto_never_probes_for_whatever_happens_to_be_installed():
    """Fairness rule 2: every controller in a comparison runs under identical
    conditions. A controller whose numerics depend on the machine's pip state is
    not comparable across machines, so 'auto' is a fixed choice, not a search."""
    assert DEFAULT_BACKEND == "dense"
    assert get_solver("auto").name == "dense"


def test_the_dependency_free_backend_is_always_available():
    assert available_backends()["dense"] is True


def test_an_unknown_backend_name_is_refused_with_the_alternatives():
    with pytest.raises(ValueError, match="unknown QP backend"):
        get_solver("gurobi")


@pytest.mark.parametrize("backend", [b for b in INSTALLED if b != "dense"])
def test_backends_agree_with_the_dense_reference(backend):
    """The dense solver terminates finitely at the exact answer, so it is the
    reference and the accelerator is checked against it -- not the other way
    round. OSQP is first-order, so it is compared at its own tolerance."""
    rng = np.random.default_rng(3)
    for _ in range(20):
        H, g, A, lb, ub, z_lb, z_ub = _random_qp(rng)
        exact = solve_qp(H, g, A, lb, ub, z_lb, z_ub, backend="dense")
        other = solve_qp(H, g, A, lb, ub, z_lb, z_ub, backend=backend)
        assert other.backend == backend
        assert other.objective == pytest.approx(exact.objective, abs=1e-5)


def test_a_missing_backend_says_how_to_install_it():
    missing = [name for name, ok in available_backends().items() if not ok]
    if not missing:
        pytest.skip("every optional backend is installed here")
    with pytest.raises(ImportError, match=r"pip install 'process-control-toolbox\["):
        get_solver(missing[0])


def test_the_toolbox_imports_with_no_optional_solver_installed():
    """The core install is numpy, scipy, matplotlib, pandas -- and must stay
    that way. Every optional import is guarded at call time, never at import
    time, so this is the test that the guards are where they claim to be."""
    import importlib
    import sys

    blocked = ("osqp", "cvxpy", "do_mpc", "casadi", "qpsolvers")
    saved = {name: sys.modules.pop(name, None) for name in blocked}

    class Blocker:
        def find_module(self, name, path=None):
            return None

    try:
        for name in blocked:
            sys.modules[name] = None                # imports of these now fail
        for module in ("process_control", "process_control.solvers"):
            sys.modules.pop(module, None)
        pc = importlib.import_module("process_control")
        for name in pc.__all__:
            assert hasattr(pc, name), name
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module
        importlib.reload(importlib.import_module("process_control"))


# ----------------------------------------------------------------------
# the stateful protocol
# ----------------------------------------------------------------------
def test_a_held_solver_gives_the_same_answer_as_the_one_shot_call():
    """LinearMPC holds a solver so the factorisation of a constant H is not
    repeated every sample. That must be an optimisation, not a different
    answer."""
    rng = np.random.default_rng(11)
    H, g, A, lb, ub, z_lb, z_ub = _random_qp(rng)

    solver = get_solver("auto")
    solver.setup(H, A, z_lb, z_ub)
    held = solver.solve(g, lb, ub)
    one_shot = solve_qp(H, g, A, lb, ub, z_lb, z_ub)
    assert np.allclose(held.z, one_shot.z, atol=1e-12)


def test_a_held_solver_can_be_re_solved_with_a_new_gradient():
    """Which is the whole point: across samples only g, lb and ub change."""
    H = np.array([[2.0, 0.0], [0.0, 2.0]])
    solver = get_solver("auto")
    solver.setup(H, z_lb=np.array([-1.0, -1.0]), z_ub=np.array([1.0, 1.0]))

    assert np.allclose(solver.solve(np.array([-4.0, -4.0])).z, [1.0, 1.0], atol=1e-12)
    assert np.allclose(solver.solve(np.array([4.0, 4.0])).z, [-1.0, -1.0], atol=1e-12)
    assert np.allclose(solver.solve(np.array([0.0, 0.0])).z, [0.0, 0.0], atol=1e-12)

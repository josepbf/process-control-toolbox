"""The MPC's matrices, checked against the equations they were derived from.

The realistic failure in a condensed MPC is not a wrong idea, it is an
off-by-one: a `Theta` column shifted by a sample, an `Upsilon` that forgot the
move is *held* rather than applied once, a weight transposed. None of those stop
the controller running, and several of them still produce a plausible-looking
closed loop. What catches them is a second, independent expression of the same
problem -- written from the equations, sample by sample, in a plain loop -- and
the requirement that the two agree.

That is what this file is. It is the cheapest high-value test in the MPC work,
and it deliberately needs no optional dependency, so it runs every time rather
than being skipped on most machines.
"""

import numpy as np
import pytest

from process_control.controllers.mpc import LinearMPC
from process_control.models.discrete import fopdt_model

TRUE = dict(K=1.5, tau=60.0, theta=15.0)


def _mpc(**kw):
    options = dict(N=25, M=6, Q=1.0, R=0.7, u_min=-np.inf, u_max=np.inf, u0=20.0)
    options.update(kw)
    return LinearMPC(fopdt_model(**TRUE, dt=1.0), **options)


def _roll(model, x0, u_prev, increments, n_steps):
    x = np.asarray(x0, dtype=float).copy()
    u = float(u_prev)
    out = []
    for i in range(n_steps):
        if i < len(increments):
            u += float(increments[i])       # applied, then held from here on
        x = model.step(x, np.array([u]))
        out.append(float((model.C @ x)[0]))
    return np.array(out)


def test_the_prediction_matrices_agree_with_a_plain_roll_forward():
    """Psi, Upsilon and Theta against a sample-by-sample loop, on random states
    and random move sequences. This is the test that catches a shifted column."""
    mpc = _mpc()
    model = mpc.model
    rng = np.random.default_rng(0)

    for _ in range(25):
        x0 = rng.normal(scale=10.0, size=model.n_states)
        u_prev = float(rng.normal(scale=20.0))
        du = rng.normal(scale=2.0, size=mpc.M)

        from_matrices = (mpc._Psi @ x0 + mpc._Upsilon @ np.array([u_prev]) + mpc._Theta @ du)
        by_hand = _roll(model, x0, u_prev, du, mpc.N)

        assert np.allclose(from_matrices, by_hand, atol=1e-9)


def test_the_free_response_is_what_happens_if_nothing_is_done():
    """Upsilon must encode the last move being *held* across the horizon, not
    applied once. Getting that wrong is invisible at steady state and wrong
    everywhere else."""
    mpc = _mpc()
    model = mpc.model
    rng = np.random.default_rng(1)
    x0 = rng.normal(scale=10.0, size=model.n_states)
    u_prev = 30.0

    free = mpc._Psi @ x0 + mpc._Upsilon @ np.array([u_prev])
    held = _roll(model, x0, u_prev, np.zeros(0), mpc.N)
    assert np.allclose(free, held, atol=1e-9)


def test_increments_past_the_control_horizon_are_held_not_dropped():
    """Beyond M the increments are zero, so the last move stands to the end of
    the prediction horizon. If Theta's final column did not accumulate the tail,
    the controller would believe the valve springs back."""
    mpc = _mpc(N=30, M=4)
    model = mpc.model
    x0 = np.zeros(model.n_states)
    du = np.array([2.0, -1.0, 0.5, 1.0])

    from_matrices = mpc._Theta @ du
    by_hand = _roll(model, x0, 0.0, du, mpc.N)      # held after the 4th increment
    assert np.allclose(from_matrices, by_hand, atol=1e-9)


def test_the_solved_move_minimises_the_cost_written_out_term_by_term():
    """The QP's answer against `scipy.optimize.minimize` on the objective
    transcribed directly from the definition -- a different algorithm, a
    different expression of the same problem, and no shared code beyond the
    model itself."""
    minimize = pytest.importorskip("scipy.optimize").minimize
    mpc = _mpc()
    model = mpc.model
    rng = np.random.default_rng(2)

    x0 = rng.normal(scale=5.0, size=model.n_states)
    u_prev, setpoint = 25.0, 50.0

    def cost(du):
        y = _roll(model, x0, u_prev, du, mpc.N)
        return float(mpc.Q * np.sum((y - setpoint) ** 2) + mpc.R * np.sum(du**2))

    # Powell: derivative-free, so it shares no algebra with our matrices, and
    # unlike Nelder-Mead it actually converges on a six-dimensional quadratic
    # this ill-conditioned. A looser optimiser would be testing itself.
    reference = minimize(
        cost, np.zeros(mpc.M), method="Powell",
        options={"xtol": 1e-12, "ftol": 1e-14, "maxiter": 100000},
    )

    # Drive the controller's own machinery to the same problem.
    f = mpc._Psi @ x0 + mpc._Upsilon @ np.array([u_prev])
    T = np.full(mpc.N, setpoint)
    g = np.zeros(mpc._n_z)
    g[: mpc.M] = 2.0 * (mpc._Theta.T @ mpc._Qbar @ (f - T))
    ours = mpc._solver.solve(g, *mpc._row_limits(f)).z[: mpc.M]

    assert cost(ours) == pytest.approx(cost(reference.x), rel=1e-9)
    assert np.allclose(ours, reference.x, atol=1e-4)


def test_the_cost_h_and_g_reproduce_the_objective_up_to_a_constant():
    """0.5 z'Hz + g'z must equal the written-out cost, offset by a term that does
    not depend on the decision. A transposed weight survives every closed-loop
    test and dies here."""
    mpc = _mpc()
    model = mpc.model
    rng = np.random.default_rng(3)

    x0 = rng.normal(scale=5.0, size=model.n_states)
    u_prev, setpoint = 25.0, 50.0
    f = mpc._Psi @ x0 + mpc._Upsilon @ np.array([u_prev])
    T = np.full(mpc.N, setpoint)

    H = mpc._H[: mpc.M, : mpc.M]
    g = 2.0 * (mpc._Theta.T @ mpc._Qbar @ (f - T))

    def written_out(du):
        y = _roll(model, x0, u_prev, du, mpc.N)
        return float(mpc.Q * np.sum((y - setpoint) ** 2) + mpc.R * np.sum(du**2))

    def condensed(du):
        return float(0.5 * du @ H @ du + g @ du)

    offsets = []
    for _ in range(20):
        du = rng.normal(scale=3.0, size=mpc.M)
        offsets.append(written_out(du) - condensed(du))

    # The gap must be the same constant every time.
    assert np.std(offsets) < 1e-6 * max(1.0, abs(np.mean(offsets)))


def test_the_condensed_qp_agrees_with_a_declarative_cvxpy_transcription():
    """The same check again through cvxpy, when it is installed: the problem
    written as variables and inequalities rather than as matrices, and solved by
    an interior-point method rather than an active set."""
    cp = pytest.importorskip("cvxpy")
    mpc = _mpc(u_min=0.0, u_max=100.0, du_max=5.0)
    model = mpc.model
    rng = np.random.default_rng(4)

    x0 = rng.normal(scale=5.0, size=model.n_states)
    u_prev, setpoint = 25.0, 50.0
    f = mpc._Psi @ x0 + mpc._Upsilon @ np.array([u_prev])
    T = np.full(mpc.N, setpoint)

    du = cp.Variable(mpc.M)
    y = f + mpc._Theta @ du
    u = u_prev + cp.cumsum(du)
    problem = cp.Problem(
        cp.Minimize(mpc.Q * cp.sum_squares(y - T) + mpc.R * cp.sum_squares(du)),
        [u >= mpc.u_min, u <= mpc.u_max, cp.abs(du) <= mpc.du_max],
    )
    problem.solve()

    g = np.zeros(mpc._n_z)
    g[: mpc.M] = 2.0 * (mpc._Theta.T @ mpc._Qbar @ (f - T))
    ours = mpc._solver.solve(g, *mpc._row_limits(f)).z[: mpc.M]

    assert np.allclose(ours, du.value, atol=1e-5)

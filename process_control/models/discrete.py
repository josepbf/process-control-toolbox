"""Discrete-time linear internal models, and the builders that make them.

The controller's belief about the process, in the one form every model-based law
in this project needs::

    x[k+1] = A x[k] + B u[k] (+ Bd d[k]),      y[k] = C x[k]

Two decisions are worth stating up front, because both are the kind of thing
that silently biases a comparison if left implicit.

**Discretisation is exact (zero-order hold), not Euler.** On these plants the
sample time is a substantial fraction of the time constant -- dt = 1 s against
tau = 60 s is the standard case, but the dead-time sweeps push much further --
and a first-order discretisation would introduce a model error nobody declared.
Exact ZOH costs one matrix exponential, once, at construction.

**Dead time becomes states, and the rounding is reported.** A delay of theta
seconds at sample time dt is a shift register of ``round(theta/dt)`` samples
appended to the state, matching what `plants/base.py` does to the true plant. If
theta is not an integer multiple of dt, both round -- and this module records
what was asked for alongside what was realised, so the residual shows up in
``describe()`` rather than hiding inside the model. Undeclared mismatch is the
bad kind.

The cost to watch is ``n_delay``: theta = 15 s at dt = 1 s means 15 extra states
on top of the process, and an MPC's prediction matrices are O(N * n^2).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
from scipy.linalg import expm


def zoh(Ac: np.ndarray, Bc: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact zero-order-hold discretisation of ``xdot = Ac x + Bc u``.

    One matrix exponential does both matrices at once (Van Loan 1978)::

        expm([[Ac, Bc], [0, 0]] * dt)  ->  [[Ad, Bd], [0, I]]

    which is exact for an input held constant over the sample, and so matches
    what the harness actually does to the plant: `simulate()` holds ``u`` over
    ``[t, t+dt)`` and the plant integrates under it.

    Parameters
    ----------
    Ac, Bc : ndarray
        Continuous-time state and input matrices, shapes ``(n, n)``, ``(n, m)``.
    dt : float
        Sample time, seconds. Must be positive.

    Returns
    -------
    (Ad, Bd) : tuple of ndarray
        The discrete-time pair.

    References
    ----------
    Van Loan, C.F. (1978). "Computing integrals involving the matrix
        exponential." IEEE Trans. Automatic Control 23(3), 395-404.
    """
    Ac = np.atleast_2d(np.asarray(Ac, dtype=float))
    Bc = np.atleast_2d(np.asarray(Bc, dtype=float))
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")
    n, m = Ac.shape[0], Bc.shape[1]
    if Ac.shape[0] != Ac.shape[1]:
        raise ValueError(f"Ac must be square, got shape {Ac.shape}")
    if Bc.shape[0] != n:
        raise ValueError(f"Bc has {Bc.shape[0]} rows but Ac is {n}x{n}")

    block = np.zeros((n + m, n + m))
    block[:n, :n] = Ac
    block[:n, n:] = Bc
    out = expm(block * float(dt))
    return out[:n, :n].copy(), out[:n, n:].copy()


@dataclass(frozen=True)
class DiscreteModel:
    """A linear discrete-time internal model.

    Attributes
    ----------
    A, B, C : ndarray
        ``x[k+1] = A x[k] + B u[k]``, ``y[k] = C x[k]``. Shapes ``(n, n)``,
        ``(n, m)``, ``(p, n)``.
    dt : float
        Sample time the model was discretised at. It must match the scenario's
        ``dt``; nothing enforces that for you, and a mismatch is silent.
    Bd : ndarray or None
        Input matrix for a *measured* disturbance, if the model carries one.
    n_delay : tuple of int
        Delay samples folded into ``A``/``B``, one entry per input. The delay
        states always sit **after** the process states -- observers, prediction
        matrices and tests all index on that convention.
    theta_requested : tuple of float
        The dead time that was asked for, before rounding to whole samples.
    note : str
        Provenance, and any rounding residual. Ends up in the run record.
    """

    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    dt: float
    Bd: np.ndarray | None = None
    n_delay: tuple[int, ...] = ()
    theta_requested: tuple[float, ...] = ()
    note: str = ""
    #: Names of the states, for readable diagnostics. Empty means unnamed.
    state_names: tuple[str, ...] = field(default=())
    #: Integrating disturbance states appended by ``augment_disturbance``, which
    #: always sit last. Nonzero is what makes a model-based law offset-free.
    n_disturbance_states: int = 0
    #: ``"output"`` or ``"input"`` -- where the estimated upset is assumed to
    #: enter. The choice is physical, not numerical; see ``augment_disturbance``.
    disturbance_kind: str = ""

    def __post_init__(self):
        A = np.atleast_2d(np.asarray(self.A, dtype=float))
        B = np.atleast_2d(np.asarray(self.B, dtype=float))
        C = np.atleast_2d(np.asarray(self.C, dtype=float))
        if A.shape[0] != A.shape[1]:
            raise ValueError(f"A must be square, got shape {A.shape}")
        if B.shape[0] != A.shape[0]:
            raise ValueError(f"B has {B.shape[0]} rows but A is {A.shape[0]}x{A.shape[0]}")
        if C.shape[1] != A.shape[0]:
            raise ValueError(f"C has {C.shape[1]} columns but A is {A.shape[0]}x{A.shape[0]}")
        if self.dt <= 0:
            raise ValueError(f"dt must be positive, got {self.dt}")
        # frozen dataclass: normalise through object.__setattr__
        object.__setattr__(self, "A", A)
        object.__setattr__(self, "B", B)
        object.__setattr__(self, "C", C)
        if self.Bd is not None:
            Bd = np.atleast_2d(np.asarray(self.Bd, dtype=float))
            if Bd.shape[0] != A.shape[0]:
                raise ValueError(
                    f"Bd has {Bd.shape[0]} rows but the state is {A.shape[0]}-dimensional"
                )
            object.__setattr__(self, "Bd", Bd)

    # -- shape ---------------------------------------------------------------
    @property
    def n_states(self) -> int:
        return self.A.shape[0]

    @property
    def n_inputs(self) -> int:
        return self.B.shape[1]

    @property
    def n_outputs(self) -> int:
        return self.C.shape[0]

    @property
    def n_disturbances(self) -> int:
        return 0 if self.Bd is None else self.Bd.shape[1]

    @property
    def theta_realised(self) -> tuple[float, ...]:
        """The dead time the model actually implements, after rounding."""
        return tuple(n * self.dt for n in self.n_delay)

    # -- simulation ----------------------------------------------------------
    def step(self, x: np.ndarray, u: np.ndarray, d: np.ndarray | None = None) -> np.ndarray:
        """One sample of the model: ``x[k+1]`` from ``x[k]``, ``u[k]``."""
        x = np.atleast_1d(np.asarray(x, dtype=float))
        u = np.atleast_1d(np.asarray(u, dtype=float))
        x_next = self.A @ x + self.B @ u
        if d is not None:
            if self.Bd is None:
                raise ValueError(
                    "this model carries no disturbance path: build one with "
                    "append_disturbance_path() before passing d"
                )
            x_next = x_next + self.Bd @ np.atleast_1d(np.asarray(d, dtype=float))
        return x_next

    def output(self, x: np.ndarray) -> np.ndarray:
        return self.C @ np.atleast_1d(np.asarray(x, dtype=float))

    def rollout(
        self,
        x0: np.ndarray,
        u_seq: np.ndarray,
        d_seq: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Open-loop simulate the model over a sequence of moves.

        Returns ``(X, Y)`` of shapes ``(n_steps + 1, n_states)`` and
        ``(n_steps + 1, n_outputs)``, both including the initial sample, so
        ``Y[0] == C x0`` and ``Y[k]`` is the output after ``k`` moves.
        """
        u_seq = np.atleast_2d(np.asarray(u_seq, dtype=float))
        if u_seq.shape[1] != self.n_inputs and u_seq.shape[0] == self.n_inputs:
            u_seq = u_seq.T
        if u_seq.shape[1] != self.n_inputs:
            raise ValueError(
                f"u_seq has {u_seq.shape[1]} columns but the model has "
                f"{self.n_inputs} input(s)"
            )
        n_steps = u_seq.shape[0]

        if d_seq is not None:
            d_seq = np.atleast_2d(np.asarray(d_seq, dtype=float))
            if d_seq.shape[0] != n_steps:
                raise ValueError("d_seq must have as many rows as u_seq")

        X = np.empty((n_steps + 1, self.n_states))
        Y = np.empty((n_steps + 1, self.n_outputs))
        X[0] = np.atleast_1d(np.asarray(x0, dtype=float))
        Y[0] = self.C @ X[0]
        for k in range(n_steps):
            X[k + 1] = self.step(X[k], u_seq[k], None if d_seq is None else d_seq[k])
            Y[k + 1] = self.C @ X[k + 1]
        return X, Y

    # -- analysis ------------------------------------------------------------
    def dc_gain(self) -> np.ndarray:
        """Steady-state gain ``C (I - A)^-1 B``.

        For a self-regulating process this is the ``K`` a tuning rule uses, and
        checking it against the plant is the cheapest test that a model was
        built correctly. An integrating model has ``A`` with an eigenvalue at 1,
        so ``I - A`` is singular and the gain is infinite -- that is reported as
        ``inf`` rather than raised, because it is a property of the process, not
        an error.
        """
        eye = np.eye(self.n_states)
        try:
            return self.C @ np.linalg.solve(eye - self.A, self.B)
        except np.linalg.LinAlgError:
            return np.full((self.n_outputs, self.n_inputs), np.inf)

    def describe(self) -> dict:
        """The model's record, written into the run metadata beside the tuning."""
        record = {
            "n_states": self.n_states,
            "n_inputs": self.n_inputs,
            "n_outputs": self.n_outputs,
            "dt": self.dt,
            "n_delay": list(self.n_delay),
            "theta_requested": list(self.theta_requested),
            "theta_realised": list(self.theta_realised),
            "note": self.note,
        }
        if self.n_disturbance_states:
            record["n_disturbance_states"] = self.n_disturbance_states
            record["disturbance_kind"] = self.disturbance_kind
        if self.Bd is not None:
            record["n_disturbances"] = self.n_disturbances
        return record


# ---------------------------------------------------------------------------
# builders
# ---------------------------------------------------------------------------
def _delay_samples(theta: float, dt: float) -> tuple[int, str]:
    """Whole samples of delay, rounded the way the plant rounds, plus a note."""
    n = int(np.round(float(theta) / float(dt)))
    if n < 0:
        raise ValueError(f"dead time must not be negative, got theta = {theta}")
    residual = float(theta) - n * float(dt)
    note = ""
    if abs(residual) > 1e-12:
        note = (
            f"dead time rounded to whole samples: theta = {theta:g} s at "
            f"dt = {dt:g} s realised as {n * dt:g} s "
            f"(residual {residual:+.3g} s, undeclared mismatch if ignored)"
        )
    return n, note


def augment_dead_time(
    model: DiscreteModel,
    theta: float,
    *,
    input_index: int = 0,
) -> DiscreteModel:
    """Append a shift register so the input reaches the process ``theta`` late.

    With ``n = round(theta/dt)`` delay samples the state becomes
    ``x_aug = [x_process ; z]`` where ``z = [u[k-n], ..., u[k-1]]``::

        A_aug = [[A, B e_1^T],      B_aug = [[0  ],
                 [0, S       ]]              [e_n]]

    ``S`` is the shift matrix and ``e_1`` picks the oldest queued move as the one
    reaching the process now. Delay states go **after** the process states and
    stay there: the observer's disturbance augmentation, an MPC's prediction
    matrices and every test index on that.

    This is the expensive part of a model-based controller on a dead-time
    dominant loop -- theta = 15 s at dt = 1 s is 15 extra states -- and it is
    also the whole reason the loop is hard, so there is nothing to be saved
    here.
    """
    n, note = _delay_samples(theta, model.dt)
    if n == 0:
        return replace(
            model,
            n_delay=model.n_delay or (0,) * model.n_inputs,
            theta_requested=model.theta_requested or (float(theta),),
            note=" ".join(s for s in (model.note, note) if s),
        )
    if model.n_inputs != 1:
        raise ValueError(
            "dead-time augmentation is written for a single input; a MIMO model "
            "needs one shift register per input and a decision about which "
            "input each delay belongs to"
        )
    if input_index != 0:
        raise ValueError(f"input_index must be 0 for a single-input model, got {input_index}")

    nx = model.n_states
    A = np.zeros((nx + n, nx + n))
    B = np.zeros((nx + n, 1))
    C = np.zeros((model.n_outputs, nx + n))

    A[:nx, :nx] = model.A
    A[:nx, nx] = model.B[:, 0]            # oldest queued move drives the process
    if n > 1:
        A[nx : nx + n - 1, nx + 1 : nx + n] = np.eye(n - 1)   # shift the queue along
    B[nx + n - 1, 0] = 1.0                # the new move enters at the back
    C[:, :nx] = model.C

    Bd = None
    if model.Bd is not None:
        Bd = np.zeros((nx + n, model.n_disturbances))
        Bd[:nx, :] = model.Bd

    names = tuple(model.state_names) + tuple(f"u_delay{i}" for i in range(n))
    return DiscreteModel(
        A=A, B=B, C=C, dt=model.dt, Bd=Bd,
        n_delay=(n,),
        theta_requested=(float(theta),),
        note=" ".join(s for s in (model.note, note) if s),
        state_names=names if model.state_names else (),
    )


def fopdt_model(
    K: float,
    tau: float,
    theta: float = 0.0,
    dt: float = 1.0,
    *,
    delay_states: bool = True,
) -> DiscreteModel:
    """The three-parameter model every classical tuning rule is stated in.

        tau * dy/dt = -y + K * u(t - theta)

    Written to be called the way a tuning rule is::

        model = fopdt_model(**plant.fopdt, dt=scenario.dt)
        tuning = simc_pi(**plant.fopdt)

    and carrying the same caveat: ``plant.fopdt`` is the *true* model, and a
    controller is only entitled to it when a no-mismatch upper bound is being
    deliberately measured (fairness rule 3).

    ``delay_states=False`` returns the dead-time-free part on its own, which is
    exactly what a Smith predictor's primary model is.
    """
    tau = float(tau)
    if tau <= 0:
        raise ValueError(f"tau must be positive for a first-order lag, got {tau}")
    Ad, Bd_ = zoh([[-1.0 / tau]], [[float(K) / tau]], dt)
    model = DiscreteModel(
        A=Ad, B=Bd_, C=[[1.0]], dt=float(dt),
        note=f"FOPDT K={K:g}, tau={tau:g}, theta={theta:g}, exact ZOH at dt={dt:g}",
        state_names=("y_process",),
    )
    if not delay_states:
        return model
    return augment_dead_time(model, theta)


def integrating_model(
    k_prime: float,
    theta: float = 0.0,
    dt: float = 1.0,
    *,
    delay_states: bool = True,
) -> DiscreteModel:
    """A non-self-regulating process: ``dy/dt = k' * u(t - theta)``.

    Surge drums, level in a tank with a pump on the outlet, pressure in a closed
    vessel. There is no steady-state gain to speak of -- :meth:`dc_gain` reports
    ``inf`` -- which is exactly why these loops need their own tuning rules
    (`tuning.rules.simc_integrating`, `averaging_level_pi`).
    """
    model = DiscreteModel(
        A=[[1.0]], B=[[float(k_prime) * float(dt)]], C=[[1.0]], dt=float(dt),
        note=f"integrating k'={k_prime:g}, theta={theta:g}, exact ZOH at dt={dt:g}",
        state_names=("y_process",),
    )
    if not delay_states:
        return model
    return augment_dead_time(model, theta)


def series_lags_model(
    K: float,
    taus,
    theta: float = 0.0,
    dt: float = 1.0,
    *,
    delay_states: bool = True,
) -> DiscreteModel:
    """N first-order lags in series, the honest high-order process.

        tau_i * dx_i/dt = -x_i + x_{i-1},     x_0 = K * u(t - theta)

    Useful as the *true* model when the controller is given only the half-rule
    FOPDT reduction of it (`tuning.rules.half_rule`) -- the mismatch between the
    two is the point, not an accident.
    """
    taus = np.atleast_1d(np.asarray(taus, dtype=float))
    if taus.size == 0 or np.any(taus <= 0):
        raise ValueError(f"every time constant must be positive, got {taus}")
    n = taus.size

    Ac = np.zeros((n, n))
    Bc = np.zeros((n, 1))
    Ac[0, 0] = -1.0 / taus[0]
    Bc[0, 0] = float(K) / taus[0]
    for i in range(1, n):
        Ac[i, i] = -1.0 / taus[i]
        Ac[i, i - 1] = 1.0 / taus[i]
    C = np.zeros((1, n))
    C[0, -1] = 1.0

    Ad, Bd_ = zoh(Ac, Bc, dt)
    model = DiscreteModel(
        A=Ad, B=Bd_, C=C, dt=float(dt),
        note=(
            f"series lags K={K:g}, taus={tuple(float(t) for t in taus)}, "
            f"theta={theta:g}, exact ZOH at dt={dt:g}"
        ),
        state_names=tuple(f"lag{i}" for i in range(n)),
    )
    if not delay_states:
        return model
    return augment_dead_time(model, theta)


def append_disturbance_path(
    model: DiscreteModel,
    Kd: float,
    tau_d: float | None = None,
    theta_d: float = 0.0,
) -> DiscreteModel:
    """Give the model a *measured* disturbance path of its own.

    On `plants.tank.Tank` the load travels a separate first-order path with its
    own gain, time constant and delay, and the output is the sum of the two
    (``y = h + x_d``). A controller that reads the disturbance needs that second
    model, and it is a genuinely different model from the one it is tuned on --
    the same distinction `FeedforwardPID` is built around, and the reason
    feedforward is not always realisable.

    ``tau_d=None`` means a static path: the disturbance appears at the output
    immediately, scaled by ``Kd``. Any ``theta_d`` becomes its own shift
    register on the disturbance channel.
    """
    if model.Bd is not None:
        raise ValueError("this model already carries a disturbance path")

    nx = model.n_states
    n_theta, note = _delay_samples(theta_d, model.dt)

    if tau_d is None or float(tau_d) <= 0:
        # Static path: one state holding the last (delayed) disturbance value.
        n_new = 1 + n_theta
        Ad_block = np.zeros((n_new, n_new))
        Bd_block = np.zeros((n_new, 1))
        if n_theta == 0:
            Bd_block[0, 0] = float(Kd)
        else:
            Ad_block[0, 1] = float(Kd)
            if n_theta > 1:
                Ad_block[1 : n_theta, 2 : n_theta + 1] = np.eye(n_theta - 1)
            Bd_block[n_theta, 0] = 1.0
        c_row = np.zeros(n_new)
        c_row[0] = 1.0 if n_theta == 0 else 1.0
        path_note = f"static disturbance path Kd={Kd:g}, theta_d={theta_d:g}"
        names = ("d_path",) + tuple(f"d_delay{i}" for i in range(n_theta))
    else:
        tau_d = float(tau_d)
        Ac = np.array([[-1.0 / tau_d]])
        Bc = np.array([[float(Kd) / tau_d]])
        Ad_1, Bd_1 = zoh(Ac, Bc, model.dt)
        n_new = 1 + n_theta
        Ad_block = np.zeros((n_new, n_new))
        Bd_block = np.zeros((n_new, 1))
        Ad_block[0, 0] = Ad_1[0, 0]
        if n_theta == 0:
            Bd_block[0, 0] = Bd_1[0, 0]
        else:
            Ad_block[0, 1] = Bd_1[0, 0]
            if n_theta > 1:
                Ad_block[1 : n_theta, 2 : n_theta + 1] = np.eye(n_theta - 1)
            Bd_block[n_theta, 0] = 1.0
        c_row = np.zeros(n_new)
        c_row[0] = 1.0
        path_note = (
            f"first-order disturbance path Kd={Kd:g}, tau_d={tau_d:g}, theta_d={theta_d:g}"
        )
        names = ("d_path",) + tuple(f"d_delay{i}" for i in range(n_theta))

    A = np.zeros((nx + n_new, nx + n_new))
    A[:nx, :nx] = model.A
    A[nx:, nx:] = Ad_block
    B = np.zeros((nx + n_new, model.n_inputs))
    B[:nx, :] = model.B
    Bd = np.zeros((nx + n_new, 1))
    Bd[nx:, :] = Bd_block
    C = np.zeros((model.n_outputs, nx + n_new))
    C[:, :nx] = model.C
    C[0, nx:] = c_row                      # the disturbance adds at the output

    return DiscreteModel(
        A=A, B=B, C=C, dt=model.dt, Bd=Bd,
        n_delay=model.n_delay,
        theta_requested=model.theta_requested,
        note=" ".join(s for s in (model.note, path_note, note) if s),
        state_names=(tuple(model.state_names) + names) if model.state_names else (),
    )

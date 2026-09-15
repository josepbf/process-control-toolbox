"""State estimation, and the disturbance state that supplies integral action.

`Controller.compute` receives ``y``, not ``x``. Every model-based law in this
project therefore needs an estimator, and the estimator needs one thing beyond
the obvious: an **integrating disturbance state**.

That state is not a refinement. Without it a model-based controller has no
integral action at all and will sit on a steady-state offset after a load
change, exactly as a P-only PID does -- the model says the output should be at
setpoint, the measurement says otherwise, and nothing in the controller is
accumulating the difference. With it, the estimated disturbance *is* the
integral action, and it is the single signal that explains why the output comes
back.

Two timing rules are load-bearing, and both are easy to get quietly wrong:

**Correct before you move.** The harness convention (`simulate.py`) is that the
controller sees ``y(t)`` at the top of the sample and returns ``u(t)`` held over
``[t, t+dt)``. So the measurement must correct the estimate *before* the move is
chosen::

    x_hat = obs.correct(y)      # x(k|k)   -- first, inside compute()
    ...  choose u  ...
    obs.predict(u)              # x(k+1|k) -- last, before returning

A predictor-form observer, correcting one sample late, costs a whole extra
sample of dead time on loops where dead time is the entire difficulty. It passes
almost every other test, and shows up only as the model-based controller losing
-- which then gets blamed on the controller.

**The noise ratio is the integral time.** ``q_disturbance / r_noise`` sets how
fast the disturbance estimate moves: large means aggressive load rejection and
amplified measurement noise, small means sluggish, exactly like a long ``Ti``.
It is a tuning knob, so it is named, recorded and reported -- fairness rule 1
admits no knob that somebody just turned until the plot looked good.

References
----------
Muske, K.R., Badgwell, T.A. (2002). "Disturbance modeling for offset-free
    linear model predictive control." J. Process Control 12(5), 617-632.
Pannocchia, G., Rawlings, J.B. (2003). "Disturbance models for offset-free
    model-predictive control." AIChE J. 49(2), 426-437.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_discrete_are

from .discrete import DiscreteModel


def augment_disturbance(
    model: DiscreteModel,
    kind: str = "output",
    Bd_shape: np.ndarray | None = None,
) -> DiscreteModel:
    """Append integrating disturbance states -- the source of offset-free control.

    Output disturbance (``kind="output"``)::

        A_a = [[A, 0], [0, I]]      C_a = [C, I]

    the upset is assumed to appear *at the measurement*: a sensor bias, a
    composition change downstream of the dynamics, or any load whose path to the
    output is faster than the manipulated path.

    Input disturbance (``kind="input"``)::

        A_a = [[A, Bd], [0, I]]     C_a = [C, 0]

    the upset is assumed to enter *where the valve does*, and is therefore
    rejected through the process dynamics before it reaches the output.

    The two are not equivalent and the difference is physical rather than
    numerical. On `plants.tank.Tank` the load travels its own first-order path
    and adds at the output (``y = h + x_d``), so the output model is
    structurally exact at steady state and close dynamically -- which makes it
    the right default here, and makes the input model the *interesting*
    comparison rather than the obvious one.

    Detectability is checked, not assumed. The augmented pair is detectable iff
    ``n_d <= n_y`` and::

        rank [[A - I, Bd], [C, Cd]] == n + n_d

    (Muske & Badgwell 2002, Lemma 1). The common way to fail it is an
    *integrating* process with an output-disturbance model: the plant's
    integrator and the disturbance integrator are indistinguishable from the
    measurement, and no observer can separate them. Such a loop needs the input
    model.

    Parameters
    ----------
    model : DiscreteModel
        Already discretised and dead-time augmented.
    kind : {"output", "input"}
    Bd_shape : ndarray, optional
        For ``kind="input"``, the column(s) the disturbance enters through.
        Defaults to the model's own ``B`` -- i.e. the upset is assumed to enter
        exactly where the manipulated variable does.
    """
    if model.n_disturbance_states:
        raise ValueError(
            "this model already carries disturbance states; augment the "
            "undisturbed model instead of augmenting twice"
        )

    n, p = model.n_states, model.n_outputs
    if kind == "output":
        n_d = p
        Bd_aug = np.zeros((n, n_d))
        Cd = np.eye(p, n_d)
    elif kind == "input":
        Bd_aug = model.B.copy() if Bd_shape is None else np.atleast_2d(
            np.asarray(Bd_shape, dtype=float)
        )
        if Bd_aug.shape[0] != n:
            raise ValueError(
                f"Bd_shape has {Bd_aug.shape[0]} rows but the state is {n}-dimensional"
            )
        n_d = Bd_aug.shape[1]
        Cd = np.zeros((p, n_d))
    else:
        raise ValueError(f"kind must be 'output' or 'input', got {kind!r}")

    if n_d > p:
        raise ValueError(
            f"cannot estimate {n_d} disturbance state(s) from {p} measurement(s): "
            "an offset-free design needs at most one disturbance state per "
            "measured output (Muske & Badgwell 2002)"
        )

    A = np.block([[model.A, Bd_aug], [np.zeros((n_d, n)), np.eye(n_d)]])
    B = np.vstack([model.B, np.zeros((n_d, model.n_inputs))])
    C = np.hstack([model.C, Cd])

    # Detectability, checked rather than hoped for.
    test = np.block([[model.A - np.eye(n), Bd_aug], [model.C, Cd]])
    if np.linalg.matrix_rank(test, tol=1e-9) < n + n_d:
        raise ValueError(
            f"an {kind} disturbance model is not detectable on this process: the "
            f"rank condition rank([[A - I, Bd], [C, Cd]]) == {n + n_d} fails, so "
            "the disturbance state cannot be distinguished from the process "
            "state by any observer. An integrating process with an output "
            "disturbance model is the usual case -- try kind='input'."
        )

    Bd_measured = None
    if model.Bd is not None:
        Bd_measured = np.vstack([model.Bd, np.zeros((n_d, model.n_disturbances))])

    names = ()
    if model.state_names:
        names = tuple(model.state_names) + tuple(f"d_hat{i}" for i in range(n_d))

    return DiscreteModel(
        A=A, B=B, C=C, dt=model.dt, Bd=Bd_measured,
        n_delay=model.n_delay,
        theta_requested=model.theta_requested,
        note=" ".join(s for s in (model.note, f"{kind}-disturbance augmented") if s),
        state_names=names,
        n_disturbance_states=n_d,
        disturbance_kind=kind,
    )


class KalmanObserver:
    """Steady-state current estimator for a :class:`DiscreteModel`.

    The gain is computed once, from the algebraic Riccati equation, and held --
    a time-varying filter would make two runs of the same controller differ in
    their transient, which is not what a benchmark wants. Supply ``gain``
    directly, or ``poles`` to place them instead, when the Riccati weights are
    not the natural way to say what you mean.

    Usage follows the harness's timing convention exactly::

        x_hat = obs.correct(y)      # x(k|k)   -- first, inside compute()
        ...  choose u  ...
        obs.predict(u)              # x(k+1|k) -- last, before returning

    Parameters
    ----------
    model : DiscreteModel
        Usually already passed through :func:`augment_disturbance`; an observer
        on a model without disturbance states gives a controller with no
        integral action, which is occasionally what you want to demonstrate and
        never what you want to ship.
    q_state, q_disturbance : float or ndarray
        Process-noise variance on the process states and on the disturbance
        states. Their *ratio* to ``r_noise`` is the tuning: ``q_disturbance /
        r_noise`` large means the disturbance estimate moves fast, which rejects
        load changes aggressively and amplifies measurement noise.
    r_noise : float or ndarray
        Measurement-noise variance, per output.
    gain : ndarray, optional
        Use this filter gain ``L`` and skip the Riccati solve entirely.
    poles : sequence of float, optional
        Place the estimator poles instead (Luenberger). Mutually exclusive with
        ``gain``.
    """

    def __init__(
        self,
        model: DiscreteModel,
        *,
        q_state: float | np.ndarray = 1e-4,
        q_disturbance: float | np.ndarray = 1e-2,
        r_noise: float | np.ndarray = 1e-2,
        gain: np.ndarray | None = None,
        poles=None,
        name: str = "Kalman (steady-state)",
    ):
        self.model = model
        self.name = name
        self.q_state = q_state
        self.q_disturbance = q_disturbance
        self.r_noise = r_noise

        n, p = model.n_states, model.n_outputs
        n_d = model.n_disturbance_states

        if gain is not None and poles is not None:
            raise ValueError("give either an explicit gain or poles to place, not both")

        if gain is not None:
            L = np.atleast_2d(np.asarray(gain, dtype=float))
            if L.shape != (n, p):
                raise ValueError(f"gain must have shape ({n}, {p}), got {L.shape}")
            self._how = "explicit gain"
        elif poles is not None:
            from scipy.signal import place_poles

            placed = place_poles(model.A.T, model.C.T, np.asarray(poles, dtype=float))
            L = placed.gain_matrix.T
            self._how = f"poles placed at {tuple(float(z) for z in poles)}"
        else:
            Q = np.zeros((n, n))
            n_model = n - n_d
            Q[:n_model, :n_model] = np.eye(n_model) * np.asarray(q_state, dtype=float)
            if n_d:
                Q[n_model:, n_model:] = np.eye(n_d) * np.asarray(q_disturbance, dtype=float)
            R = np.eye(p) * np.asarray(r_noise, dtype=float)

            # Filter form: the DARE in (A^T, C^T) gives the prediction
            # covariance, and the current-estimator gain follows from it.
            P = solve_discrete_are(model.A.T, model.C.T, Q, R)
            S = model.C @ P @ model.C.T + R
            L = np.linalg.solve(S.T, (P @ model.C.T).T).T
            self._how = (
                f"steady-state Riccati, q_state={q_state:g}, "
                f"q_disturbance={q_disturbance:g}, r_noise={r_noise:g} "
                f"(q_disturbance/r_noise = {float(np.asarray(q_disturbance)) / float(np.asarray(r_noise)):g} "
                "is the integral action)"
            )

        self.L = L
        self.reset()

    # -- state ---------------------------------------------------------------
    def reset(self, x0: np.ndarray | None = None) -> None:
        """Clear the estimate. Called by every controller's own ``reset()``."""
        n = self.model.n_states
        self._x = np.zeros(n) if x0 is None else np.asarray(x0, dtype=float).copy()
        self._innovation = np.zeros(self.model.n_outputs)

    def seed_from_measurement(self, y, u0: float | np.ndarray = 0.0) -> None:
        """Bumpless start: begin from a state consistent with the first reading.

        Puts the whole of ``y`` on the process output, primes the input delay
        queue with ``u0``, and starts the disturbance estimate at zero. Without
        this the estimate starts at the origin, the first innovation is the
        entire operating point, and the controller's first move is a kick that
        contaminates every settling-time metric in the run.
        """
        model = self.model
        x = np.zeros(model.n_states)
        y = np.atleast_1d(np.asarray(y, dtype=float))

        n_delay = int(sum(model.n_delay))
        n_d = model.n_disturbance_states
        n_process = model.n_states - n_delay - n_d

        # Least-squares rather than an index, so this keeps working for a model
        # whose output is a combination of states rather than one of them.
        C_process = model.C[:, :n_process]
        x[:n_process] = np.linalg.lstsq(C_process, y, rcond=None)[0]
        if n_delay:
            x[n_process : n_process + n_delay] = float(np.atleast_1d(u0)[0])
        self._x = x
        self._innovation = np.zeros(model.n_outputs)

    # -- the two halves of a sample ------------------------------------------
    def correct(self, y) -> np.ndarray:
        """Fold in this sample's measurement. Call before choosing the move."""
        y = np.atleast_1d(np.asarray(y, dtype=float))
        self._innovation = y - self.model.C @ self._x
        self._x = self._x + self.L @ self._innovation
        return self._x.copy()

    def predict(self, u, d=None) -> np.ndarray:
        """Roll the estimate forward under the move just issued."""
        self._x = self.model.step(self._x, u, d)
        return self._x.copy()

    # -- what the controller publishes ---------------------------------------
    @property
    def x_hat(self) -> np.ndarray:
        return self._x.copy()

    @property
    def d_hat(self) -> np.ndarray:
        """The estimated disturbance: a model-based controller's integral term."""
        n_d = self.model.n_disturbance_states
        if not n_d:
            return np.zeros(0)
        return self._x[-n_d:].copy()

    @property
    def innovation(self) -> np.ndarray:
        """Measurement minus prediction: how wrong the model just was."""
        return self._innovation.copy()

    def describe(self) -> dict:
        return {
            "observer": self.name,
            "gain": self._how,
            "n_disturbance_states": self.model.n_disturbance_states,
            "disturbance_kind": self.model.disturbance_kind,
        }

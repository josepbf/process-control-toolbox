"""Frequency-domain robustness analysis of a PID loop on a FOPDT process.

Why this module exists: comparing tuning rules on IAE alone is exactly the
trap that fairness rule 1 is written to avoid. Any rule can be made to look
good on a tracking metric by turning the gain up, and the cost of that gain is
not visible in the time-domain metrics of a *nominal* simulation -- it is
visible as reduced robustness, and it gets paid later, when the process gain
drifts with tonnage, moisture or liner wear.

The standard single number for this is the **maximum sensitivity**

    Ms = max over w of | 1 / (1 + L(jw)) |

the peak of the sensitivity function, i.e. the inverse of the shortest
distance from the Nyquist curve of the loop transfer function to the -1 point.
Interpretation, and the reason it is preferred to gain and phase margin
separately: Ms bounds *both* of them at once,

    GM >= Ms/(Ms-1),      PM >= 2*arcsin(1/(2*Ms))

so Ms = 1.4 guarantees GM >= 3.5 and PM >= 42 deg. Typical process practice:
Ms in 1.2-1.6 is comfortable, 1.6-2.0 is aggressive, above 2.0 is a loop that
will oscillate the first time something about the plant changes.

References: Astrom & Hagglund, *Advanced PID Control* (2006), chapter 4;
Skogestad & Postlethwaite, *Multivariable Feedback Control* (2005), chapter 2.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import brentq


def fopdt_response(K: float, tau: float, theta: float, w: np.ndarray) -> np.ndarray:
    """G(jw) = K * exp(-j*w*theta) / (1 + j*w*tau)."""
    w = np.asarray(w, dtype=float)
    return K * np.exp(-1j * w * theta) / (1.0 + 1j * w * tau)


def integrator_response(k_prime: float, theta: float, w: np.ndarray) -> np.ndarray:
    """G(jw) = k' * exp(-j*w*theta) / (j*w), for an integrating process."""
    w = np.asarray(w, dtype=float)
    return k_prime * np.exp(-1j * w * theta) / (1j * w)


def pid_response(
    Kc: float, Ti: float | None, Td: float = 0.0, N: float = 10.0, w: np.ndarray | None = None
) -> np.ndarray:
    """C(jw) for the ideal PID with the derivative filter actually implemented.

    Including the ``Td/N`` filter matters here: an unfiltered derivative has
    unbounded high-frequency gain, which makes Ms meaningless and flatters PID
    rules that lean on derivative action.
    """
    w = np.asarray(w, dtype=float)
    c = np.full(w.shape, complex(Kc))
    if Ti is not None and Ti > 0:
        c = c + Kc / (1j * w * Ti)
    if Td > 0:
        c = c + Kc * (1j * w * Td) / (1.0 + 1j * w * Td / N)
    return c


def _frequency_grid(*time_scales: float, decades: float = 4.0, n: int = 40001) -> np.ndarray:
    scales = [abs(s) for s in time_scales if s is not None and np.isfinite(s) and abs(s) > 0]
    ref = 1.0 / max(scales) if scales else 1.0
    return np.logspace(np.log10(ref) - decades, np.log10(ref) + decades, n)


def loop_metrics(
    G: np.ndarray | None = None,
    C: np.ndarray | None = None,
    w: np.ndarray | None = None,
) -> dict:
    """Ms, Mt, gain margin, phase margin and the crossover frequencies of L=G*C."""
    L = G * C
    S = 1.0 / (1.0 + L)
    T = L * S

    Ms = float(np.max(np.abs(S)))
    Mt = float(np.max(np.abs(T)))
    w_ms = float(w[int(np.argmax(np.abs(S)))])

    mag = np.abs(L)
    phase = np.unwrap(np.angle(L))

    # Phase crossover: first frequency where the phase passes -180 deg.
    gm, w180 = np.inf, np.nan
    idx = np.nonzero((phase[:-1] > -np.pi) & (phase[1:] <= -np.pi))[0]
    if idx.size:
        i = int(idx[0])
        w180 = float(np.interp(-np.pi, [phase[i + 1], phase[i]], [w[i + 1], w[i]]))
        gm = float(1.0 / np.interp(w180, w, mag))

    # Gain crossover: first frequency where |L| falls through 1.
    pm, wc = np.nan, np.nan
    idx = np.nonzero((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
    if idx.size:
        i = int(idx[0])
        wc = float(np.interp(0.0, [np.log(mag[i + 1]), np.log(mag[i])], [w[i + 1], w[i]]))
        pm = float(np.degrees(np.pi + np.interp(wc, w, phase)))

    return {
        "Ms": Ms,
        "Mt": Mt,
        "GM": gm,
        "PM_deg": pm,
        "wc": wc,
        "w180": w180,
        "w_Ms": w_ms,
        # Robustness read straight off Ms, no extra assumptions:
        "GM_from_Ms": Ms / (Ms - 1.0) if Ms > 1.0 else np.inf,
        "PM_from_Ms_deg": float(np.degrees(2.0 * np.arcsin(min(1.0, 1.0 / (2.0 * Ms))))),
    }


def pid_on_fopdt(
    K: float, tau: float, theta: float, Kc: float, Ti: float | None, Td: float = 0.0, N: float = 10.0
) -> dict:
    """Robustness of a PID tuning on a FOPDT plant. The workhorse of exp03."""
    w = _frequency_grid(tau, theta, Ti, Td)
    return loop_metrics(fopdt_response(K, tau, theta, w), pid_response(Kc, Ti, Td, N, w), w)


def pid_on_integrator(
    k_prime: float, theta: float, Kc: float, Ti: float | None, Td: float = 0.0, N: float = 10.0
) -> dict:
    """Robustness of a PID tuning on an integrating plant."""
    # 1/|k'| is the natural time scale of an integrator; the sign only sets
    # the controller action (a reverse-acting loop has k' and Kc both negative).
    w = _frequency_grid(theta, Ti, Td, 1.0 / max(abs(k_prime), 1e-12))
    return loop_metrics(integrator_response(k_prime, theta, w), pid_response(Kc, Ti, Td, N, w), w)


def ultimate_gain_period(K: float, tau: float, theta: float) -> tuple[float, float]:
    """Analytic ultimate gain and period of a FOPDT process under P control.

    Solves the phase condition ``-w*theta - atan(w*tau) = -pi`` for the phase
    crossover ``wu``, then

        Ku = 1/|G(j*wu)| = sqrt(1 + (wu*tau)^2) / K,     Pu = 2*pi/wu

    These are the two numbers the Ziegler-Nichols and Tyreus-Luyben closed-loop
    rules consume. Having them in closed form lets those rules be applied
    without running the relay experiment -- useful as a reference, but note
    that on a real plant only the relay test (``tuning.relay``) can produce
    them, and it produces them from a *describing-function approximation*, so
    the two will not agree exactly.
    """
    if theta <= 0:
        raise ValueError("a FOPDT process with no dead time has no finite ultimate gain")

    def phase_excess(w: float) -> float:
        return -w * theta - np.arctan(w * tau) + np.pi

    lo, hi = 1e-12, 1.0 / theta
    while phase_excess(hi) > 0:
        hi *= 2.0
        if hi > 1e12:
            raise RuntimeError("failed to bracket the phase crossover")
    wu = brentq(phase_excess, lo, hi, xtol=1e-14, rtol=1e-14)
    Ku = np.sqrt(1.0 + (wu * tau) ** 2) / K
    Pu = 2.0 * np.pi / wu
    return float(Ku), float(Pu)

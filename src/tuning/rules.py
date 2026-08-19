"""Named, citable tuning rules.

Fairness rule 1 of this project: no baseline is ever hand-tuned. Every PID in
every reported comparison comes out of one of the functions below, and the rule
(with its citation and its tuning knob) is recorded in the results table. It is
very easy to make MPC look brilliant by detuning the PID; the defence against
that is to fix the tuning method up front and let the number fall where it may.

References
----------
Skogestad, S. (2003). "Simple analytic rules for model reduction and PID
    controller tuning." *Journal of Process Control* 13(4), 291-309.  [SIMC]
Rivera, D.E., Morari, M., Skogestad, S. (1986). "Internal model control: PID
    controller design." *Ind. Eng. Chem. Process Des. Dev.* 25(1), 252-265. [IMC]
Ziegler, J.G., Nichols, N.B. (1942). "Optimum settings for automatic
    controllers." *Trans. ASME* 64, 759-768.  [ZN]
"""

from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass
class PIDTuning:
    """Parameters in the ideal / parallel (ISA) PID form used by PIDController."""

    Kc: float
    Ti: float | None = None
    Td: float = 0.0
    rule: str = ""

    def as_kwargs(self) -> dict:
        return {"Kc": self.Kc, "Ti": self.Ti, "Td": self.Td}

    def to_dict(self) -> dict:
        return asdict(self)


# ----------------------------------------------------------------------
# SIMC -- Skogestad's Internal Model Control rules
# ----------------------------------------------------------------------
def simc_pi(K: float, tau: float, theta: float, tau_c: float | None = None) -> PIDTuning:
    """SIMC PI tuning for a first-order-plus-dead-time process.

        Kc = (1/K) * tau / (tau_c + theta)
        Ti = min( tau, 4*(tau_c + theta) )

    ``tau_c`` is the desired closed-loop time constant and is the *only* knob.
    Skogestad's recommended default for a good robustness/performance trade-off
    is ``tau_c = theta``, which gives roughly a gain margin of ~3 and phase
    margin of ~60 deg. Larger ``tau_c`` = slower and more robust.

    The ``min`` on ``Ti`` is the important half of the rule: for a lag-dominant
    process the pure IMC choice ``Ti = tau`` gives beautiful setpoint tracking
    and dreadful *load disturbance* rejection, because the integral is far too
    slow. Capping ``Ti`` at ``4*(tau_c+theta)`` fixes that.
    """
    if tau_c is None:
        tau_c = theta
    Kc = (1.0 / K) * tau / (tau_c + theta)
    Ti = min(tau, 4.0 * (tau_c + theta))
    return PIDTuning(Kc=Kc, Ti=Ti, Td=0.0, rule=f"SIMC PI (Skogestad 2003), tau_c={tau_c:g}")


def simc_pid(K: float, tau1: float, tau2: float, theta: float, tau_c: float | None = None) -> PIDTuning:
    """SIMC PID for a second-order-plus-dead-time process.

    Skogestad states the rule in the *series* (cascade) PID form; it is
    converted here to the ideal form the controller implements.
    """
    if tau_c is None:
        tau_c = theta
    Kc_s = (1.0 / K) * tau1 / (tau_c + theta)
    Ti_s = min(tau1, 4.0 * (tau_c + theta))
    Td_s = tau2
    Kc, Ti, Td = series_to_ideal(Kc_s, Ti_s, Td_s)
    return PIDTuning(Kc=Kc, Ti=Ti, Td=Td, rule=f"SIMC PID (Skogestad 2003), tau_c={tau_c:g}")


def series_to_ideal(Kc_s: float, Ti_s: float, Td_s: float) -> tuple[float, float, float]:
    """Convert series/cascade PID parameters to the ideal (parallel) form."""
    f = 1.0 + Td_s / Ti_s
    return Kc_s * f, Ti_s * f, Td_s / f


# ----------------------------------------------------------------------
# IMC -- Rivera/Morari/Skogestad
# ----------------------------------------------------------------------
def imc_pi(K: float, tau: float, theta: float, tau_c: float | None = None) -> PIDTuning:
    """IMC-PI for FOPDT using a first-order Pade approximation of the delay.

        Kc = (1/K) * (tau + theta/2) / (tau_c + theta/2),   Ti = tau + theta/2

    Compared with SIMC this keeps ``Ti`` equal to the (Pade-corrected) process
    lag, which tracks setpoints cleanly but rejects load disturbances slowly on
    lag-dominant processes.
    """
    if tau_c is None:
        tau_c = max(theta, 0.2 * tau)
    Ti = tau + theta / 2.0
    Kc = (1.0 / K) * Ti / (tau_c + theta / 2.0)
    return PIDTuning(Kc=Kc, Ti=Ti, Td=0.0, rule=f"IMC-PI (Rivera et al. 1986), tau_c={tau_c:g}")


def imc_pid(K: float, tau: float, theta: float, tau_c: float | None = None) -> PIDTuning:
    """IMC-PID for FOPDT (first-order Pade)."""
    if tau_c is None:
        tau_c = max(theta, 0.2 * tau)
    Ti = tau + theta / 2.0
    Kc = (1.0 / K) * Ti / (tau_c + theta / 2.0)
    Td = tau * theta / (2.0 * tau + theta)
    return PIDTuning(Kc=Kc, Ti=Ti, Td=Td, rule=f"IMC-PID (Rivera et al. 1986), tau_c={tau_c:g}")


# ----------------------------------------------------------------------
# Ziegler-Nichols open-loop (reaction curve)
# ----------------------------------------------------------------------
def ziegler_nichols_open_loop(K: float, tau: float, theta: float, kind: str = "PI") -> PIDTuning:
    """Classic 1942 reaction-curve rules, targeting quarter-amplitude decay.

    Included as a deliberately aggressive reference point: ZN is famously
    oscillatory and is not a fair "best classical" baseline, but it is the rule
    most people picture when they hear "tuned PID", so it is worth having.
    """
    kind = kind.upper()
    if kind == "P":
        return PIDTuning(Kc=tau / (K * theta), Ti=None, Td=0.0, rule="ZN open-loop P (1942)")
    if kind == "PI":
        return PIDTuning(
            Kc=0.9 * tau / (K * theta), Ti=3.33 * theta, Td=0.0, rule="ZN open-loop PI (1942)"
        )
    if kind == "PID":
        return PIDTuning(
            Kc=1.2 * tau / (K * theta), Ti=2.0 * theta, Td=0.5 * theta, rule="ZN open-loop PID (1942)"
        )
    raise ValueError(f"unknown ZN variant {kind!r}")


# ----------------------------------------------------------------------
# Cohen-Coon -- open-loop, built for dead-time-dominant processes
# ----------------------------------------------------------------------
def cohen_coon(K: float, tau: float, theta: float, kind: str = "PI") -> PIDTuning:
    """Cohen & Coon (1953) reaction-curve rules.

    Like Ziegler-Nichols these target quarter-amplitude decay, but they were
    derived for processes where the dead time is a large fraction of the lag,
    which is where the ZN rules become wildly aggressive. Expect Cohen-Coon to
    be gentler than ZN when ``theta/tau`` is large and comparable when it is
    small -- and still oscillatory by modern standards, because
    quarter-amplitude decay is a 1950s taste, not a robustness specification.
    """
    r = theta / tau
    kind = kind.upper()
    if kind == "P":
        return PIDTuning(Kc=(1 / K) * (1 / r) * (1 + r / 3.0), Ti=None, Td=0.0,
                         rule="Cohen-Coon P (1953)")
    if kind == "PI":
        Kc = (1 / K) * (1 / r) * (0.9 + r / 12.0)
        Ti = theta * (30.0 + 3.0 * r) / (9.0 + 20.0 * r)
        return PIDTuning(Kc=Kc, Ti=Ti, Td=0.0, rule="Cohen-Coon PI (1953)")
    if kind == "PID":
        Kc = (1 / K) * (1 / r) * (4.0 / 3.0 + r / 4.0)
        Ti = theta * (32.0 + 6.0 * r) / (13.0 + 8.0 * r)
        Td = theta * 4.0 / (11.0 + 2.0 * r)
        return PIDTuning(Kc=Kc, Ti=Ti, Td=Td, rule="Cohen-Coon PID (1953)")
    raise ValueError(f"unknown Cohen-Coon variant {kind!r}")


# ----------------------------------------------------------------------
# Closed-loop (ultimate gain / ultimate period) rules
# ----------------------------------------------------------------------
def ziegler_nichols_closed_loop(Ku: float, Pu: float, kind: str = "PI") -> PIDTuning:
    """The original 1942 continuous-cycling rules.

    ``Ku`` is the proportional gain at which the loop oscillates with constant
    amplitude, and ``Pu`` the period of that oscillation. Getting them requires
    pushing a real plant to the edge of instability, which is why the relay
    experiment (see ``tuning.relay``) replaced this procedure industrially: it
    finds the same two numbers from a bounded, safe oscillation.
    """
    kind = kind.upper()
    if kind == "P":
        return PIDTuning(Kc=0.5 * Ku, Ti=None, Td=0.0, rule="ZN closed-loop P (1942)")
    if kind == "PI":
        return PIDTuning(Kc=0.45 * Ku, Ti=Pu / 1.2, Td=0.0, rule="ZN closed-loop PI (1942)")
    if kind == "PID":
        return PIDTuning(Kc=0.6 * Ku, Ti=Pu / 2.0, Td=Pu / 8.0, rule="ZN closed-loop PID (1942)")
    raise ValueError(f"unknown ZN variant {kind!r}")


def tyreus_luyben(Ku: float, Pu: float, kind: str = "PI") -> PIDTuning:
    """Tyreus & Luyben (1992): the same experiment, a chemical engineer's taste.

    Deliberately much more conservative than Ziegler-Nichols -- roughly half
    the gain and twice the integral time -- because quarter-amplitude decay is
    unacceptably oscillatory on real process plant, where loops are coupled and
    the model drifts with operating point.
    """
    kind = kind.upper()
    if kind == "PI":
        return PIDTuning(Kc=Ku / 3.2, Ti=2.2 * Pu, Td=0.0, rule="Tyreus-Luyben PI (1992)")
    if kind == "PID":
        return PIDTuning(Kc=Ku / 2.2, Ti=2.2 * Pu, Td=Pu / 6.3, rule="Tyreus-Luyben PID (1992)")
    raise ValueError(f"unknown Tyreus-Luyben variant {kind!r}")


# ----------------------------------------------------------------------
# Lambda (Dahlin) tuning -- the DCS default in pulp, paper and cement
# ----------------------------------------------------------------------
def lambda_tuning(K: float, tau: float, theta: float, lam: float | None = None) -> PIDTuning:
    """Lambda tuning for FOPDT: ``Kc = tau / (K*(lam+theta))``, ``Ti = tau``.

    ``lam`` is the desired closed-loop time constant; plant practice is
    ``lam = tau`` for a "safe" loop and ``lam = 3*theta`` for a faster one.

    Note what is *not* here: the ``min`` that SIMC puts on ``Ti``. Cancelling
    the process lag exactly (``Ti = tau``) gives a clean first-order setpoint
    response and, on a lag-dominant process, poor load rejection -- the
    integral is then far slower than the disturbance. Lambda tuning is included
    precisely because it is so widely deployed and because this experiment
    shows the price it pays.
    """
    if lam is None:
        lam = tau
    Kc = tau / (K * (lam + theta))
    return PIDTuning(Kc=Kc, Ti=tau, Td=0.0, rule=f"Lambda/Dahlin PI, lambda={lam:g}")


# ----------------------------------------------------------------------
# AMIGO -- the modern robustness-constrained rule
# ----------------------------------------------------------------------
def amigo_pi(K: float, tau: float, theta: float) -> PIDTuning:
    """AMIGO PI (Astrom & Hagglund 2004).

    Fitted to a large batch of representative process models under an explicit
    robustness constraint (maximum sensitivity Ms <= 1.4), rather than to a
    decay-ratio taste. It is the fairest single "modern classical" baseline
    available, and a reasonable default when the loop must not be re-tuned.
    """
    Kc = (0.15 / K) + (0.35 - theta * tau / (theta + tau) ** 2) * (tau / (K * theta))
    Ti = 0.35 * theta + (13.0 * theta * tau ** 2) / (tau ** 2 + 12.0 * theta * tau + 7.0 * theta ** 2)
    return PIDTuning(Kc=Kc, Ti=Ti, Td=0.0, rule="AMIGO PI (Astrom & Hagglund 2004)")


def amigo_pid(K: float, tau: float, theta: float) -> PIDTuning:
    """AMIGO PID (Astrom & Hagglund 2004), targeting Ms <= 1.4."""
    Kc = (1.0 / K) * (0.2 + 0.45 * tau / theta)
    Ti = (0.4 * theta + 0.8 * tau) / (theta + 0.1 * tau) * theta
    Td = 0.5 * theta * tau / (0.3 * theta + tau)
    return PIDTuning(Kc=Kc, Ti=Ti, Td=Td, rule="AMIGO PID (Astrom & Hagglund 2004)")


# ----------------------------------------------------------------------
# Integrating processes
# ----------------------------------------------------------------------
def simc_integrating(k_prime: float, theta: float, tau_c: float | None = None) -> PIDTuning:
    """SIMC PI for an integrating process ``G(s) = k' * exp(-theta*s) / s``.

        Kc = 1 / (k' * (tau_c + theta)),    Ti = 4 * (tau_c + theta)

    A surge tank, a drum level, a silo: the output does not come back on its
    own, so there is no ``tau`` to cancel and the FOPDT rules do not apply. Any
    rule that sets ``Ti`` from a process lag will hand an integrating loop an
    integral time far too long and produce a slow, rolling oscillation -- the
    classic badly-tuned level loop.
    """
    if tau_c is None:
        tau_c = theta
    return PIDTuning(
        Kc=1.0 / (k_prime * (tau_c + theta)),
        Ti=4.0 * (tau_c + theta),
        Td=0.0,
        rule=f"SIMC PI integrating (Skogestad 2003), tau_c={tau_c:g}",
    )


def averaging_level_pi(k_prime: float, v_max: float, y_max_dev: float) -> PIDTuning:
    """P-only 'averaging level' control: use the tank, do not fight it.

    On a surge tank the level is not a product-quality variable -- the tank
    exists to *absorb* flow variation so the downstream unit sees a smooth
    feed. Tight level control defeats the equipment it is installed on. The
    right design is the loosest proportional gain that still keeps the level
    inside its alarm band for the largest expected flow upset ``v_max``:

        Kc = v_max / y_max_dev

    (Skogestad 2003, section on averaging level control; the same reasoning
    appears in Buckley's 1964 book as 'averaging level control'.)
    """
    return PIDTuning(
        Kc=v_max / y_max_dev,
        Ti=None,
        Td=0.0,
        rule=f"averaging level P-only, |dev| <= {y_max_dev:g} for upset {v_max:g}",
    )


# ----------------------------------------------------------------------
# Model reduction
# ----------------------------------------------------------------------
def half_rule(
    taus: list[float], theta: float = 0.0, inverse_zeros: list[float] | None = None, dt: float = 0.0
) -> tuple[float, float]:
    """Skogestad's half rule: reduce a high-order model to FOPDT.

    Given lags sorted largest-first, the largest *neglected* lag is split in
    half: one half is added to the retained time constant, the other half to
    the effective dead time. Every smaller lag, every right-half-plane
    (inverse-response) zero, and half the sample time go entirely into the dead
    time.

        tau_eff   = tau1 + tau2/2
        theta_eff = theta + tau2/2 + sum(tau_i, i>=3) + sum(inverse zeros) + dt/2

    This is the step that lets a two-parameter tuning rule be applied honestly
    to a process that is not first order -- and it is also a preview of the
    core argument of this project: everything a PID cannot represent about the
    process gets swept into an effective dead time, and dead time is precisely
    what feedback handles worst.

    Returns ``(tau_eff, theta_eff)``.
    """
    taus = sorted((float(t) for t in taus), reverse=True)
    inverse_zeros = [float(z) for z in (inverse_zeros or [])]
    if not taus:
        return 0.0, theta + sum(inverse_zeros) + dt / 2.0

    tau1 = taus[0]
    tau2 = taus[1] if len(taus) > 1 else 0.0
    tau_eff = tau1 + tau2 / 2.0
    theta_eff = theta + tau2 / 2.0 + sum(taus[2:]) + sum(inverse_zeros) + dt / 2.0
    return tau_eff, theta_eff

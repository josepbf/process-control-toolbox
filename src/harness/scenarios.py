"""Scenario definitions: what the world does to the loop.

A scenario fixes the sample time, the run length, the setpoint programme and
the disturbance programme. Fairness rule 2 of the project: every controller in
a comparison is run against the *same* scenario object and the same RNG seed,
so differences in the metrics come from the control law and nothing else.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np


def constant(value) -> Callable[[float], np.ndarray]:
    """Signal that never changes."""
    v = np.atleast_1d(np.asarray(value, dtype=float))
    return lambda t: v.copy()


def staircase(changes: list[tuple[float, object]]) -> Callable[[float], np.ndarray]:
    """Piecewise-constant signal.

    ``changes`` is a list of ``(time, value)`` pairs, held from ``time`` until
    the next entry. The first entry must be at t = 0.
    """
    times = np.array([c[0] for c in changes], dtype=float)
    values = [np.atleast_1d(np.asarray(c[1], dtype=float)) for c in changes]
    if times[0] != 0.0:
        raise ValueError("staircase must define a value at t = 0")
    if np.any(np.diff(times) <= 0):
        raise ValueError("staircase times must be strictly increasing")

    def signal(t: float) -> np.ndarray:
        idx = int(np.searchsorted(times, t, side="right") - 1)
        return values[idx].copy()

    return signal


def pulse(t_on: float, t_off: float, amplitude, baseline=0.0) -> Callable[[float], np.ndarray]:
    """Rectangular pulse: ``amplitude`` between ``t_on`` and ``t_off``."""
    amp = np.atleast_1d(np.asarray(amplitude, dtype=float))
    base = np.atleast_1d(np.asarray(baseline, dtype=float)) * np.ones_like(amp)

    def signal(t: float) -> np.ndarray:
        return amp.copy() if t_on <= t < t_off else base.copy()

    return signal


@dataclass
class Scenario:
    """A reproducible closed-loop test case."""

    name: str
    dt: float
    t_final: float
    setpoint: Callable[[float], np.ndarray]
    disturbance: Callable[[float], np.ndarray] = field(default_factory=lambda: constant(0.0))
    seed: int = 0
    #: Noise on the *measurement* of the disturbance, for feedforward
    #: controllers. Zero means a perfectly measured disturbance, which is an
    #: idealised upper bound on what feedforward can do and should be labelled
    #: as such when reported.
    d_noise_std: float = 0.0
    description: str = ""
    #: Named time windows for reporting metrics separately, e.g. the setpoint
    #: change and the load disturbance judged on their own terms.
    windows: dict[str, tuple[float, float]] = field(default_factory=dict)

    @property
    def n_steps(self) -> int:
        return int(round(self.t_final / self.dt))

    @property
    def time(self) -> np.ndarray:
        return np.arange(self.n_steps + 1) * self.dt


# ----------------------------------------------------------------------
# Phase 1 standard test
# ----------------------------------------------------------------------
def setpoint_and_load(
    dt: float = 1.0,
    y_start: float = 30.0,
    y_step: float = 50.0,
    t_step: float = 100.0,
    d_load: float = -12.0,
    t_load: float = 700.0,
    t_final: float = 1400.0,
    seed: int = 0,
) -> Scenario:
    """The phase-1 workhorse: one setpoint change, then one load disturbance.

    These are the two jobs every regulatory loop actually has, and they are in
    tension: tuning that tracks a setpoint elegantly is often sluggish against
    load upsets (this is exactly what the ``min`` in the SIMC ``Ti`` rule is
    there to fix). Reporting them in separate windows keeps that visible.
    """
    return Scenario(
        name="setpoint_and_load",
        dt=dt,
        t_final=t_final,
        setpoint=staircase([(0.0, y_start), (t_step, y_step)]),
        disturbance=staircase([(0.0, 0.0), (t_load, d_load)]),
        seed=seed,
        description=(
            f"level setpoint {y_start:g} -> {y_step:g} %% at t={t_step:g}s; "
            f"unmeasured load step d={d_load:g} at t={t_load:g}s"
        ),
        windows={
            "setpoint": (t_step, t_load),
            "disturbance": (t_load, t_final),
        },
    )

"""Relay feedback auto-tuning (Astrom & Hagglund, Automatica 20(5), 1984).

The Ziegler-Nichols closed-loop rules need two numbers, the ultimate gain
``Ku`` and the ultimate period ``Pu``, and the original way to get them was to
raise the proportional gain until the loop oscillated -- on a real plant,
deliberately walking to the edge of instability with no guarantee of stopping
there. The relay experiment replaced it, and is what sits behind the "autotune"
button on essentially every industrial controller.

Method: replace the controller with a relay of amplitude ``h``. The loop
settles into a limit cycle whose frequency is, by construction, the frequency
at which the process has -180 degrees of phase -- the ultimate frequency. The
oscillation is *bounded*, because its amplitude is set by the relay amplitude
the engineer chose, and it can be made as small as the measurement noise
allows.

Describing-function analysis gives the ultimate gain from the amplitude ``a``
of the resulting oscillation:

    Ku = 4h / (pi * a)                    ideal relay
    Ku = 4h / (pi * sqrt(a^2 - eps^2))    relay with hysteresis eps

The hysteresis is not optional in practice: without it, measurement noise
chatters the relay and the experiment measures the noise. Its cost is a small
bias -- the oscillation is a little slower than the true ultimate frequency,
so ``Ku`` comes out slightly conservative.

The approximation to keep in mind: the describing function assumes the process
filters the relay's harmonics well enough that only the fundamental matters. On
a lag-dominant process that is excellent; on a dead-time-dominant one the
estimate can be off by 10-20 %%, which is why the result of an autotune is
usually passed through a conservative rule (Tyreus-Luyben) rather than the
original Ziegler-Nichols settings.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from ..controllers.onoff import OnOffController
from ..harness.scenarios import Scenario, constant
from ..harness.simulate import simulate


@dataclass
class RelayResult:
    """Outcome of a relay experiment."""

    Ku: float
    Pu: float
    amplitude: float
    n_cycles: int
    periods: np.ndarray
    amplitudes: np.ndarray
    df: pd.DataFrame

    def summary(self) -> str:
        return (
            f"Ku = {self.Ku:.3f}, Pu = {self.Pu:.1f} s from {self.n_cycles} cycles "
            f"(amplitude {self.amplitude:.2f}, period spread "
            f"{np.std(self.periods):.1f} s)"
        )


def _relay_switch_times(t: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Times at which the relay switched *on*, i.e. the rising edges of u.

    The period is measured from the relay output rather than from zero
    crossings of the measurement, because the relay output is clean by
    construction -- its hysteresis has already rejected the noise -- while the
    measurement crosses its setpoint several times per transition when the
    noise is comparable to the local slope. A real autotuner reads its own
    output for exactly this reason.
    """
    u = np.asarray(u, float)
    hi = u > 0.5 * (u.max() + u.min())
    idx = np.nonzero((~hi[:-1]) & hi[1:])[0]
    return t[idx + 1]


def relay_autotune(
    plant,
    setpoint: float,
    u_bias: float,
    h: float,
    dt: float = 1.0,
    hysteresis: float = 0.0,
    t_final: float = 1200.0,
    settle_cycles: int = 1,
    seed: int = 0,
) -> RelayResult:
    """Run a relay experiment on ``plant`` and return the ultimate gain/period.

    Parameters
    ----------
    setpoint : the operating point to oscillate about; the plant should start there.
    u_bias   : the valve position that holds that operating point.
    h        : relay amplitude. The oscillation in ``y`` scales with it, so it
               trades identification quality against how much the experiment
               upsets production -- the central practical decision of an
               autotune, and the reason operators are asked before pressing it.
    hysteresis : relay deadband, in units of ``y``. Rule of thumb: 2-3 times
               the peak-to-peak measurement noise.
    settle_cycles : leading cycles discarded before averaging, so the estimate
               uses the established limit cycle rather than the approach to it.
    """
    relay = OnOffController(
        u_on=u_bias + h, u_off=u_bias - h, hysteresis=hysteresis, name="relay"
    )
    scenario = Scenario(
        name="relay_test", dt=dt, t_final=t_final,
        setpoint=constant(setpoint), disturbance=constant(0.0), seed=seed,
        description=f"relay experiment, h={h:g}, hysteresis={hysteresis:g}",
    )
    df = simulate(plant, relay, scenario)

    t = df["t"].to_numpy(float)
    switches = _relay_switch_times(t, df["u"].to_numpy(float))

    if len(switches) < settle_cycles + 2:
        raise RuntimeError(
            f"relay experiment produced only {max(len(switches) - 1, 0)} cycles; "
            "increase t_final or the relay amplitude h"
        )

    switches = switches[settle_cycles:]
    periods = np.diff(switches)

    # Peak-to-peak amplitude within each complete cycle.
    y = df["y"].to_numpy(float)
    sp = df["sp"].to_numpy(float)
    amplitudes = []
    for c0, c1 in zip(switches[:-1], switches[1:]):
        m = (t >= c0) & (t < c1)
        if m.sum() > 2:
            amplitudes.append(0.5 * (np.max(y[m] - sp[m]) - np.min(y[m] - sp[m])))
    amplitudes = np.array(amplitudes)

    a = float(np.mean(amplitudes))
    Pu = float(np.mean(periods))
    inner = a ** 2 - hysteresis ** 2
    if inner <= 0:
        raise RuntimeError(
            "relay hysteresis is as large as the oscillation it produced; "
            "reduce the hysteresis or increase h"
        )
    Ku = 4.0 * h / (np.pi * np.sqrt(inner))

    return RelayResult(
        Ku=float(Ku), Pu=Pu, amplitude=a, n_cycles=len(periods),
        periods=periods, amplitudes=amplitudes, df=df,
    )

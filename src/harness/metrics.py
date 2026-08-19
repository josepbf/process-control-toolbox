"""Performance metrics.

Project rule (section 7): **every reported comparison carries a tracking metric
and a control-effort metric.** Tracking bought with violent valve movement is
not a win -- in a real plant it destroys the actuator. Total variation of ``u``
and the actuator-reversal count are the numbers that expose that trade, and
they are the reason an ON/OFF controller can look acceptable on IAE alone while
being obviously unusable.
"""

from __future__ import annotations

import numpy as np
import pandas as pd


def _cols(df: pd.DataFrame, prefix: str) -> list[str]:
    if prefix in df.columns:
        return [prefix]
    cols = [c for c in df.columns if c.startswith(prefix) and c[len(prefix):].isdigit()]
    return sorted(cols, key=lambda c: int(c[len(prefix):]))


def signal(df: pd.DataFrame, prefix: str) -> np.ndarray:
    """Return signal ``prefix`` as an (n_samples, n_channels) array."""
    cols = _cols(df, prefix)
    if not cols:
        raise KeyError(f"no columns for signal {prefix!r}")
    return df[cols].to_numpy(dtype=float)


def _window(df: pd.DataFrame, window: tuple[float, float] | None) -> pd.DataFrame:
    if window is None:
        return df
    t0, t1 = window
    return df[(df["t"] >= t0) & (df["t"] <= t1)]


# ----------------------------------------------------------------------
# tracking
# ----------------------------------------------------------------------
def iae(t, e) -> float:
    """Integral of absolute error. The default 'how well did it track' number."""
    return float(np.trapezoid(np.abs(e), t))


def ise(t, e) -> float:
    """Integral of squared error -- punishes large excursions harder than IAE."""
    return float(np.trapezoid(e ** 2, t))


def itae(t, e) -> float:
    """Time-weighted IAE -- punishes long tails, so it dislikes sluggish
    settling and offset far more than it dislikes an early transient."""
    return float(np.trapezoid(np.abs(t - t[0]) * np.abs(e), t))


def noise_sigma(y) -> float:
    """Robust estimate of the measurement noise standard deviation.

    Uses the median absolute deviation of the one-sample differences: real
    process motion is smooth and contributes only a few large differences,
    while noise contributes many small ones, so the *median* is dominated by
    noise even when the window contains a transient. The 1.4826 factor converts
    MAD to sigma for Gaussian data, and the sqrt(2) removes the variance
    doubling introduced by differencing.
    """
    dy = np.diff(np.asarray(y, float).ravel())
    if dy.size == 0:
        return 0.0
    mad = float(np.median(np.abs(dy - np.median(dy))))
    return 1.4826 * mad / np.sqrt(2.0)


def settling_time(
    t, y, sp, tol: float = 0.02, step_size: float | None = None, dwell_fraction: float = 0.05
) -> float:
    """Time until |y - sp| stays inside the settling band, permanently.

    The band is built in two steps.

    *Scale.* The nominal band is ``tol`` times the size of the excursion being
    settled -- the setpoint step where there is one, otherwise the largest
    deviation in the window. That generalisation is what lets the same metric
    describe a load-disturbance recovery, where the setpoint never moves and a
    band defined as a fraction of the step would be zero.

    *Noise floor.* A "last sample outside the band" statistic is fragile if the
    band sits at a fixed few sigma of measurement noise: a clean Gaussian
    sequence of length n wanders out to about ``sqrt(2 ln n)`` sigma by chance,
    so on a run of a few thousand samples a perfectly settled loop would be
    reported as never settling. The band is therefore raised to that expected
    extreme plus a sigma of margin.

    Known limitation: the noise estimator works on one-sample differences, so
    an oscillation whose period approaches a few samples is indistinguishable
    from white noise and would be absorbed into the band. Real limit cycles on
    a lagged process are many samples long and are caught correctly (they come
    back as NaN, i.e. never settled), but a settling time reported for a
    near-Nyquist oscillation should not be trusted.

    Settling is only declared if the loop then *stays* in the band for at least
    ``dwell_fraction`` of the window. Without that dwell requirement a slow
    oscillation gets a settling time whenever its last sample happens to land
    near the setpoint, which is an artefact of where the run was cut off.
    Otherwise the result is NaN -- the honest answer for a limit-cycling
    ON/OFF controller.
    """
    if step_size is None:
        step_size = abs(sp[-1] - y[0])
    scale = max(abs(step_size), float(np.max(np.abs(y - sp))), 1e-12)
    n = max(len(y), 2)
    k = np.sqrt(2.0 * np.log(n)) + 1.0
    band = max(tol * scale, k * noise_sigma(y), 1e-12)

    outside = np.abs(y - sp) > band
    if not outside.any():
        return 0.0
    last = int(np.max(np.nonzero(outside)[0]))
    if last >= len(t) - 1:
        return float("nan")
    t_settle = float(t[last + 1])
    if (t[-1] - t_settle) < dwell_fraction * (t[-1] - t[0]):
        return float("nan")
    return float(t_settle - t[0])


def overshoot(y, sp, y_start: float | None = None) -> float:
    """Peak excursion beyond the setpoint, as a %% of the step size.

    Returns NaN when the window contains no meaningful setpoint change (the
    test being that the step is small compared with the excursions actually
    seen). Overshoot of a step that never happened is not a number worth
    printing -- for a load-disturbance window, read ``peak_dev`` instead.
    """
    if y_start is None:
        y_start = y[0]
    step = sp[-1] - y_start
    excursion = float(np.max(np.abs(y - sp)))
    if abs(step) < max(1e-9, 0.5 * excursion):
        return float("nan")
    peak = np.max(y - sp[-1]) if step > 0 else -np.min(y - sp[-1])
    return float(max(0.0, peak / abs(step)) * 100.0)


def peak_deviation(y, sp) -> float:
    """Largest |y - sp| in the window. The right headline number for a load
    disturbance, where 'overshoot' has no meaning."""
    return float(np.max(np.abs(y - sp)))


def steady_state_offset(t, y, sp, fraction: float = 0.1) -> float:
    """Mean residual error over the last ``fraction`` of the window."""
    n = max(1, int(len(t) * fraction))
    return float(np.mean(sp[-n:] - y[-n:]))


# ----------------------------------------------------------------------
# control effort
# ----------------------------------------------------------------------
def total_variation(u) -> float:
    """Sum |du| -- total distance the actuator travelled."""
    return float(np.sum(np.abs(np.diff(u, axis=0))))


def max_move(u) -> float:
    """Largest single-sample move."""
    du = np.diff(u, axis=0)
    return 0.0 if du.size == 0 else float(np.max(np.abs(du)))


def reversals(u, eps: float = 1e-9) -> int:
    """Number of direction changes of the actuator.

    Valve and damper wear tracks reversals far more closely than it tracks
    travel, and this is the metric that makes ON/OFF limit cycling impossible
    to hide. Moves smaller than ``eps`` are ignored: on a noisy measurement any
    proportional controller dithers by a fraction of a percent every sample,
    and counting that as wear would swamp the comparison with something no
    plant operator would ever notice. ``compute_metrics`` sets ``eps`` to 0.5 %%
    of the actuator span.
    """
    du = np.diff(np.asarray(u, float).ravel())
    du = du[np.abs(du) > eps]
    if du.size < 2:
        return 0
    return int(np.sum(np.sign(du[1:]) != np.sign(du[:-1])))


# ----------------------------------------------------------------------
# summary
# ----------------------------------------------------------------------
def compute_metrics(df: pd.DataFrame, window: tuple[float, float] | None = None) -> dict:
    """All phase-1 metrics for one run (single-output loops)."""
    w = _window(df, window)
    t = w["t"].to_numpy(float)
    y = signal(w, "y")[:, 0]
    sp = signal(w, "sp")[:, 0]
    u = signal(w, "u")[:, 0]
    e = sp - y
    solve = w["solve_time"].to_numpy(float)
    solve = solve[np.isfinite(solve)]

    # Reversal threshold: 0.5 %% of the actuator span, so measurement-noise
    # dither is not counted as valve wear.
    u_min, u_max = df.attrs.get("u_min"), df.attrs.get("u_max")
    if u_min is not None and u_max is not None and np.isfinite(u_max[0] - u_min[0]):
        eps = 0.005 * (u_max[0] - u_min[0])
    else:
        eps = 1e-9

    return {
        "IAE": iae(t, e),
        "ISE": ise(t, e),
        "ITAE": itae(t, e),
        "settling_2pct_s": settling_time(t, y, sp),
        "overshoot_pct": overshoot(y, sp),
        "peak_dev": peak_deviation(y, sp),
        "ss_offset": steady_state_offset(t, y, sp),
        "TV_u": total_variation(u),
        "max_du": max_move(u),
        "reversals": reversals(u, eps=eps),
        "n_violations": int(w["violated"].sum()),
        "violation_integral": float(np.trapezoid(w["violation"].to_numpy(float), t)),
        "solve_ms_mean": float(np.mean(solve) * 1e3) if solve.size else np.nan,
        "solve_ms_p95": float(np.percentile(solve, 95) * 1e3) if solve.size else np.nan,
    }


def summarize(
    runs: dict[str, pd.DataFrame],
    windows: dict[str, tuple[float, float]] | None = None,
    include_full: bool = True,
) -> pd.DataFrame:
    """Metric table over several runs and several named time windows."""
    windows = dict(windows or {})
    if include_full:
        windows = {"full": None, **windows}

    rows = []
    for label, df in runs.items():
        for wname, w in windows.items():
            rows.append(
                {
                    "controller": label,
                    "window": wname,
                    **compute_metrics(df, w),
                    "tuning": df.attrs.get("tuning", ""),
                }
            )
    return pd.DataFrame(rows).set_index(["controller", "window"])


def format_table(table: pd.DataFrame, columns: list[str] | None = None) -> str:
    """Compact console rendering of a metric table."""
    columns = columns or [
        "IAE", "ITAE", "settling_2pct_s", "overshoot_pct", "peak_dev",
        "ss_offset", "TV_u", "reversals", "solve_ms_mean",
    ]
    view = table[columns].copy()
    return view.to_string(float_format=lambda v: f"{v:9.3f}")

"""Standard figures for closed-loop runs.

Every comparison figure in the project has the same three-panel layout, on a
shared time axis:

    1. controlled variable and setpoint (plus any declared output limit)
    2. manipulated variable, with the actuator limits drawn in
    3. the disturbance that was applied

Panel 2 is not optional decoration. A tracking plot on its own systematically
flatters aggressive controllers; showing the valve underneath it is what makes
the control-effort cost visible at a glance.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: figures are files, not windows
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from .metrics import signal  # noqa: E402
from .scenarios import Scenario  # noqa: E402


def plot_runs(
    runs: dict[str, pd.DataFrame],
    title: str = "",
    scenario: Scenario | None = None,
    y_label: str = "level y [%]",
    u_label: str = "valve u [%]",
    path: str | Path | None = None,
    figsize: tuple[float, float] = (11.0, 8.0),
):
    """Overlay several controllers on the standard three-panel layout."""
    fig, axes = plt.subplots(
        3, 1, figsize=figsize, sharex=True, gridspec_kw={"height_ratios": [3, 2, 1]}
    )
    ax_y, ax_u, ax_d = axes

    first = next(iter(runs.values()))
    t = first["t"].to_numpy(float)
    ax_y.plot(t, signal(first, "sp")[:, 0], "k--", lw=1.4, label="setpoint", zorder=1)

    for label, df in runs.items():
        tt = df["t"].to_numpy(float)
        ax_y.plot(tt, signal(df, "y")[:, 0], lw=1.6, label=label)
        ax_u.step(tt, signal(df, "u")[:, 0], where="post", lw=1.4, label=label)

    # Declared output band (reporting-only in phase 1) and actuator limits.
    for lim, name in ((first.attrs.get("y_min"), "y_min"), (first.attrs.get("y_max"), "y_max")):
        if lim is not None:
            ax_y.axhline(lim[0], color="crimson", ls=":", lw=1.2)
            ax_y.text(t[-1], lim[0], f" {name}", color="crimson", va="center", fontsize=8)
    for lim in (first.attrs.get("u_min"), first.attrs.get("u_max")):
        if lim is not None and np.isfinite(lim[0]):
            ax_u.axhline(lim[0], color="grey", ls=":", lw=1.2)

    ax_d.step(t, signal(first, "d")[:, 0], where="post", color="darkorange", lw=1.4)

    if scenario is not None:
        for wname, (w0, w1) in scenario.windows.items():
            for ax in axes:
                ax.axvline(w0, color="grey", alpha=0.35, lw=0.9)
            ax_y.text(
                w0, ax_y.get_ylim()[1], f" {wname}", va="top", fontsize=8, color="grey"
            )

    ax_y.set_ylabel(y_label)
    ax_y.legend(loc="best", fontsize=9)
    ax_y.grid(alpha=0.3)
    ax_u.set_ylabel(u_label)
    ax_u.grid(alpha=0.3)
    ax_d.set_ylabel("disturbance d")
    ax_d.set_xlabel("time [s]")
    ax_d.grid(alpha=0.3)

    if title:
        fig.suptitle(title, fontsize=12)
    fig.tight_layout()

    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150)
    return fig, axes


def save_table(table: pd.DataFrame, path: str | Path) -> Path:
    """Write a metric table to CSV, creating the results directory if needed."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    table.to_csv(path)
    return path


def plot_tradeoff(
    table: pd.DataFrame,
    x: str,
    y: str,
    label_col: str | None = None,
    title: str = "",
    x_label: str | None = None,
    y_label: str | None = None,
    good_x: tuple[float, float] | None = None,
    path: str | Path | None = None,
    figsize: tuple[float, float] = (9.0, 6.5),
    annotate: bool = True,
):
    """Scatter one metric against another, one point per controller.

    The intended use is a performance-versus-robustness frontier: tracking
    error on one axis, maximum sensitivity Ms on the other. A tuning rule can
    only be judged on the pair. ``good_x`` shades the region considered
    acceptable (for Ms, roughly 1.2-1.6 in process practice).
    """
    fig, ax = plt.subplots(figsize=figsize)

    if good_x is not None:
        ax.axvspan(good_x[0], good_x[1], color="seagreen", alpha=0.10, zorder=0)
        # Axes-fraction y, so the label sits at the top of the band whatever
        # the data limits turn out to be once the points are drawn.
        ax.text(
            0.5 * (good_x[0] + good_x[1]), 0.98, "comfortable",
            transform=ax.get_xaxis_transform(), ha="center", va="top",
            fontsize=8, color="seagreen",
        )

    labels = table[label_col] if label_col else table.index
    ax.scatter(table[x], table[y], s=70, zorder=3, edgecolor="k", linewidth=0.6)
    if annotate:
        for xi, yi, li in zip(table[x], table[y], labels):
            ax.annotate(
                str(li), (xi, yi), textcoords="offset points", xytext=(7, 4), fontsize=8.5
            )

    ax.set_xlabel(x_label or x)
    ax.set_ylabel(y_label or y)
    ax.grid(alpha=0.3)
    if title:
        ax.set_title(title, fontsize=12)
    fig.tight_layout()

    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150)
    return fig, ax


def plot_sweep(
    table: pd.DataFrame,
    x: str,
    panels: list[dict],
    title: str = "",
    x_label: str | None = None,
    logx: bool = False,
    path: str | Path | None = None,
    figsize: tuple[float, float] | None = None,
):
    """Stacked line plots of several metrics against one swept parameter.

    ``panels`` is a list of ``{"series": [col, ...], "ylabel": str, "logy": bool}``.
    Used for parameter sweeps (dead-time ratio, model mismatch), where the shape
    of the degradation curve is the result and a single operating point is not.
    """
    n = len(panels)
    fig, axes = plt.subplots(
        n, 1, figsize=figsize or (9.0, 2.6 * n + 1.0), sharex=True, squeeze=False
    )
    axes = axes[:, 0]

    for ax, panel in zip(axes, panels):
        for col in panel["series"]:
            ax.plot(table[x], table[col], marker="o", ms=4.5, lw=1.6, label=col)
        ax.set_ylabel(panel.get("ylabel", ""))
        ax.grid(alpha=0.3)
        if panel.get("logy"):
            ax.set_yscale("log")
        if len(panel["series"]) > 1:
            ax.legend(fontsize=8.5)
        for line in panel.get("hlines", []):
            ax.axhline(line, color="crimson", ls=":", lw=1.1)
        for band in panel.get("vspans", []):
            ax.axvspan(band[0], band[1], color="crimson", alpha=0.08, zorder=0)

    if logx:
        axes[-1].set_xscale("log")
    axes[-1].set_xlabel(x_label or x)
    if title:
        fig.suptitle(title, fontsize=12)
    fig.tight_layout()

    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150)
    return fig, axes

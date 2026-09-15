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
    diagnostic: str | None = None,
    diagnostic_label: str | None = None,
):
    """Overlay several controllers on the standard three-panel layout.

    ``diagnostic`` names a signal published by the controllers via
    :meth:`Controller.diagnostics` -- ``"inner_setpoint"``,
    ``"u_feedforward"``, ``"duty"``. Naming one adds a panel for it, which is
    how a cascade's inner setpoint or a feedforward's contribution becomes
    visible rather than being buried inside the controller.
    """
    if diagnostic is not None:
        fig, axes = plt.subplots(
            4, 1, figsize=(figsize[0], figsize[1] * 1.2), sharex=True,
            gridspec_kw={"height_ratios": [3, 2, 1.5, 1]},
        )
        ax_y, ax_u, ax_diag, ax_d = axes
    else:
        fig, axes = plt.subplots(
            3, 1, figsize=figsize, sharex=True, gridspec_kw={"height_ratios": [3, 2, 1]}
        )
        ax_y, ax_u, ax_d = axes
        ax_diag = None

    first = next(iter(runs.values()))
    t = first["t"].to_numpy(float)
    ax_y.plot(t, signal(first, "sp")[:, 0], "k--", lw=1.4, label="setpoint", zorder=1)

    for label, df in runs.items():
        tt = df["t"].to_numpy(float)
        ax_y.plot(tt, signal(df, "y")[:, 0], lw=1.6, label=label)
        ax_u.step(tt, signal(df, "u")[:, 0], where="post", lw=1.4, label=label)
        if ax_diag is not None:
            col = f"diag_{diagnostic}"
            if col in df.columns:
                ax_diag.step(tt, df[col].to_numpy(float), where="post", lw=1.4, label=label)

    # Declared output band (reporting-only in phase 1) and actuator limits.
    for lim, name in ((first.attrs.get("y_min"), "y_min"), (first.attrs.get("y_max"), "y_max")):
        if lim is not None:
            ax_y.axhline(lim[0], color="crimson", ls=":", lw=1.2)
            ax_y.text(t[-1], lim[0], f" {name}", color="crimson", va="center", fontsize=8)
    for lim in (first.attrs.get("u_min"), first.attrs.get("u_max")):
        if lim is not None and np.isfinite(lim[0]):
            ax_u.axhline(lim[0], color="grey", ls=":", lw=1.2)

    # Every disturbance channel, not just the first: a plant can be upset in
    # more than one place, and a panel that silently shows only channel 0 makes
    # the others look like they never happened.
    d_all = signal(first, "d")
    for j in range(d_all.shape[1]):
        ax_d.step(
            t, d_all[:, j], where="post", lw=1.4,
            label=f"d{j}" if d_all.shape[1] > 1 else None,
        )
    if d_all.shape[1] > 1:
        ax_d.legend(fontsize=8, ncol=d_all.shape[1])

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
    if ax_diag is not None:
        ax_diag.set_ylabel(diagnostic_label or diagnostic.replace("_", " "))
        ax_diag.grid(alpha=0.3)
        ax_diag.legend(fontsize=8)
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


def plot_horizon(
    df: pd.DataFrame,
    key: str = "y_pred",
    u_key: str | None = "u_plan",
    title: str = "",
    y_label: str = "level y [%]",
    u_label: str = "valve u [%]",
    path: str | Path | None = None,
    figsize: tuple[float, float] = (11.0, 7.0),
    max_steps: int | None = None,
):
    """The receding-horizon fan: what the controller thought would happen.

    Each stored snapshot is drawn forward from the sample it was made at, over
    the trace of what actually happened. Where the fan hugs the trace the
    internal model is good; where it peels away it is not, and the distance is
    the model mismatch the controller was working with at that moment.

    Needs a run made with ``simulate(..., snapshot_stride=n)`` -- the snapshots
    live in ``df.attrs``, not in the columns, because a predicted trajectory
    does not fit one row per sample.

    ``max_steps`` draws only the first that-many samples of each prediction.
    A prediction horizon long enough to cover the settling time (which is what
    `LinearMPC` wants, having no terminal cost) is far longer than the part
    anyone can read: every curve ends up flat on the setpoint and the fan
    becomes a smear. Windowing the *drawing* changes nothing about the
    controller.
    """
    snaps = df.attrs.get("snapshots") or []
    if not snaps:
        raise ValueError(
            "this run carries no snapshots: pass snapshot_stride=n to simulate(), "
            "and check the controller implements Controller.snapshot()"
        )

    show_u = u_key is not None and u_key in snaps[0]
    if show_u:
        fig, axes = plt.subplots(
            2, 1, figsize=figsize, sharex=True, gridspec_kw={"height_ratios": [3, 2]}
        )
        ax_y, ax_u = axes
    else:
        fig, ax_y = plt.subplots(figsize=(figsize[0], figsize[1] * 0.7))
        axes, ax_u = (ax_y,), None

    t = df["t"].to_numpy(float)
    ax_y.plot(t, signal(df, "sp")[:, 0], "k--", lw=1.4, label="setpoint", zorder=1)
    ax_y.plot(t, signal(df, "y")[:, 0], lw=1.8, color="k", label="what happened", zorder=3)
    if ax_u is not None:
        ax_u.step(t, signal(df, "u")[:, 0], where="post", lw=1.6, color="k", zorder=3)

    dt = float(df.attrs.get("dt", 1.0))
    for i, snap in enumerate(snaps):
        pred = np.atleast_2d(np.asarray(snap[key], dtype=float))
        if pred.shape[0] == 1 and pred.shape[1] > 1:
            pred = pred.T
        if max_steps is not None:
            pred = pred[:max_steps]
        n = pred.shape[0]
        # Snapshots may carry their own horizon clock; fall back to the sample
        # time so a controller that publishes only the trajectory still plots.
        th = (
            np.asarray(snap["t_horizon"], float)[:n]
            if "t_horizon" in snap
            else snap["t"] + np.arange(1, n + 1) * dt
        )
        label = "predicted" if i == 0 else None
        ax_y.plot(th, pred[:, 0], lw=1.0, alpha=0.75, color="tab:orange", label=label)
        ax_y.plot(snap["t"], pred[0, 0], ".", ms=4, color="tab:orange")
        if ax_u is not None:
            plan = np.atleast_2d(np.asarray(snap[u_key], dtype=float))
            if plan.shape[0] == 1 and plan.shape[1] > 1:
                plan = plan.T
            if max_steps is not None:
                plan = plan[:max_steps]
            ax_u.step(
                th[: plan.shape[0]], plan[:, 0], where="post",
                lw=1.0, alpha=0.75, color="tab:orange",
            )

    for lim, name in ((df.attrs.get("y_min"), "y_min"), (df.attrs.get("y_max"), "y_max")):
        if lim is not None:
            ax_y.axhline(lim[0], color="crimson", ls=":", lw=1.2)
            ax_y.text(t[-1], lim[0], f" {name}", color="crimson", va="center", fontsize=8)
    if ax_u is not None:
        for lim in (df.attrs.get("u_min"), df.attrs.get("u_max")):
            if lim is not None and np.isfinite(lim[0]):
                ax_u.axhline(lim[0], color="grey", ls=":", lw=1.2)

    ax_y.set_ylabel(y_label)
    ax_y.legend(loc="best", fontsize=9)
    ax_y.grid(alpha=0.3)
    ax_y.set_xlim(t[0], t[-1])
    if ax_u is not None:
        ax_u.set_ylabel(u_label)
        ax_u.grid(alpha=0.3)
    axes[-1].set_xlabel("time [s]")

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

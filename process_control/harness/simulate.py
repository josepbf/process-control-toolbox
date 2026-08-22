"""Closed-loop simulation runner.

One loop, used by every controller and every plant in the project. Keeping this
single-sourced is what makes the comparisons trustworthy: nobody gets a
different integrator, a different sample time or a different noise stream.

Timing convention per sample k (t = k*dt):

    1. the controller sees the measurement y(t) taken at the top of the sample
    2. it returns u(t), which is held constant over [t, t+dt)   (zero-order hold)
    3. the plant integrates one dt under u(t) and the disturbance d(t)
    4. a new noisy measurement y(t+dt) is taken

The logged row for time t therefore holds the measurement the controller acted
on and the move it made in response -- the causal pairing you want when reading
the plots.
"""

from __future__ import annotations

import copy
from time import perf_counter

import numpy as np
import pandas as pd

from ..controllers.base import Controller
from ..plants.base import Plant
from .scenarios import Scenario


def _row(prefix: str, values: np.ndarray) -> dict:
    values = np.atleast_1d(values)
    if values.size == 1:
        return {prefix: float(values[0])}
    return {f"{prefix}{i}": float(v) for i, v in enumerate(values)}


def _violation(y: np.ndarray, y_min, y_max) -> tuple[float, bool]:
    """Signed magnitude by which the output is outside its declared band."""
    mag = 0.0
    if y_min is not None:
        mag += float(np.sum(np.maximum(y_min - y, 0.0)))
    if y_max is not None:
        mag += float(np.sum(np.maximum(y - y_max, 0.0)))
    return mag, mag > 0.0


def simulate(
    plant: Plant,
    controller: Controller,
    scenario: Scenario,
    seed: int | None = None,
    copy_plant: bool = True,
) -> pd.DataFrame:
    """Run one controller against one plant for one scenario.

    Returns a tidy DataFrame with one row per sample and these columns:
    ``t, y, sp, u, d, violation, violated, solve_time``  (vector signals get an
    index suffix, e.g. ``y0``, ``y1``). Controllers that implement
    :meth:`Controller.diagnostics` add one ``diag_<name>`` column each. Run
    metadata lands in ``df.attrs``.
    """
    if copy_plant:
        plant = copy.deepcopy(plant)

    dt = scenario.dt
    seed = scenario.seed if seed is None else seed

    y = plant.reset(seed=seed)
    controller.reset()

    # Feedforward controllers read a measured disturbance. Its measurement
    # noise gets its own RNG stream, so switching feedforward on or off cannot
    # change the measurement-noise realisation seen on y.
    uses_d = getattr(controller, "uses_measured_disturbance", False)
    rng_d = np.random.default_rng(seed + 104729)

    rows = []
    u = np.zeros(plant.n_inputs)
    diag: dict[str, float] = {}
    for k in range(scenario.n_steps):
        t = k * dt
        sp = np.atleast_1d(scenario.setpoint(t))
        d = np.atleast_1d(scenario.disturbance(t))

        d_meas = d.copy()
        if scenario.d_noise_std > 0:
            d_meas = d_meas + rng_d.normal(0.0, scenario.d_noise_std, size=d.shape)

        t0 = perf_counter()
        if uses_d:
            u = controller.compute(y, sp, t, d=d_meas)
        else:
            u = controller.compute(y, sp, t)
        u = np.atleast_1d(np.asarray(u, dtype=float))
        solve_time = perf_counter() - t0

        # Internal signals the controller chooses to expose. Namespaced so they
        # can never collide with the y/u/d/sp columns the metrics select on.
        diag = {f"diag_{k}": float(v) for k, v in controller.diagnostics().items()}

        mag, flag = _violation(y, plant.y_min, plant.y_max)
        rows.append(
            {
                "t": t,
                **_row("y", y),
                **_row("sp", sp),
                **_row("u", u),
                **_row("d", d),
                **_row("d_meas", d_meas),
                **diag,
                "violation": mag,
                "violated": flag,
                "solve_time": solve_time,
            }
        )
        y = plant.step(u, dt, d)

    # Final measurement, so the trace ends where the process actually ended.
    t = scenario.n_steps * dt
    mag, flag = _violation(y, plant.y_min, plant.y_max)
    rows.append(
        {
            "t": t,
            **_row("y", y),
            **_row("sp", np.atleast_1d(scenario.setpoint(t))),
            **_row("u", u),
            **_row("d", np.atleast_1d(scenario.disturbance(t))),
            **_row("d_meas", np.atleast_1d(scenario.disturbance(t))),
            # No controller call on the final row, so the last values stand.
            **diag,
            "violation": mag,
            "violated": flag,
            "solve_time": np.nan,
        }
    )

    df = pd.DataFrame(rows)
    df.attrs.update(
        {
            "controller": controller.name,
            "tuning": controller.tuning_note,
            "controller_info": controller.describe(),
            "plant": repr(plant),
            "scenario": scenario.name,
            "dt": dt,
            "seed": seed,
            "uses_measured_disturbance": uses_d,
            "u_min": plant.u_min.tolist(),
            "u_max": plant.u_max.tolist(),
            "y_min": None if plant.y_min is None else plant.y_min.tolist(),
            "y_max": None if plant.y_max is None else plant.y_max.tolist(),
        }
    )
    return df


def run_all(
    plant: Plant,
    controllers: dict[str, Controller],
    scenario: Scenario,
    seed: int | None = None,
) -> dict[str, pd.DataFrame]:
    """Run several controllers against an identical plant/scenario/seed."""
    return {
        label: simulate(plant, ctrl, scenario, seed=seed)
        for label, ctrl in controllers.items()
    }

"""Tutorial 2: from a step test to a tuned, robustness-checked PI."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from process_control.plants.tank import Tank
from process_control.tuning.analysis import pid_on_fopdt
from process_control.tuning.rules import (
    amigo_pi,
    lambda_tuning,
    simc_pi,
    ziegler_nichols_open_loop,
)


def open_loop_step(plant, u_before, u_after, t_step=50.0, dt=1.0, t_final=600.0):
    """Hold the valve steady, step it, and record the reaction curve."""
    plant.reset()
    t, y = [0.0], [float(plant.measure()[0])]
    for k in range(int(t_final / dt)):
        u = u_before if k * dt < t_step else u_after
        y.append(float(plant.step([u], dt)[0]))
        t.append((k + 1) * dt)
    return np.array(t), np.array(y)


def fit_fopdt(t, y, u_step, t_step):
    """Two-point reaction-curve fit at 35.3 % and 85.3 % of the final change."""
    y0 = float(np.mean(y[(t > t_step - 20.0) & (t <= t_step)]))
    y_inf = float(np.mean(y[t >= 0.9 * t[-1]]))
    K = (y_inf - y0) / u_step

    def time_at(frac):
        target = y0 + frac * (y_inf - y0)
        i = int(np.argmax(np.sign(y_inf - y0) * (y - target) >= 0))
        return float(np.interp(target, [y[i - 1], y[i]], [t[i - 1], t[i]]))

    t1, t2 = time_at(0.353), time_at(0.853)
    return {"K": K, "tau": 0.67 * (t2 - t1), "theta": 1.3 * t1 - 0.29 * t2 - t_step}


plant = Tank(K=1.5, tau=60.0, theta=15.0, h0=30.0, noise_std=0.15, seed=7)
t, y = open_loop_step(plant, u_before=20.0, u_after=30.0)
model = fit_fopdt(t, y, u_step=10.0, t_step=50.0)

print("identified:", {k: round(float(v), 2) for k, v in model.items()})
print("true      :", plant.fopdt)
print()

rules = {
    "SIMC": simc_pi(**model),
    "AMIGO": amigo_pi(**model),
    "Lambda": lambda_tuning(**model),
    "ZN open loop": ziegler_nichols_open_loop(**model),
}
for label, tuning in rules.items():
    on_model = pid_on_fopdt(**model, Kc=tuning.Kc, Ti=tuning.Ti)
    on_truth = pid_on_fopdt(**plant.fopdt, Kc=tuning.Kc, Ti=tuning.Ti)
    print(
        f"{label:14s} Kc={tuning.Kc:5.2f} Ti={tuning.Ti:6.1f}  "
        f"Ms(identified)={on_model['Ms']:.2f}  Ms(true plant)={on_truth['Ms']:.2f}"
    )

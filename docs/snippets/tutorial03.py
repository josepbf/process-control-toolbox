"""Tutorial 3: relay auto-tuning, and what the hysteresis is for."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from process_control.harness.plotting import plot_runs
from process_control.plants.tank import Tank
from process_control.tuning.analysis import pid_on_fopdt, ultimate_gain_period
from process_control.tuning.relay import relay_autotune
from process_control.tuning.rules import simc_pi, tyreus_luyben, ziegler_nichols_closed_loop

TRUE = {"K": 1.5, "tau": 60.0, "theta": 15.0}
Ku_true, Pu_true = ultimate_gain_period(**TRUE)
print(f"analytic truth: Ku = {Ku_true:.3f}, Pu = {Pu_true:.1f} s\n")

# --- the experiment, and what the two knobs do -------------------------
for noise, hyst in [(0.0, 0.0), (0.0, 0.5), (0.15, 0.5), (0.15, 0.0)]:
    plant = Tank(**TRUE, h0=50.0, noise_std=noise, seed=7)
    r = relay_autotune(
        plant, setpoint=50.0, u_bias=plant.steady_input(50.0),
        h=10.0, hysteresis=hyst, dt=1.0, t_final=900.0, seed=7,
    )
    print(
        f"noise={noise:<5} hysteresis={hyst:<4} "
        f"Ku={r.Ku:6.3f} ({100 * (r.Ku - Ku_true) / Ku_true:+6.1f} %)  "
        f"Pu={r.Pu:5.1f} ({100 * (r.Pu - Pu_true) / Pu_true:+5.1f} %)  "
        f"cycles={r.n_cycles:3d}"
    )

# --- the tuning that follows from it -----------------------------------
plant = Tank(**TRUE, h0=50.0, noise_std=0.15, seed=7)
relay = relay_autotune(
    plant, setpoint=50.0, u_bias=plant.steady_input(50.0),
    h=10.0, hysteresis=0.5, dt=1.0, t_final=900.0, seed=7,
)
print(f"\n{relay.summary()}\n")

rules = {
    "ZN-CL (from relay)": ziegler_nichols_closed_loop(relay.Ku, relay.Pu, kind="PI"),
    "Tyreus-Luyben (from relay)": tyreus_luyben(relay.Ku, relay.Pu, kind="PI"),
    "SIMC (from the true model)": simc_pi(**TRUE),
}
for label, t in rules.items():
    m = pid_on_fopdt(**TRUE, Kc=t.Kc, Ti=t.Ti)
    print(f"{label:28s} Kc={t.Kc:5.2f}  Ti={t.Ti:6.1f}  Ms={m['Ms']:.2f}  GM={m['GM']:.2f}")

plot_runs({"relay experiment": relay.df}, title="Relay experiment",
          path="results/tutorial03.png")

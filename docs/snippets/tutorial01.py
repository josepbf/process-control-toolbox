"""Tutorial 1: a first closed loop, end to end."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from src.controllers.onoff import OnOffController
from src.controllers.pid import PIDController
from src.harness.metrics import format_table, summarize
from src.harness.plotting import plot_runs
from src.harness.scenarios import setpoint_and_load
from src.harness.simulate import run_all
from src.plants.tank import Tank
from src.tuning.rules import simc_pi

plant = Tank(
    K=1.5, tau=60.0, theta=15.0, Kd=1.0, h0=30.0,
    u_min=0.0, u_max=100.0, noise_std=0.15, seed=7,
)
scenario = setpoint_and_load(
    dt=1.0, y_start=30.0, y_step=50.0, t_step=100.0,
    d_load=-12.0, t_load=700.0, t_final=1400.0, seed=7,
)

tuning = simc_pi(**plant.fopdt)
controllers = {
    "ON/OFF": OnOffController(u_on=100.0, u_off=0.0, hysteresis=1.0),
    "PI (SIMC)": PIDController(
        **tuning.as_kwargs(), dt=scenario.dt,
        u_min=plant.u_min[0], u_max=plant.u_max[0],
        u0=plant.steady_input(30.0),
        name="PI (SIMC)", tuning_note=tuning.rule,
    ),
}

runs = run_all(plant, controllers, scenario)
print(format_table(summarize(runs, windows=scenario.windows)))
plot_runs(runs, title="Tutorial 1", scenario=scenario, path="results/tutorial01.png")

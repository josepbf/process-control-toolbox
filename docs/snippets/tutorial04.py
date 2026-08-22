"""Tutorial 4: a custom plant — steam-jacketed stirred vessel."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import numpy as np

from process_control.controllers.pid import PIDController
from process_control.harness.metrics import format_table, summarize
from process_control.harness.plotting import plot_runs
from process_control.harness.scenarios import Scenario, staircase
from process_control.harness.simulate import simulate
from process_control.plants.base import Plant
from process_control.tuning.analysis import pid_on_fopdt
from process_control.tuning.rules import half_rule, simc_pi


class JacketedVessel(Plant):
    """Steam-jacketed stirred vessel: jacket temperature drives the contents.

        tau_j * dT_j/dt = -T_j + K_j * u(t - theta)
        tau_c * dT_c/dt = -T_c + T_j + Kd * d(t - theta_d)
        y = T_c

    Two lags in series with a transport delay on the steam line. The jacket is
    fast and the contents are slow, which is the usual arrangement -- and the
    reason this process is a natural candidate for cascade control if the
    jacket temperature is instrumented.
    """

    def __init__(
        self,
        K_j: float = 0.8,        # degC jacket per % steam valve
        tau_j: float = 25.0,     # jacket lag [s]
        tau_c: float = 180.0,    # contents lag [s]
        theta: float = 8.0,      # steam line transport delay [s]
        Kd: float = 1.0,         # ambient / feed disturbance gain
        theta_d: float = 0.0,
        T0: float = 40.0,
        u_min: float = 0.0,
        u_max: float = 100.0,
        noise_std: float = 0.0,
        y_min: float | None = None,
        y_max: float | None = None,
        seed: int = 0,
    ):
        self.K_j, self.tau_j, self.tau_c = float(K_j), float(tau_j), float(tau_c)
        self.theta, self.Kd = float(theta), float(Kd)

        super().__init__(
            n_states=2, n_inputs=1, n_outputs=1,
            u_min=u_min, u_max=u_max,
            x0=[T0, T0], dead_time=theta, d_dead_time=theta_d,
            noise_std=noise_std, y_min=y_min, y_max=y_max, seed=seed,
        )
        # Prime the delay line with the valve position that holds T0, so the
        # run does not start with a hidden step buried in the pipeline.
        self.u_init = self.saturate(np.array([T0 / self.K_j]))

    def dynamics(self, x, u, d):
        T_j, T_c = x
        return np.array([
            (-T_j + self.K_j * u[0]) / self.tau_j,
            (-T_c + T_j + self.Kd * d[0]) / self.tau_c,
        ])

    def output(self, x):
        return np.array([x[1]])          # only the contents are measured

    def steady_input(self, y_target: float) -> float:
        return y_target / self.K_j

    @property
    def fopdt(self) -> dict:
        """What a two-parameter tuning rule is actually given, via the half rule."""
        tau_eff, theta_eff = half_rule([self.tau_c, self.tau_j], theta=self.theta)
        return {"K": self.K_j, "tau": tau_eff, "theta": theta_eff}


if __name__ == "__main__":
    vessel = JacketedVessel(noise_std=0.1, seed=7)
    print("half-rule model:", {k: round(v, 2) for k, v in vessel.fopdt.items()})

    tuning = simc_pi(**vessel.fopdt)
    print(f"{tuning.rule}: Kc={tuning.Kc:.3f}, Ti={tuning.Ti:.1f}")
    print(f"Ms = {pid_on_fopdt(**vessel.fopdt, Kc=tuning.Kc, Ti=tuning.Ti)['Ms']:.3f}\n")

    scenario = Scenario(
        name="heat_up_then_load",
        dt=1.0,
        t_final=2400.0,
        setpoint=staircase([(0.0, 40.0), (200.0, 60.0)]),
        disturbance=staircase([(0.0, 0.0), (1400.0, -8.0)]),
        seed=7,
        description="contents 40 -> 60 degC at t=200 s; heat-loss upset at t=1400 s",
        windows={"setpoint": (200.0, 1400.0), "disturbance": (1400.0, 2400.0)},
    )
    pi = PIDController(
        **tuning.as_kwargs(), dt=scenario.dt,
        u_min=vessel.u_min[0], u_max=vessel.u_max[0],
        u0=vessel.steady_input(40.0),
        name="PI (SIMC)", tuning_note=tuning.rule,
    )

    df = simulate(vessel, pi, scenario)
    print(format_table(summarize({"PI (SIMC)": df}, windows=scenario.windows)))
    plot_runs(
        {"PI (SIMC)": df}, title="Jacketed vessel", scenario=scenario,
        y_label="contents T [degC]", u_label="steam valve u [%]",
        path="results/tutorial04.png",
    )

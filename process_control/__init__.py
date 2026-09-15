"""A toolbox for classical process control.

Plants, controllers, tuning rules, frequency-domain robustness analysis and a
shared simulation harness, written to be read: explicit implementations in
preference to clever ones, with the control concept behind each one explained
where it appears.

The names below are the common entry points, re-exported for convenience::

    from process_control import Tank, PIDController, simulate, simc_pi

Everything else is reachable by its submodule path, e.g.::

    from process_control.controllers.cascade import CascadeController
    from process_control.tuning.relay import relay_autotune

Submodules
----------
``plants``       ground-truth process simulators; own all saturation, dead time
                 and measurement noise
``models``       the controller's *belief* about the process: discretisation,
                 dead-time and disturbance augmentation, state estimation
``controllers``  control laws, interchangeable inside the harness
``tuning``       named and cited tuning rules, robustness analysis, relay
                 auto-tuning
``harness``      the closed-loop runner, scenarios, metrics and plotting
``solvers``      the single point where numerics are delegated: a QP backend,
                 dependency-free by default
"""

from .controllers.base import Controller
from .controllers.cascade import CascadeController
from .controllers.feedforward import FeedforwardPID
from .controllers.mpc import LinearMPC
from .controllers.onoff import OnOffController, TimeProportioningController
from .controllers.pid import PIDController, VelocityPIDController
from .controllers.smith import SmithPredictor
from .harness.metrics import compute_metrics, format_table, summarize
from .harness.plotting import (
    plot_horizon,
    plot_runs,
    plot_sweep,
    plot_tradeoff,
    save_table,
)
from .harness.scenarios import Scenario, constant, pulse, setpoint_and_load, staircase
from .harness.simulate import run_all, simulate
from .models.discrete import DiscreteModel, fopdt_model, integrating_model, series_lags_model
from .models.observer import KalmanObserver, augment_disturbance
from .plants.base import Plant
from .plants.cascade_process import CascadeProcess
from .plants.integrating_tank import IntegratingTank
from .plants.inverse_response import InverseResponseTank
from .plants.series_tanks import SeriesTanks
from .plants.tank import Tank
from .solvers import available_backends, solve_qp
from .tuning.analysis import pid_on_fopdt, pid_on_integrator, ultimate_gain_period
from .tuning.mpc_rules import MPCTuning, settling_horizon, shridhar_cooper
from .tuning.relay import relay_autotune
from .tuning.rules import (
    PIDTuning,
    amigo_pi,
    amigo_pid,
    averaging_level_pi,
    cohen_coon,
    half_rule,
    imc_pi,
    imc_pid,
    lambda_tuning,
    simc_integrating,
    simc_pi,
    simc_pid,
    tyreus_luyben,
    ziegler_nichols_closed_loop,
    ziegler_nichols_open_loop,
)

__version__ = "0.1.0"

__all__ = [
    # plants
    "Plant", "Tank", "IntegratingTank", "SeriesTanks", "InverseResponseTank",
    "CascadeProcess",
    # models -- what the controller believes, as opposed to what is true
    "DiscreteModel", "fopdt_model", "integrating_model", "series_lags_model",
    "augment_disturbance", "KalmanObserver",
    # controllers
    "Controller", "OnOffController", "TimeProportioningController",
    "PIDController", "VelocityPIDController", "CascadeController", "FeedforwardPID",
    "SmithPredictor", "LinearMPC",
    # harness
    "simulate", "run_all", "Scenario", "constant", "staircase", "pulse",
    "setpoint_and_load", "compute_metrics", "summarize", "format_table",
    "plot_runs", "plot_tradeoff", "plot_sweep", "plot_horizon", "save_table",
    # tuning
    "PIDTuning", "simc_pi", "simc_pid", "imc_pi", "imc_pid", "lambda_tuning",
    "cohen_coon", "amigo_pi", "amigo_pid", "tyreus_luyben",
    "ziegler_nichols_open_loop", "ziegler_nichols_closed_loop",
    "simc_integrating", "averaging_level_pi", "half_rule",
    "pid_on_fopdt", "pid_on_integrator", "ultimate_gain_period", "relay_autotune",
    "MPCTuning", "shridhar_cooper", "settling_horizon",
    # the one delegation boundary
    "solve_qp", "available_backends",
]

"""What the controller believes about the process.

A *model* in this package is never the process. `plants/` is the truth; this is
what the controller thinks, built from the two or three numbers a tuning rule
uses, discretised at the controller's sample time, and under no obligation to be
right. The gap between the two is where the interesting results in this project
live -- fairness rule 3 exists precisely to stop a model-based controller being
handed the plant it is supposed to be guessing at.

Everything here is linear, discrete-time and dense:

    x[k+1] = A x[k] + B u[k] (+ Bd d[k]),     y[k] = C x[k]

because that is the form a Smith predictor, an observer and a linear MPC all
need, and because `scipy.linalg.expm` and `solve_discrete_are` were already
dependencies. No solver arrives with this package.
"""

from .discrete import (
    DiscreteModel,
    append_disturbance_path,
    augment_dead_time,
    fopdt_model,
    integrating_model,
    series_lags_model,
    zoh,
)
from .observer import KalmanObserver, augment_disturbance

__all__ = [
    "DiscreteModel",
    "zoh",
    "fopdt_model",
    "integrating_model",
    "series_lags_model",
    "augment_dead_time",
    "append_disturbance_path",
    "augment_disturbance",
    "KalmanObserver",
]

# Models

What the controller *believes* about the process — never the process itself.
[`plants`](plants.md) is the truth; this is the guess, built from the two or
three numbers a tuning rule uses, discretised at the controller's sample time,
and under no obligation to be right. The gap between the two is where the
interesting results live, and [fairness rule 3](../concepts/fairness.md) exists
to stop a model-based controller quietly being handed the plant it is supposed
to be guessing at.

Everything here is linear, discrete-time and dense:

$$
x[k+1] = A\,x[k] + B\,u[k] \;(+\; B_d\,d[k]), \qquad y[k] = C\,x[k]
$$

No solver arrives with this package — `scipy.linalg.expm` and
`solve_discrete_are` were already dependencies.

| what | why it is here |
|---|---|
| `DiscreteModel` | the one form a Smith predictor, an observer and a linear MPC all need |
| `fopdt_model` and friends | built to be called like a tuning rule: `fopdt_model(**plant.fopdt, dt=dt)` |
| `augment_dead_time` | dead time as states, rounded the way the plant rounds, and **reported** |
| `augment_disturbance` | the integrating state that *is* a model-based law's integral action |
| `KalmanObserver` | `compute()` receives `y`, not `x`, so somebody has to supply the estimate |

---

## DiscreteModel

::: process_control.models.discrete.DiscreteModel

---

## Discretisation

::: process_control.models.discrete.zoh

---

## Builders

::: process_control.models.discrete.fopdt_model

::: process_control.models.discrete.integrating_model

::: process_control.models.discrete.series_lags_model

::: process_control.models.discrete.augment_dead_time

::: process_control.models.discrete.append_disturbance_path

---

## Offset-free estimation

::: process_control.models.observer.augment_disturbance

::: process_control.models.observer.KalmanObserver

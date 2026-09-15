# Architecture

How the pieces fit, and — more importantly — the contract for adding a
controller that needs more than the current measurement.

## Two abstractions, one rule

```python
class Plant(ABC):
    """Ground truth. The controller never sees inside it."""
    def step(self, u, dt, d=None) -> np.ndarray: ...   # saturate → delay → integrate → measure
    def reset(self, x0=None, seed=None) -> np.ndarray: ...

class Controller(ABC):
    def compute(self, y, setpoint, t) -> np.ndarray: ...
    def reset(self) -> None: ...
```

The rule that makes everything else work: **the plant owns every physical
limit** — actuator saturation, all three dead-time paths, and measurement
noise. A controller cannot exempt itself from a constraint, because it never
touches the mechanism that enforces one. A controller may hold its own
*internal model*, and that model is not required to match the plant.

`simulate()` in `harness/simulate.py` is the only closed-loop runner. Nothing
gets its own integrator, its own sample time or its own noise stream.

## The capability-flag pattern

Some control structures need information beyond `(y, setpoint, t)`. The
established pattern is a **default-off class flag** that the controller
declares and the harness honours:

```python
class FeedforwardPID(Controller):
    uses_measured_disturbance = True     # harness now passes d= to compute()
```

Declared in `controllers/base.py`, honoured in `simulate.py`. Two properties
make this the right shape:

- **It is additive.** Existing controllers inherit `False` and are untouched.
- **It is a declaration, not a convenience.** Setting it asserts that the
  disturbance is instrumented on the real plant. It is why feedforward cannot
  quietly enjoy information the controllers it is compared against lack — the
  asymmetry is in the class definition, in the run metadata, and in the
  results table.

Any future capability follows the same shape.

## The diagnostics channel

Controllers compute quantities that are not their output but explain it. Those
are published as scalars and logged under a `diag_` prefix:

```python
def diagnostics(self) -> dict[str, float]:
    return {"inner_setpoint": float(self.inner_setpoint)}
```

Current publishers: `CascadeController` (`inner_setpoint` — the outer loop's
real output), `FeedforwardPID` (`u_feedforward` — the anticipated share of the
move), `TimeProportioningController` (`duty` — the continuous demand behind a
pulsed output). `plot_runs(..., diagnostic="inner_setpoint")` adds a panel.

The `diag_` prefix is not decoration: `metrics._cols()` selects signals by
`y`/`u`/`d`/`sp` prefix, and namespacing keeps a diagnostic from ever being
mistaken for a process signal.

## The second capability flag: `uses_preview`

Some control laws read *future* setpoints. Declaring `uses_preview = True` and a
`preview_horizon` makes the harness pass `sp_preview=` to `compute()`: an array
whose row `j` is the setpoint at `t + j*dt`, so row 0 is the setpoint the
controller would have seen anyway.

It is justified by classical needs on its own — setpoint ramping and kiln
heat-up profiles are preview problems. And it carries the same obligation as
`uses_measured_disturbance`: preview is an information advantage, so a
controller using it must be compared against a classical scheme given the
equivalent (a ramped setpoint, or setpoint feedforward), or the asymmetry must
be stated. It is recorded in `df.attrs["uses_preview"]` for exactly that
reason.

The harness assembles the preview **before** starting the solve-time clock.
Building it is the harness's work, not the controller's, and `solve_ms_mean` is
a reported metric.

## The snapshot channel

`diagnostics()` is scalars only, because the log is one row per sample. A
receding-horizon controller also computes something that log cannot hold: the
whole predicted output trajectory and the whole planned move sequence.

```python
def snapshot(self) -> dict[str, np.ndarray] | None:
    return {"y_pred": ..., "u_plan": ...}
```

`simulate(..., snapshot_stride=n)` calls it every `n` samples and collects the
results in `df.attrs["snapshots"]`, leaving the DataFrame scalar and the metrics
untouched. Off by default. `plot_horizon()` draws the result: each prediction
fanned forward from the sample it was made at, over the trace of what actually
happened.

Snapshots are diagnostics, not results. `df.attrs` does not survive
`pd.concat` and does not reach a CSV, so nothing in `metrics` may depend on
them.

## What a model-based controller will need

Two utilities do not exist yet:

- **A discrete internal model** — FOPDT → state space, discretisation via
  `scipy.linalg.expm`, and dead-time augmentation (θ = 15 s at dt = 1 s means
  15 extra states). Both `scipy.linalg.expm` and `solve_discrete_are` are
  already available; no new dependency.
- **A state observer** — `compute()` receives `y`, not `x`. Any model-based law
  needs an estimate. An integrating output-disturbance state is what supplies
  integral action; without one, a model-based controller has none and will
  show steady-state offset on a load change.

**The first thing that needs both is the Smith predictor**, which is next on
the roadmap on its own classical merit. They get built there.

## Does this enable MPC?

Yes, and the answer is structural rather than optimistic. MPC needs three
things beyond the original interface: a state estimate, optional setpoint
preview, and a way to publish predicted trajectories. **Two of the three are now
built** — `uses_preview` and `snapshot()` above — each additive, each following
a pattern already working in the codebase, and each justified by a classical
need of its own. The third, the observer, arrives with the Smith predictor.
`y_min`/`y_max` already exist on the plant as reporting-only bands with
violation metrics, so the constraint-handling comparison needs no new machinery
at all.

The one place MPC is allowed to reach outside the toolbox is the **numerical QP
solve**, behind a single `solve_qp` boundary with a dependency-free default.
Building the prediction matrices, the disturbance model and the constraint rows
is toolbox code, because that is what has to be read when a result looks too
good. Backend selection never probes for whatever happens to be pip installed —
a controller whose numerics depend on the machine is a controller whose results
are not comparable — and the backend that actually solved a run is recorded in
`df.attrs["controller_info"]["solver"]`.

MPC would arrive as one more `Controller` — not as a reason to reshape the
toolbox around it.

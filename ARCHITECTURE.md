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

## Extension points that are designed but not built

Deliberately **not** implemented, because nothing uses them yet and speculative
hooks are the same smell as an empty package. Recorded here so the cost of
adding them is known rather than guessed:

**`uses_preview`** — future setpoints over a horizon, following the flag
pattern exactly. Justified by classical needs on their own: setpoint ramping
and kiln heat-up profiles are preview problems. Note that preview is an
information advantage, so a controller using it must be compared against a
classical scheme given the equivalent (setpoint feedforward), or the asymmetry
must be stated.

**`snapshot()`** — vector-valued output per sample (a predicted trajectory),
which does not fit the tidy one-row-per-sample log. Intended design: the
harness captures it every `snapshot_stride` samples into `df.attrs`, leaving
the DataFrame scalar. Needed by anything that plots a receding horizon.

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
things beyond the current interface: a state estimate, optional setpoint
preview, and a way to publish predicted trajectories. Each is additive, each
follows a pattern already working in the codebase, and each is independently
required by a classical component ahead of MPC in the queue. `y_min`/`y_max`
already exist on the plant as reporting-only bands with violation metrics, so
the constraint-handling comparison needs no new machinery at all.

MPC would arrive as one more `Controller` — not as a reason to reshape the
toolbox around it.

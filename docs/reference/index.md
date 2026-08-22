# API reference

Generated from the source docstrings. The docstrings in this project carry the
*reasoning* as well as the signature — why the anti-windup uses
back-calculation, why the reversal metric has a threshold, why the half rule
sends a right-half-plane zero into the dead time — so this section is worth
reading rather than only searching.

| module | what is in it |
|---|---|
| [Plants](plants.md) | `Plant` ABC and the five concrete processes |
| [Controllers](controllers.md) | `Controller` ABC, ON/OFF, PWM, PID, velocity PID, cascade, feedforward |
| [Harness](harness.md) | `simulate`, `run_all`, scenarios, metrics, plotting |
| [Tuning](tuning.md) | thirteen rules, robustness analysis, relay auto-tuning |

## The two interfaces in full

```python
class Plant(ABC):
    """Ground truth. The controller never sees inside it."""
    def step(self, u, dt, d=None) -> np.ndarray: ...   # saturate -> delay -> integrate -> measure
    def reset(self, x0=None, seed=None) -> np.ndarray: ...

class Controller(ABC):
    def compute(self, y, setpoint, t) -> np.ndarray: ...
    def reset(self) -> None: ...
```

Everything in the project is one of those two, plus a `Scenario` describing
what the world does to the loop. See
[Architecture](../concepts/architecture.md).

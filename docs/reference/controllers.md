# Controllers

Any control law — ON/OFF, PID, LQR, MPC — implements the same interface, so
the harness can swap one for another without changing a line of simulation
code.

| class | form | notes |
|---|---|---|
| `OnOffController` | two-position with hysteresis | limit-cycles by construction |
| `TimeProportioningController` | PWM over any inner law | binary actuator, continuous effect |
| `PIDController` | positional / ISA ideal | back-calculation anti-windup, filtered derivative |
| `VelocityPIDController` | incremental | structural anti-windup, bumpless transfer |
| `CascadeController` | primary/secondary pair | the primary's output is the secondary's setpoint |
| `FeedforwardPID` | feedback + lead-lag | declares `uses_measured_disturbance` |
| `SmithPredictor` | PI around a dead-time-free internal model | the first controller carrying a model |
| `LinearMPC` | condensed QP in the move increments | constraints in the control law; declares `uses_preview` |

---

## Base class

::: process_control.controllers.base.Controller

---

## OnOffController

::: process_control.controllers.onoff.OnOffController

---

## TimeProportioningController

::: process_control.controllers.onoff.TimeProportioningController

---

## PIDController

::: process_control.controllers.pid.PIDController

---

## VelocityPIDController

::: process_control.controllers.pid.VelocityPIDController

---

## CascadeController

::: process_control.controllers.cascade.CascadeController

---

## FeedforwardPID

::: process_control.controllers.feedforward.FeedforwardPID

---

## LeadLag

::: process_control.controllers.feedforward.LeadLag

---

## SmithPredictor

::: process_control.controllers.smith.SmithPredictor

---

## LinearMPC

::: process_control.controllers.mpc.LinearMPC

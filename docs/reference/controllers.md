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
| `FeedforwardPID` | feedback + lead-lag | the only one declaring `uses_measured_disturbance` |

---

## Base class

::: src.controllers.base.Controller

---

## OnOffController

::: src.controllers.onoff.OnOffController

---

## TimeProportioningController

::: src.controllers.onoff.TimeProportioningController

---

## PIDController

::: src.controllers.pid.PIDController

---

## VelocityPIDController

::: src.controllers.pid.VelocityPIDController

---

## CascadeController

::: src.controllers.cascade.CascadeController

---

## FeedforwardPID

::: src.controllers.feedforward.FeedforwardPID

---

## LeadLag

::: src.controllers.feedforward.LeadLag

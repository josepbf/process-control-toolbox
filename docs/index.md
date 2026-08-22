# Process Control Toolbox

A toolbox for **classical process control** — plants, controllers, tuning
rules, frequency-domain robustness analysis and a shared simulation harness —
aimed at the features that actually make industrial control hard: **long dead
time, loop interaction, recycle, and hard actuator limits**.

Everything is written to be read: explicit implementations in preference to
clever ones, with the control concept behind each one explained where it
appears. Every component is exercised by a reproducible experiment that
demonstrates what it does and what it costs.

!!! info "Project status"
    Five plants, six controllers, thirteen tuning rules, relay auto-tuning,
    frequency-domain robustness analysis, and nine reproducible experiments
    (123 tests). Dead-time compensation is next — see the
    [roadmap](roadmap.md).

## What the toolbox is built on

Comparing control strategies is something the toolbox is *used for*, not what
it is for — and comparisons are only worth reading if the ground rules are
fixed in advance. Four properties are built in rather than promised.

<div class="grid cards" markdown>

-   :material-scale-balance:{ .lg .middle } __Every baseline is tuned by a named rule__

    ---

    Thirteen published tuning rules, each cited, each recorded in the results
    table alongside the run. No silent hand-tuning, ever.

    [:octicons-arrow-right-24: Tuning rules](concepts/tuning-rules.md)

-   :material-factory:{ .lg .middle } __All physical limits live in the plant__

    ---

    Saturation and transport delay belong to the process, not the controller.
    A control law cannot exempt itself from a constraint, only anticipate one.

    [:octicons-arrow-right-24: Architecture](concepts/architecture.md)

-   :material-chart-line:{ .lg .middle } __Tracking is reported with effort__

    ---

    Every comparison carries a tracking metric **and** a control-effort
    metric. Tracking bought with violent valve movement is not a win — in a
    real plant it destroys the actuator.

    [:octicons-arrow-right-24: Metrics](concepts/metrics.md)

-   :material-flask:{ .lg .middle } __Losses get reported too__

    ---

    Scenarios where the sophisticated method does *not* win are written up in
    the same detail as the ones where it does. Cascade is 7.7× better on one
    disturbance and 0.9× on another; both numbers are published.

    [:octicons-arrow-right-24: Fairness rules](concepts/fairness.md)

</div>

## Where to start

| If you want to… | Go to |
|---|---|
| install it and run something | [Getting started](getting-started.md) |
| learn the library by building a loop | [Tutorials](tutorials/index.md) |
| understand *why* it is built this way | [Concepts](concepts/index.md) |
| read the experimental results and what they mean | [Articles](articles/index.md) |
| look up a class or a function | [API reference](reference/index.md) |
| decode a piece of process-control jargon | [Glossary](glossary.md) |

## The shape of the code in one diagram

```mermaid
flowchart LR
    SC["<b>Scenario</b><br/>setpoint programme<br/>disturbance programme<br/>dt, seed, windows"]
    CT["<b>Controller</b><br/>ON/OFF · PID · velocity PID<br/>cascade · feedforward · PWM"]
    PL["<b>Plant</b><br/>tank · integrating tank<br/>series tanks · inverse response<br/>cascade process"]
    SIM["<b>simulate()</b><br/>one closed loop,<br/>used by everything"]
    MET["<b>metrics</b><br/>tracking + effort<br/>+ constraint + compute"]
    PLT["<b>plotting</b><br/>three-panel figure<br/>trade-off · sweep"]

    SC --> SIM
    CT --> SIM
    PL --> SIM
    SIM -->|"tidy DataFrame"| MET
    SIM -->|"tidy DataFrame"| PLT

    style SIM fill:#00897b,color:#fff
```

The `Plant` is the truth and owns every physical limit. The `Controller` sees
only a noisy measurement. `simulate()` is the single closed loop that every
comparison in the project runs through — nobody gets a different integrator, a
different sample time, or a different noise stream.

## Headline results so far

| experiment | headline |
|---|---|
| [ON/OFF vs PID](articles/01-onoff-vs-pid.md) | ON/OFF limit-cycles at ±18 % with a period of ~4θ; SIMC PI settles without offset |
| [Integral windup](articles/02-integral-windup.md) | back-calculation is worth **16×** on recovery IAE — a structural fix, not a tuning one |
| [Tuning shootout](articles/03-tuning-shootout.md) | ten PI rules scored on performance **and** robustness Ms; SIMC sits at the knee |
| [Relay auto-tuning](articles/04-relay-autotune.md) | the industrial autotune recovers Pu to +7 % and under-gains by 24 % — and errs safe |
| [Dead-time sweep](articles/05-dead-time-sweep.md) | usable loop gain falls **80×** while the peak deviation approaches a floor no controller can beat |
| [Averaging level](articles/06-averaging-level.md) | tight and averaging level control rank **opposite** depending on the objective |
| [Cascade](articles/07-cascade.md) | **7.7×** on the disturbance it was designed for, 0.9× on the one it was not |
| [Feedforward](articles/08-feedforward.md) | **5.4×** on IAE, 16× on peak — and what a 30 % gain error costs |
| [Inverse response](articles/09-inverse-response.md) | more gain deepens the wrong-way dip **8.6×**; an RHP zero costs what dead time costs |

The complete running record, with every table, is in
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md).

## What the experiments demonstrate

1. **Tuning is a frontier, not an optimum.** Ten published PI rules on one
   plant span 4× in load-disturbance IAE and 5.7× in valve travel, lined up
   monotonically against maximum sensitivity M<sub>s</sub>. Choosing a rule is
   a decision about expected model error, and both axes get reported.
2. **Structure beats tuning, repeatedly.** Anti-windup 16×, cascade 7.7×,
   feedforward 5.4× — every one larger than the entire spread of ten tuning
   rules on the same plant. When a loop underperforms, the question is usually
   which structure is missing, not which gain is wrong.
3. **Some of the loss is physics.** After a load step no controller can act for
   one dead time. In the dead-time-dominant regime a SIMC PI is already within
   2 % of that floor on peak deviation, while the recovery degrades 8×.
4. **The metric decides the winner.** On a surge tank, tight and averaging
   level control rank exactly opposite depending on the objective — and the
   controller with the best robustness number is the one that breaches the
   alarm band.
5. **Every improvement costs valve travel.** Nothing here is reported without
   the effort column.

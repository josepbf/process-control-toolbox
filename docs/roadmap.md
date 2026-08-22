# Roadmap

Organised by capability. Each item lands with the tuning method, the tests and
the experiment that demonstrate it.

| capability | content | status |
|---|---|---|
| harness and metrics | closed-loop runner, scenarios, tracking/effort/constraint/computation metrics, plotting | :material-check-circle: done |
| plants | FOPDT tank, integrating surge tank, series lags, inverse response, two-stage cascade | :material-check-circle: done |
| controllers | ON/OFF, time-proportioning, PID (positional and velocity), cascade, feedforward | :material-check-circle: done |
| tuning and analysis | thirteen named rules, half-rule reduction, M<sub>s</sub>/GM/PM analysis, relay auto-tuning | :material-check-circle: done |
| **dead-time compensation** | Smith predictor and variants | :material-arrow-right-circle: next |
| selector, ratio, split-range | the classical structural repertoire beyond cascade and feedforward | |
| MIMO and interaction | quadruple tank, RGA pairing, decentralised control | |
| model predictive control | *conditional* — see below | |

System identification is deliberately out of scope for now.

## What the existing experiments oblige the next ones to do

The results so far are not a warm-up. They set specific, falsifiable
expectations that anything added later has to meet.

### Structure beats tuning, repeatedly

| structural change | benefit |
|---|---|
| anti-windup ([2](articles/02-integral-windup.md)) | 16× |
| cascade ([7](articles/07-cascade.md)) | 7.7× |
| feedforward ([8](articles/08-feedforward.md)) | 5.4× |
| *the entire spread of ten tuning rules* | *~4× on load IAE* |

Every structural change is worth more than the whole tuning axis. Any new
control strategy is therefore measured against a *well-structured* classical
scheme — a cascade with feedforward, correct anti-windup, and a cited tuning
rule — not against a bare PID, and certainly not against a Ziegler–Nichols
loop showing 50 % overshoot.

### The peak deviation is mostly physics

In the dead-time-dominant regime PI is already within **2 %** of the
theoretical floor on peak deviation ([article 5](articles/05-dead-time-sweep.md)).
The remaining headroom is in the **recovery**, in **constraint handling**, and
in **multivariable coordination**.

A result claiming a large improvement in peak deviation at high θ/τ is a
result to be checked, not celebrated.

### Every improvement costs valve travel

Cascade 2.1×, feedforward 5.8×, aggressive tuning 5.7×. No comparison is
reported without the effort column, and nothing added later is exempt.

## Dead-time compensation, next

The Smith predictor is the classical answer to the θ/τ degradation measured in
[article 5](articles/05-dead-time-sweep.md), and it brings two utilities the
toolbox does not yet have:

1. **A discrete internal model** — FOPDT to state space, discretisation, and
   dead-time state augmentation.
2. **A state observer** — `compute()` receives `y`, not `x`, so any model-based
   law needs an estimate.

Both are prerequisites for anything model-based, which is why they arrive with
the Smith predictor rather than later.

## On model predictive control

MPC is listed as conditional, and the condition is architectural: it is added
if it fits the `Controller` abstraction rather than requiring the toolbox to be
reshaped around it.

It needs three things beyond the current interface — a state estimate, optional
setpoint preview, and a way to publish predicted trajectories. Each is
additive, each follows a pattern already working in the codebase, and the first
two are independently required by the Smith predictor above. Output limits
already exist on every plant as reporting-only bands with violation metrics, so
constraint handling needs no new machinery.

The extension contract is written up in
[architecture](concepts/architecture.md) and in
[`ARCHITECTURE.md`](https://github.com/josepbf/process-control-toolbox/blob/main/ARCHITECTURE.md).

## Contributing a result

The conventions an experiment script follows — docstring as argument,
`build()` separate from `main()`, everything to `results/`, a fundamental
limit on the plot wherever one exists — are in
[Tutorial 6](tutorials/06-designing-an-experiment.md).

Then write it up in
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md),
including the cases where the new thing loses.

# Roadmap

Organised by capability. Each item lands with the tuning method, the tests and
the experiment that demonstrate it.

| capability | content | status |
|---|---|---|
| harness and metrics | closed-loop runner, scenarios, tracking/effort/constraint/computation metrics, plotting | :material-check-circle: done |
| plants | FOPDT tank, integrating surge tank, series lags, inverse response, two-stage cascade | :material-check-circle: done |
| controllers | ON/OFF, time-proportioning, PID (positional and velocity), cascade, feedforward | :material-check-circle: done |
| tuning and analysis | thirteen named rules, half-rule reduction, M<sub>s</sub>/GM/PM analysis, relay auto-tuning | :material-check-circle: done |
| dead-time compensation | Smith predictor, discrete internal model, Kalman observer | :material-check-circle: done |
| selector, ratio, split-range | the classical structural repertoire beyond cascade and feedforward | |
| MIMO and interaction | quadruple tank, RGA pairing, decentralised control | |
| **model predictive control** | linear MPC as a condensed QP, soft output constraints, setpoint preview, cited tuning | :material-check-circle: done — condition discharged, see below |

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

## On model predictive control

MPC was listed as conditional, and the condition was architectural: it is added
**if it fits the `Controller` abstraction rather than requiring the toolbox to
be reshaped around it**. That condition is now discharged, and the accounting is
worth stating plainly.

What MPC needed beyond the original interface was three things, each additive,
each justified by a classical need of its own:

| needed | how it landed | who else wanted it |
|---|---|---|
| a state estimate | `models.observer.KalmanObserver` | the Smith predictor, first |
| setpoint preview | the `uses_preview` flag, following `uses_measured_disturbance` exactly | setpoint ramping, kiln heat-up profiles |
| predicted trajectories | `Controller.snapshot()` into `df.attrs`, drawn by `plot_horizon` | anything with a horizon |

`Plant` did not change. `simulate()` gained one keyword and lost a branch. Output
limits already existed on every plant as reporting-only bands with violation
metrics, so constraint handling needed no new machinery at all.

### The one thing delegated, and the one thing not

Building the prediction matrices, the disturbance model and the constraint rows
is toolbox code. The *numerical QP solve* sits behind a single `solve_qp`
boundary whose default is dependency-free — see [solvers](reference/solvers.md).
The default install is still numpy, scipy, matplotlib and pandas.

That split is not squeamishness about dependencies. `FINDINGS.md` records PI at
1.02× the physical floor on peak deviation; this roadmap says a large claimed
improvement there is a result to be checked, not celebrated. A black-box MPC
cannot be checked. Owning the matrices is the audit trail.

### What it actually bought

The honest summary of [article 11](articles/11-mpc.md), and it is not the
summary a vendor would write:

- **With nothing binding, MPC ties a well-tuned PI and is very slightly worse
  on the trade-off.** Tuned from the same FOPDT model by cited rules — SIMC for
  the PI, Shridhar & Cooper for the MPC — they land within 0.1 % on IAE, and the
  PI gets there with less valve travel. An unconstrained MPC *is* a linear
  controller; `LinearMPC.linear_gain()` returns which one.
- **The move weight `R` brackets the PI.** Small `R` buys tracking error with
  valve travel; large `R` gives it back. So "MPC beat the PI on IAE" is a
  statement about a weight somebody chose, not about MPC.
- **With an output limit binding, MPC respects it and the PI cannot.** That is
  the structural difference, and it is the only one. It costs 2.7× the valve
  travel and about 230× the computation per sample.

### What is still missing

No terminal cost and no terminal constraint: the stability argument is a horizon
long enough to cover the settling time, which is the industrial DMC position and
is stated as such. No nonlinear MPC — and, more to the point, **no nonlinear
plant to need one**. Adding `do-mpc` before there is a process that defeats a
single linear model would be a solution without a problem; the honest order is a
pH loop or a jacketed reactor first, `LinearMPC` visibly failing on it, and only
then CasADi.

## Contributing a result

The conventions an experiment script follows — docstring as argument,
`build()` separate from `main()`, everything to `results/`, a fundamental
limit on the plot wherever one exists — are in
[Tutorial 6](tutorials/06-designing-an-experiment.md).

Then write it up in
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md),
including the cases where the new thing loses.

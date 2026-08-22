# Roadmap

| phase | content | status |
|---|---|---|
| 1 | harness, FOPDT tank, ON/OFF, PID, SIMC | :material-check-circle: done |
| 1+ | integrating / high-order / inverse-response / cascade plants; cascade, feedforward, velocity-form and PWM controllers; thirteen tuning rules; Ms robustness analysis; relay auto-tuning | :material-check-circle: done |
| 2 | linear MPC built by hand (prediction matrices → QP in CVXPY); unconstrained MPC must converge to LQR, as a unit test | :material-arrow-right-circle: next |
| 3 | quadruple tank, RGA pairing, decentralised PID vs multivariable MPC, minimum and non-minimum phase | |
| 4 | system identification (PRBS, ARX/subspace) and deliberate model mismatch sweeps | |
| 5 | nonlinear: CSTR with NMPC, grinding circuit with recycle and long dead time, Smith predictor baseline | |
| 6 | optional: MHE, offset-free MPC, RL contender | |

## What phase 1 established, and what it obliges phase 2 to do

Phase 1 was not a warm-up. It was the construction of a set of expectations
that phase 2 onward has to meet, and each one is a specific, falsifiable
constraint on what a good MPC result will look like.

### The baseline is fixed, and it was fixed first

**SIMC (τ_c = θ)**, with **AMIGO** as the robust alternative, chosen in
[article 3](articles/03-tuning-shootout.md) on stated grounds before any MPC
exists. Both appear in later comparisons.

Any MPC result in this project is measured against those two, not against a
Ziegler–Nichols loop showing 50 % overshoot.

### Structure beats tuning, repeatedly

| structural change | benefit |
|---|---|
| anti-windup ([2](articles/02-integral-windup.md)) | 16× |
| cascade ([7](articles/07-cascade.md)) | 7.7× |
| feedforward ([8](articles/08-feedforward.md)) | 5.4× |
| *the entire spread of ten tuning rules* | *~4× on load IAE* |

Every structural change is worth more than the whole tuning axis.

!!! danger "So the question for MPC is not the obvious one"
    Not *"can it beat a PID"* — but **"can it beat a well-structured classical
    scheme"**: a cascade with feedforward, correct anti-windup, and a cited
    tuning rule.

    Phases 3–5 have to be set up that way, or the comparison is not worth
    running.

### The peak deviation is mostly physics

In the dead-time-dominant regime PI is already within **2 %** of the
theoretical floor on peak deviation ([article 5](articles/05-dead-time-sweep.md)).
MPC's opportunity is in the **recovery**, in **constraint handling**, and in
**multivariable coordination** — not in the peak.

An MPC result claiming a large improvement in peak deviation at high θ/τ is a
result to be checked, not celebrated.

### Every improvement costs valve travel

Cascade 2.1×, feedforward 5.8×, aggressive tuning 5.7×. No comparison is
reported without the effort column, and MPC will not be exempt.

## Phase 2 in detail

Linear MPC, built by hand:

1. **Prediction matrices** from a discrete state-space model — the $\Phi$ /
   $\Gamma$ construction written out rather than called.
2. **The QP**, assembled explicitly and handed to CVXPY/OSQP. Cost weights,
   horizons, input and output constraints.
3. **The unit test that matters**: unconstrained MPC with an infinite horizon
   must converge to the LQR solution. If it does not, the prediction matrices
   are wrong.
4. **The scenario PID cannot win**: an *active output constraint*. Phase 1
   established that on an unconstrained fast SISO loop there is nothing for
   MPC to do ([article 1](articles/01-onoff-vs-pid.md)), so phase 2 has to
   build the case where anticipating a limit is worth something.
5. `solve_ms_mean` and `solve_ms_p95` in the table, in the column that has
   been there since experiment 1.

The `src/identification/` package exists and is empty; it is phase 4's home.

## Contributing a result

The conventions an experiment script follows — docstring as argument,
`build()` separate from `main()`, everything to `results/`, a fundamental
limit on the plot wherever one exists — are in
[Tutorial 6](tutorials/06-designing-an-experiment.md).

Then write it up in
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md),
including the cases where the new thing loses.

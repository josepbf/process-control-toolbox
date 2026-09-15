# Model predictive control

What MPC is, in one sentence: **solve a constrained optimisation over a horizon
every sample, apply the first move, throw the rest away, repeat.**

The "throw the rest away" is not waste. It is what makes the scheme feedback
rather than a schedule — the next sample starts from a fresh measurement, so an
error in the model or an unexpected disturbance is corrected rather than
accumulated.

## The problem solved every sample

The decision vector is the sequence of **move increments** plus one slack
variable, $z = [\Delta u_0 \ldots \Delta u_{M-1},\; \varepsilon]$. With the
estimated state $\hat{x}$ and the last move issued $u_{-1}$:

\[
Y \;=\; \underbrace{\Psi \hat{x} + \Upsilon u_{-1}}_{\text{free response}}
\;+\; \underbrace{\Theta\, \Delta U}_{\text{what this sample's moves buy}}
\]

- $\Psi$ has block rows $C A^i$ — where the state is heading on its own.
- $\Upsilon$ has block rows $C S_i$ with $S_i = \sum_{j<i} A^j B$ — the response
  to simply holding the last move.
- $\Theta$ is block lower triangular with $\Theta_{i,l} = C S_{i-l}$: the effect
  at sample $i$ of a unit increment applied at $l$ **and held from then on**.

Beyond the control horizon $M$ the increments are zero, so the last move stands
to the end of the prediction horizon. That is what makes a short control horizon
stabilising rather than short-sighted.

The cost is

\[
J = (Y-T)^\top \bar{Q}(Y-T) + \Delta U^\top \bar{R}\,\Delta U
    + \rho|\varepsilon| + \rho_q \varepsilon^2
\]

giving $H = \Theta^\top\bar{Q}\Theta + \bar{R}$ and
$g = \Theta^\top\bar{Q}(f - T)$. For a linear time-invariant model with fixed
weights **$H$ never changes**, so it is factorised once at construction rather
than once a sample — which matters, because solve time is a reported metric and
self-inflicted cost distorts a comparison as surely as an unfair advantage.

$T$ is the setpoint repeated across the horizon, or the previewed trajectory
when `preview=True`. That substitution is the entire implementation of preview.

## Three decisions, and why

### Increments, not positions

Penalising $\Delta u$ and never $u$ means the unconstrained steady state has
$\Delta u \to 0$ and $y \to sp$ **exactly**. Offset-free tracking falls out of
the parameterisation.

The alternative — penalising $(u - u_s)$ against a steady-state target — needs
you to compute $(x_s, u_s)$ from

\[
\begin{bmatrix} I-A & -B \\ C & 0\end{bmatrix}
\begin{bmatrix} x_s \\ u_s \end{bmatrix}
= \begin{bmatrix} B_d \hat{d} \\ sp - C_d \hat{d}\end{bmatrix}
\]

every sample, which adds a linear solve, a feasibility question and a second
failure mode. It is the right choice the moment anything penalises $u$ itself or
asks for an input target. Nothing here does, so it is not done — but the reader
who expects to see it should know it was a decision rather than an oversight.

!!! warning "This is not the whole of offset-free"
    The increment parameterisation removes the *structural* offset. It does
    nothing about an unmeasured load, which the model has no representation of
    at all. That is what the disturbance state below is for, and without it an
    MPC sits on an offset exactly as a P-only PID does.

### Input constraints hard, output constraints soft

Input limits are physics: the valve does not open past 100 %. They are hard
rows, and the controller is given **the same numbers the plant enforces** — it
does not get its own, softer actuator.

Output limits are declared bands that [the plant does *not* enforce](../reference/plants.md).
A run may legitimately start outside one. A hard output constraint would then be
infeasible at the very first sample — a silent failure buried in a sweep — so
they are soft, with an exact penalty:

\[
y_{min} - \varepsilon \le \Theta \Delta U + f \le y_{max} + \varepsilon,
\qquad \varepsilon \ge 0
\]

With $\rho$ above the multiplier norm of the hard problem, this recovers
hard-constraint behaviour wherever the hard problem is feasible at all
(Kerrigan & Maciejowski 2000). The quadratic term $\rho_q$ is there only to keep
the slack block of $H$ positive definite.

The slack is published as a diagnostic. A controller that met its limits by
spending slack has not met its limits.

### The horizon is the stability argument

There is **no terminal cost and no terminal constraint** here. The prediction
horizon is instead sized to cover the open-loop settling time —
`tuning.mpc_rules.settling_horizon`, roughly $(\theta + 5\tau)/dt$.

That is the industrial DMC position, it is what the Shridhar–Cooper tuning rule
assumes, and it is stated rather than implied. A horizon too short to see the
process settle optimises over a window in which the consequences of its own
moves have not yet arrived — which on a dead-time dominant loop is most of them.
No weight fixes that.

## Where the integral action comes from

`compute()` receives $y$, not $x$, so there is an observer. The observer's model
is **augmented with an integrating disturbance state**, and that state is the
integral action:

\[
A_a = \begin{bmatrix} A & 0 \\ 0 & I\end{bmatrix},\qquad
C_a = \begin{bmatrix} C & I \end{bmatrix}
\quad\text{(output disturbance)}
\]

The estimate $\hat{d}$ is held constant across the horizon — the "constant
future disturbance" assumption every industrial DMC makes — and it enters the
free response, so the optimiser plans against a world in which the upset
persists.

The choice between an output and an input disturbance model is **physical, not
numerical**: output says the upset appears at the measurement, input says it
enters where the valve does and is rejected through the process dynamics. And it
is not always free: an *integrating* process with an output-disturbance model is
undetectable — two integrators indistinguishable from one measurement — and
`augment_disturbance` refuses it rather than misbehaving quietly.

The ratio $q_d / r$ sets how fast $\hat{d}$ moves, and it behaves exactly like
$1/T_i$: large is aggressive load rejection and amplified noise, small is
sluggish. It is a tuning knob, so it is named and recorded.

## What MPC is *not*

**Not automatically better than a PI.** With no constraint active an MPC *is* a
linear controller — `LinearMPC.linear_gain()` returns which one — and a
well-tuned PI is already that. [Article 11](../articles/11-mpc.md) finds them
tied to 0.04 % when both are tuned from the same model by cited rules.

**Not a substitute for structure.** Cascade is worth 7.7× and feedforward 5.4×
on their own plants. An MPC that ignores an available secondary measurement is
throwing away more than its optimiser can recover.

**Not a way to escape physics.** [Article 5](../articles/05-dead-time-sweep.md)
puts PI within 2 % of the theoretical floor on peak deviation at high θ/τ. MPC
does not reach past a floor; a result claiming it does is a result to check.

**Not robust because it is optimal.** It is optimal *for its model*. The
[Smith predictor article](../articles/10-smith-predictor.md) shows what a 3×
error in θ does to a scheme that depends on a model — an order of magnitude
worse than the controller it replaced. MPC inherits that exposure.

What it does have, that a PI structurally cannot, is **a representation of a
limit it is not allowed to cross**. That is the whole of the difference, and it
is enough.

## Further reading

- Cutler & Ramaker (1980), "Dynamic matrix control — a computer control
  algorithm" — the original industrial scheme.
- Maciejowski (2002), *Predictive Control with Constraints*.
- Rawlings, Mayne & Diehl (2017), *Model Predictive Control: Theory,
  Computation, and Design* — the modern reference on stability and terminal
  ingredients, which this implementation deliberately does without.
- Muske & Badgwell (2002) and Pannocchia & Rawlings (2003) on disturbance
  modelling for offset-free control.
- Shridhar & Cooper (1997) for the tuning rule used here.

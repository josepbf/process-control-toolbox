# Glossary

Process-control vocabulary as it is used in this project. Where a term maps
onto something in the code, the code is named.

---

### AMIGO

*Approximate M-constrained Integral Gain Optimisation.* A PID tuning rule
(Åström & Hägglund, 2004) fitted to a batch of representative process models
under an explicit robustness constraint, $M_s \le 1.4$. This project's
**robust** baseline. → `amigo_pi()`

### Anti-windup

Any mechanism that stops the integral term accumulating while the actuator is
saturated. This project uses **back-calculation**: the integrator is fed the
amount by which the requested move was clipped, divided by a tracking time
constant `Tt`. → [Article 2](articles/02-integral-windup.md)

### Averaging level control

A deliberately loose P-only design for a surge tank, whose objective is to
*absorb* flow variation rather than hold level. → `averaging_level_pi()`,
[Article 6](articles/06-averaging-level.md)

### Back-calculation

See *anti-windup*.

### Bumpless transfer

Starting a controller from the valve position the operator left it at, rather
than stepping the actuator. In `PIDController` this is the `u0` argument,
which chooses the initial integral so the first output is the nominal
position. Free in the velocity form.

### Cascade control

Two nested loops: the primary controller's output is the **setpoint** of a
faster secondary controller closing on an intermediate measurement. Buys
information rather than gain. → `CascadeController`,
[Article 7](articles/07-cascade.md)

### Dead time (θ)

Transport delay: the interval between a cause and the first sign of its
effect. Contributes phase lag without gain reduction, which is what makes it
so damaging. Modelled in **three independent paths** here — manipulated,
disturbance, measurement. → [Dead time](concepts/dead-time.md)

### Deadband / hysteresis

The band around the setpoint in which an ON/OFF controller holds its previous
position. Stops chattering on measurement noise; guarantees a limit cycle. →
`OnOffController(hysteresis=...)`

### Describing function

The approximation behind relay auto-tuning: keep only the fundamental harmonic
of the relay's square wave and assume the process filters the rest. Good on a
lag-dominant process, 10–20 % off on a dead-time-dominant one. → `relay_autotune()`

### FOPDT

*First order plus dead time*, $G(s) = K e^{-\theta s}/(\tau s + 1)$. The
two-and-a-half-parameter model nearly every classical tuning rule is stated
in. → `Tank`, `plant.fopdt`

### Gain margin (GM) / phase margin (PM)

How much the loop gain can grow, or how much extra phase lag can appear,
before the closed loop becomes unstable. Both are bounded by $M_s$. →
[Robustness](concepts/robustness.md)

### Half rule

Skogestad's reduction of a high-order model to FOPDT: the largest neglected
lag is split in half between the retained time constant and the effective dead
time; everything smaller, plus RHP zeros and half the sample time, goes
entirely into the dead time. → `half_rule()`

### IAE / ISE / ITAE

Integral of absolute / squared / time-weighted absolute error. The three
standard tracking metrics. → [Metrics](concepts/metrics.md)

### Integrating process

A process with no self-regulation: leave the valve alone and the output ramps
rather than settling. Surge tanks, drum levels, silos. Needs its own tuning
rules — there is no `tau` to cancel. → `IntegratingTank`, `simc_integrating()`

### Inverse response

The output initially moves *opposite* to its eventual direction. Equivalent to
a right-half-plane zero. → `InverseResponseTank`,
[Article 9](articles/09-inverse-response.md)

### Lambda tuning (Dahlin)

$K_c = \tau / (K(\lambda + \theta))$, $T_i = \tau$. The DCS default in pulp,
paper and cement. Cancels the process lag exactly, which costs load-rejection
performance. → `lambda_tuning()`

### Limit cycle

A self-sustaining oscillation the loop settles into and never leaves. For an
ON/OFF controller it is not a fault to be tuned away — it is what the control
structure *is*. Reported as `settling_2pct_s = NaN`.

### Maximum sensitivity (Ms)

$\max_\omega |1/(1+L(j\omega))|$ — the inverse of the shortest distance from
the Nyquist curve to the −1 point. Bounds GM and PM simultaneously. 1.2–1.6 is
comfortable process practice. → `pid_on_fopdt()`,
[Robustness](concepts/robustness.md)

### Manipulated variable (MV)

What the controller moves: `u`. Usually a valve position in %.

### Minimum phase / non-minimum phase

A transfer function with all zeros in the left half plane is minimum phase.
A right-half-plane zero (or a dead time) makes it non-minimum phase, and puts
a hard ceiling on achievable bandwidth.

### Overshoot

Peak excursion beyond the setpoint after a step, as a percentage of the step.
Returns `NaN` in a window with no meaningful setpoint change — read
`peak_dev` instead.

### Reaction curve

The open-loop step response, from which FOPDT parameters are read. →
[Tutorial 2](tutorials/02-tuning-a-loop.md)

### Relay auto-tuning

Replace the controller with a relay; the resulting bounded limit cycle sits at
the ultimate frequency, giving $K_u$ and $P_u$ without a model and without
walking the plant to the edge of instability. → `relay_autotune()`,
[Tutorial 3](tutorials/03-relay-autotuning.md)

### Reversals

Direction changes of the actuator. Valve wear tracks reversals more closely
than travel, which is why this metric exists. Moves below 0.5 % of the
actuator span are ignored so measurement dither is not counted as wear.

### RHP zero

*Right-half-plane zero.* See *inverse response*.

### Self-regulating process

A process that settles somewhere on its own when the valve is left alone. The
opposite of an *integrating process*.

### Setpoint weighting (β)

Applying only a fraction of the setpoint to the proportional term, to soften
the kick on a setpoint step: $u = K_c(\beta \cdot sp - y + \ldots)$. →
`PIDController(beta=...)`

### SIMC

*Simple/Skogestad IMC.* $K_c = \tau/(K(\tau_c+\theta))$,
$T_i = \min(\tau, 4(\tau_c+\theta))$. **This project's PID baseline**, at
$\tau_c = \theta$. The `min` is the half of the rule that fixes load
rejection. → `simc_pi()`, [Article 3](articles/03-tuning-shootout.md)

### Smith predictor

A dead-time compensator: the controller acts on a model-predicted
*undelayed* output, with the delayed model output compared against the real
measurement for correction. Next on the roadmap; not yet implemented.

### Surge tank

A vessel between two units whose purpose is to absorb flow variation. Tight
level control defeats it. → [Article 6](articles/06-averaging-level.md)

### Time proportioning (PWM)

Turning a continuous demand into ON/OFF pulses fast compared with the process,
so the process averages them. How an oven holds temperature to a fraction of a
degree with a relay output. → `TimeProportioningController`

### Total variation TV(u)

$\sum|\Delta u|$ — the total distance the actuator travelled. The
control-effort metric that appears in every table in this project.

### Tracking time constant (Tt)

The back-calculation gain in the anti-windup path. Åström's rule of thumb:
$T_t = \sqrt{T_i T_d}$ for PID, $T_t = T_i$ for PI.

### Tyreus–Luyben

$K_c = K_u/3.2$, $T_i = 2.2 P_u$. The conservative closed-loop rule most
industrial autotuners ship. → `tyreus_luyben()`

### Ultimate gain / ultimate period (Ku, Pu)

The proportional gain at which the loop oscillates with constant amplitude,
and the period of that oscillation. The inputs to the closed-loop tuning
rules. → `ultimate_gain_period()`, `relay_autotune()`

### Velocity form (incremental PID)

A PID that computes the *change* in output, $u_k = u_{k-1} + \Delta u_k$.
Windup is structurally limited and bumpless transfer is free, which is why
most DCS and PLC function blocks implement it. → `VelocityPIDController`

### Windup

See *anti-windup*.

### θ/τ (dead-time ratio)

The single best predictor of how hard a loop is to control. Below ~0.2 almost
anything works; above ~1 feedback alone struggles. →
[Article 5](articles/05-dead-time-sweep.md)

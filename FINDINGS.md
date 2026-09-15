# FINDINGS

What the toolbox demonstrates, quantitatively. Each section is produced by the
experiment script named against it, and every number here regenerates from a
fixed seed.

These are not a benchmark of one method against another. They are what the
components in the toolbox actually do to a process, measured: what a deadband
costs, what anti-windup is worth, how tuning rules trade performance against
robustness, where feedback runs out of road as dead time grows, and what a
structural change like cascade or feedforward buys and charges for. Where a
comparison appears, it is between classical alternatives, on identical plants,
scenarios and seeds.

Every result carries a control-effort number alongside its tracking number.
That is the house rule, and several of the findings below only make sense
because of it.

---

## Two-position vs PID control on a FOPDT tank

Plant: `Tank(K=1.5, tau=60 s, theta=15 s, Kd=1.0)`, valve 0–100 %, measurement
noise sigma = 0.15 %, seed 7, dt = 1 s. Dead-time ratio theta/tau = 0.25.
Scenario: setpoint 30 → 50 % at t = 100 s, unmeasured load step d = −12 at
t = 700 s. Script: `experiments/exp01_onoff_vs_pid.py`.

Both PI tunings come from published rules applied to the **true** plant
parameters — the most generous possible setting for the baselines. Model
mismatch is a separate question, taken up when identification lands.

| controller | window | IAE | settling (2 %) | overshoot | peak dev | TV(u) | reversals |
|---|---|---:|---:|---:|---:|---:|---:|
| ON/OFF (deadband 1 %) | setpoint | 5903 | never | 95 % | 27.1 | 1900 | 18 |
| ON/OFF | disturbance | 6294 | never | — | 20.8 | 2400 | 23 |
| PI, SIMC (tau_c = theta) | setpoint | 715 | 92 s | 6.0 % | 20.2 | 140 | 15 |
| PI, SIMC | disturbance | 411 | 153 s | — | 4.49 | 157 | 34 |
| PI, Ziegler–Nichols | setpoint | 902 | 142 s | 50.4 % | 20.1 | 309 | 108 |
| PI, Ziegler–Nichols | disturbance | 234 | 111 s | — | 3.93 | 284 | 156 |

**ON/OFF limit-cycles permanently**, as it must: the valve is either fully open
or fully shut, so once the level crosses the deadband the process keeps moving
for another `theta` seconds before the correction even arrives. Amplitude is
±18.0 % of level (37.8–73.8 %) around a 1 % deadband, with a period of ~62 s
≈ 4·theta — the dead time, not the deadband, sets both. Its IAE is 8× worse
than SIMC PI on the setpoint change and 15× worse on the load disturbance, its
valve travel is 13–15× worse, and it never settles, so settling time is
correctly reported as NaN.
It has exactly one virtue: it needs no model and no tuning.

**SIMC PI beats Ziegler–Nichols PI on setpoint tracking, and loses to it on
load rejection.** This is the trade the project exists to measure, and it shows
up on the very first experiment. ZN's higher gain (Kc = 2.40 vs 1.33) buys a
21 % smaller disturbance peak and faster load recovery, and costs 50 %
overshoot on the setpoint step and 2.2× the valve travel. Neither is "better"
without saying what the loop is for. If this were a real mill loop, the SIMC
tuning is the one that survives a valve maintenance review.

**Reported honestly: on this loop there is nothing for MPC to do.** One input,
one output, no active constraint, a dead-time ratio of 0.25 that a PI handles
comfortably, and a tracking error already down at the noise floor. Nothing
more elaborate than a PI is warranted here, and the toolbox says so. The
interesting cases are the ones the later experiments construct: an active
constraint, severe dead time, or loop interaction.
Expect PID to remain the right answer for fast SISO loops throughout.

## Integral windup

Script: `experiments/exp02_antiwindup.py`. Same plant and same SIMC tuning; the
valve is mechanically limited to 30 % (so the maximum attainable level is
45 %), and the setpoint is driven to an infeasible 60 %, held 500 s, then
dropped to 35 %.

| controller | recovery IAE | recovery settling | valve travel during saturation |
|---|---:|---:|---:|
| PI, no anti-windup | 8573 | 972 s | 0 |
| PI + back-calculation | 538 | 49 s | 13.0 |

(The 13.0 vs 0 in the last column is not noise-chasing for its own sake: with
back-calculation the integrator parks *at* the saturation boundary, so the
valve is one sample away from responding. The naive integrator is buried far
inside windup and is therefore, in effect, disconnected.)

**16× difference in recovery IAE from a structural fix, not from tuning.** The
naive integrator accumulates for the whole 500 s of saturation; when the
setpoint becomes reachable again it sits pinned at the limit for a further
~800 s unwinding a fictitious integral before the valve moves at all. The
level stays 10 % above target that entire time.

The industrial reading: the difference between a usable and an unusable loop
here has nothing to do with Kc or Ti. Any comparison that tunes PID carefully
but leaves anti-windup out is not comparing control laws, it is comparing a
correct implementation against a broken one — and constrained operation is
exactly the regime where MPC is supposed to be judged.

---

## Choosing a tuning rule: the performance/robustness frontier

Script: `experiments/exp03_tuning_shootout.py`. Ten published PI rules, same
plant (`K=1.5, tau=60, theta=15`), same scenario, same seed. Each scored on
time-domain performance **and** on maximum sensitivity Ms (peak of `1/(1+L)`),
the standard frequency-domain robustness measure. Sorted by Ms:

| rule | Kc | Ti | Ms | GM | PM | IAE setpoint | overshoot | IAE load | TV(u) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Lambda (λ=τ) | 0.53 | 60.0 | 1.19 | 7.85 | 79° | 1519 | 1.9 % | 935 | 116 |
| AMIGO | 0.61 | 49.2 | 1.25 | 6.71 | 70° | 1209 | 3.4 % | 704 | 132 |
| Lambda (λ=3θ) | 0.67 | 60.0 | 1.25 | 6.28 | 76° | 1229 | 1.9 % | 761 | 144 |
| SIMC (τ_c=2θ) | 0.89 | 60.0 | 1.35 | 4.71 | 71° | 942 | 2.0 % | 587 | 193 |
| Tyreus–Luyben | 1.44 | 120.9 | 1.58 | 3.06 | 74° | 1096 | 1.4 % | 689 | 321 |
| **SIMC (τ_c=θ)** | **1.33** | **60.0** | **1.59** | **3.14** | **61°** | **715** | **6.0 %** | **411** | **297** |
| IMC | 2.00 | 67.5 | 2.09 | 2.12 | 49° | 717 | 23.8 % | 321 | 465 |
| ZN closed-loop | 2.08 | 45.8 | 2.37 | 1.93 | 39° | 837 | 40.8 % | 244 | 499 |
| ZN open-loop | 2.40 | 50.0 | 2.78 | 1.70 | 35° | 902 | 50.4 % | 234 | 593 |
| Cohen–Coon | 2.46 | 33.0 | 3.66 | 1.51 | 23° | 1266 | 74.6 % | 247 | 663 |

Three things fall out, and the third is the one that matters for the project.

**The load-rejection axis is monotone in Ms, and so is the valve wear.** IAE on
the load disturbance improves by 4× from the most robust rule to the least
(935 → 234) and valve travel worsens by 5.7× (116 → 663) over the same span.
There is no rule that is simply better; there is a frontier, and choosing a
point on it is an engineering decision about how much model error the plant is
expected to develop.

**Setpoint tracking is not monotone — it has an optimum.** IAE on the setpoint
step falls from 1519 to a minimum of 715 at SIMC and then *rises* again to 1266
at Cohen–Coon, because past a certain gain the overshoot costs more than the
speed gains. This is why "we tuned the PID aggressively" is not the same as
"we tuned the PID well".

**SIMC with τ_c = θ sits at the knee**, and that is the stated ground on which
it is adopted as the standard PID baseline for the rest of the project: the
best setpoint IAE of any rule tested, the second-best load IAE inside the
comfortable robustness band, at GM 3.1 / PM 61° / Ms 1.59 — matching the
robustness Skogestad reports for the rule. Where a *robust* baseline is wanted
instead, AMIGO (Ms 1.25) is the one to use, and both appear in later
comparisons rather than one being quietly chosen after the fact.

A calibration note for the whole project: Cohen–Coon and Ziegler–Nichols are
included because they are what most people picture when they hear "tuned PID",
and they land at Ms 2.8–3.7 — loops that will oscillate the first time the
process gain moves. Benchmarking MPC against *those* would be flattering and
meaningless.

## Relay auto-tuning: how good is the industrial autotune?

Script: `experiments/exp04_relay_autotune.py`. Åström & Hägglund's 1984 relay
experiment (relay amplitude 10 % of valve travel, hysteresis 0.5 % ≈ 3σ of the
measurement noise), run on a plant whose true ultimate values are known
analytically.

| | ultimate gain Ku | ultimate period Pu |
|---|---:|---:|
| relay experiment | 3.51 | 58.7 s |
| analytic truth | 4.62 | 54.9 s |
| error | **−24 %** | **+7 %** |

**The experiment gets the period nearly right and the gain conservatively
wrong**, and both are explainable rather than incidental. The limit cycle sits
at the frequency where the process has −180° of phase, so `Pu` is close by
construction; the residual +7 % is the phase lag of the hysteresis, which is
also why the hysteresis-free run comes out at +2 %. The gain comes from a
describing-function approximation that keeps only the fundamental of the
relay's square wave, and with θ/τ = 0.25 the process does not filter the
harmonics well. The error direction is safe: it under-gains.

Closed loop, the tunings that follow from it:

| tuning | needs a model? | Ms | IAE setpoint | IAE load |
|---|---|---:|---:|---:|
| ZN closed-loop (from relay) | no | 1.82 | 759 | 309 |
| Tyreus–Luyben (from relay) | no | 1.39 | 1521 | 962 |
| SIMC (from the true model) | yes | 1.59 | 715 | 411 |

**An autotune gets you into the right neighbourhood without a model at all**,
which is a real result and the reason the button exists. The model-based rule
still wins: SIMC matches ZN-CL's setpoint tracking at meaningfully better
robustness, and Tyreus–Luyben — the rule most autotuners actually ship, for
good reason — pays 2.3× the load-disturbance IAE for its safety. The gap
between "no model" and "a model" is what a mismatch study would price properly.

## Where feedback runs out of road: the dead-time sweep

Script: `experiments/exp05_deadtime_sweep.py`. τ = 60 s held fixed, θ swept over
two decades, SIMC PI tuned on the true model each time, identical load
disturbance. This is the setup experiment for everything the project is about.

| θ/τ | loop gain Kc·K | Ms | load peak | physical floor | peak / floor | IAE/(τ+θ) |
|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | 10.0 | 1.65 | 1.29 | 0.59 | 2.21 | 2.7 |
| 0.25 | 2.0 | 1.59 | 4.40 | 2.65 | 1.66 | 6.6 |
| 1.0 | 0.50 | 1.59 | 9.00 | 7.59 | 1.19 | 14.0 |
| 2.0 | 0.25 | 1.59 | 11.26 | 10.38 | 1.09 | 18.7 |
| 4.0 | 0.125 | 1.59 | 12.06 | 11.78 | **1.02** | 22.2 |

The "physical floor" is the deviation that accumulates during the dead time,
before any controller's first move can possibly arrive:
`|Kd·d|·(1 − exp(−θ/τ))`. No control law can beat it.

**Usable loop gain collapses by a factor of 80** across the sweep while Ms
stays pinned at 1.59 — the tuning rule is not getting worse, the process is
getting harder, and the rule is correctly refusing to buy performance it cannot
pay for.

**The finding that matters for the rest of the project:** as dead time takes
over, PI gets *closer* to the physical floor on peak deviation — 2.21× the
floor at θ/τ = 0.05, but 1.02× at θ/τ = 4 — while scaled IAE gets **8× worse**.
So dead time costs you two different things:

* the **peak** deviation, which is physics, and which no controller recovers —
  including MPC. In the dead-time-dominant regime there is essentially nothing
  left to win here;
* the **recovery**, which is control, and where all the remaining headroom is.

Any later claim that a Smith predictor or a predictive controller "handles dead time
better" has to show up in the second of those, not the first. This sets the
expectation quantitatively before the contenders arrive, which is the point of
running it now.

## The surge tank: the metric decides the winner

Script: `experiments/exp06_averaging_level.py`. Integrating level process
(`k' = −0.02 %/s per % valve`, θ = 10 s), outflow manipulated, inflow steps as
the disturbance, alarm band 25–75 %.

| controller | Kc | Ti | Ms | peak level dev | TV(outflow) | alarm violations |
|---|---:|---:|---:|---:|---:|---:|
| SIMC tight (τ_c=θ) | −2.50 | 80 | 1.70 | **10.1** | 1862 | 0 |
| SIMC loose (τ_c=5θ) | −0.83 | 240 | 1.17 | 24.0 | 635 | 0 |
| averaging P-only | −0.75 | — | 1.14 | 20.5 | **570** | 0 |
| lambda, integrator ignored | −0.46 | 1000 | 1.09 | 35.4 | 353 | **890** |

**The two rankings are opposite.** On level deviation the tight tuning wins by
2×; on outflow variability — what the downstream unit actually experiences —
it is the worst of the four by 3.3×. A surge tank is installed to give the
downstream unit a steady feed; a controller that holds level perfectly has
passed every inflow upset straight through and removed the reason the tank was
built. Both columns are true. Which one is the objective is a question about
the plant, not about control theory, and it has to be answered before any
comparison means anything.

**A second, sharper warning about single-number robustness.** The worst
controller in the table has the *lowest* Ms (1.09). It earns that by being
detuned into uselessness — it spends 890 samples outside the alarm band. Ms
bounds how close a loop is to instability; it says nothing about whether the
loop is doing its job. Robustness and performance both have to be reported, and
this row is the counterexample to reading either one alone.

## Cascade control: buying information, not gain

Script: `experiments/exp07_cascade.py`. Fast inner stage (valve → flow,
τ = 5 s, measured immediately) feeding a slow outer stage (flow → temperature,
τ = 60 s, measured 20 s late). Both controllers SIMC-tuned; the only difference
is one extra measurement.

| disturbance location | single-loop PI | cascade PI/PI | ratio |
|---|---:|---:|---:|
| inner stage — IAE | 909 | 118 | **7.7×** |
| inner stage — peak deviation | 8.90 | 1.24 | **7.2×** |
| inner stage — settling | 239 s | 57 s | 4.2× |
| outer stage — IAE | 429 | 471 | **0.9× (worse)** |
| setpoint — IAE | 1211 | 1153 | 1.05× |
| valve travel TV(u) | 111 | 236 | **2.1× worse** |

**7.7× on the disturbance it was designed for, nothing on the one it was not,
and double the valve travel throughout.** The negative result is as important
as the positive one: the outer-stage upset never passes through the inner loop,
so the extra measurement carries no information about it, and the cascade is
marginally *worse* there. That is the correct shape for a structural change —
it buys a specific thing, and reporting only the case where it wins would be
the same sleight of hand this project is trying to avoid for MPC.

The asymmetry that makes it work is the measurement, not the dynamics: the
primary is reported 20 s late and the secondary is not.

## Feedforward: the value of a measured disturbance

Script: `experiments/exp08_feedforward.py`. Valve path `K=1.5, τ=60 s, θ=15 s`;
disturbance path `Kd=1.0, τ_d=15 s, θ_d=45 s` — so the ideal compensator is a
real lead-lag with 30 s of usable advance. Disturbance measurement carries its
own noise (σ = 0.4).

| design | IAE | peak deviation | TV(u) |
|---|---:|---:|---:|
| PI only | 427 | 8.99 | 138 |
| + static feedforward (gain only) | 362 | 5.49 | 234 |
| + dynamic feedforward (lead-lag + delay) | **79** | **0.56** | **795** |
| + dynamic feedforward, 30 % gain error | 152 | 2.26 | 617 |
| dynamic feedforward, θ_d = 0 (non-causal ideal) | 312 | 7.49 | 805 |

**Dynamic feedforward is worth 5.4× on IAE and 16× on peak deviation** — far
more than any tuning change on this plant, and more than the whole spread of
the tuning shootout. It is also the cheapest possible "prediction": it works
because the controller is told what is about to happen.

Three qualifications, all of which apply directly to MPC later:

* **The dynamics carry the benefit, not the gain.** Static feedforward — the
  version that gets deployed when nobody has modelled the disturbance path —
  is worth only 1.2×. Getting the ratio right is not enough; the *timing* is
  the thing.
* **It is open loop, so model error is not corrected.** A 30 % error in the
  feedforward gain gives back half the benefit (5.4× → 2.8×). What saves it is
  the feedback controller underneath, which still removes the offset. This is
  the same argument that applies to any model-based scheme: the model buys
  speed, the feedback keeps it honest.
* **It costs 5.8× the valve travel.** The compensator differentiates a noisy
  disturbance measurement, and the lead-lag has a high-frequency gain of
  τ_p/τ_d = 4. Perfect rejection at the cost of a valve that never stops
  moving is not automatically a win.

And the realisability condition is not a technicality: with θ_d = 0, the
disturbance beats the valve to the output, the ideal compensator is non-causal,
and the same design delivers 1.4× instead of 5.4×. Knowing which case you are
in is most of the engineering.

## Inverse response: where more gain digs the hole deeper

Script: `experiments/exp09_inverse_response.py`. Drum-level model: a slow
positive path and a fast negative one, net gain 1.0, RHP zero at T_z = 22.5 s.
The half rule folds that zero into an effective dead time of 27 s, which is how
a classical tuning rule ends up seeing it.

Four SIMC tunings on the same plant:

| tuning | Kc | Ms | wrong-way dip | IAE setpoint | overshoot | IAE load | TV(u) |
|---|---:|---:|---:|---:|---:|---:|---:|
| very slow (τ_c=4θ) | 0.46 | 1.19 | **1.5** | 2516 | 2.1 % | 1504 | 191 |
| conservative (τ_c=2θ) | 0.77 | 1.35 | 3.9 | 1641 | 2.2 % | 998 | 318 |
| SIMC default (τ_c=θ) | 1.16 | 1.59 | 7.3 | **1208** | 2.5 % | 749 | 490 |
| aggressive (τ_c=0.3θ) | 1.78 | 2.18 | **12.9** | 1598 | 52.5 % | 589 | 832 |

The "wrong-way dip" is how far the level falls **below its starting value after
a step up in setpoint**. It grows monotonically with gain, by 8.6× across the
range: *the controller is digging its own hole*. On a minimum-phase plant a
higher gain always reduces the initial deviation; here it does the opposite,
because the controller's first action is a reaction to a movement in the wrong
direction. This is the signature of the right-half-plane zero and it has no
analogue on an ordinary process.

Sweeping the zero at constant steady-state gain (so only the zero changes):

| T_z/τ | inverse response? | effective dead time | Kc the rule allows | IAE setpoint | load peak |
|---:|---|---:|---:|---:|---:|
| −0.08 | no | 4.5 s | 6.94 | 391 | 1.30 |
| 0.10 | yes | 10.5 s | 2.98 | 668 | 3.08 |
| 0.28 | yes | 21.5 s | 1.45 | 1036 | 5.17 |
| 0.65 | yes | 43.5 s | 0.72 | 1720 | 8.09 |

A 10× reduction in usable gain and a 6× worse disturbance peak, with the plant
gain and both time constants untouched. **An RHP zero costs what dead time
costs**, for the same underlying reason — the inability to act on information
you do not yet have — which is why the half rule is right to treat them
identically. It is the non-minimum-phase problem in miniature, isolated on a
SISO loop before loop interaction is layered on top.

---

## The Smith predictor: dead-time compensation, and the price of the model

`experiments/exp10_smith_predictor.py`

A Smith predictor closes the PI around the internal model's *undelayed*
prediction and uses the measurement only to correct the model. With a perfect
model it is worth **1.66x on IAE at theta/tau = 0.25, growing to 2.0x at
theta/tau >= 1**, for 2.7x the valve travel.

| theta/tau | IAE, relative to the SIMC PI at the same ratio |
|---:|---:|
| 0.10 | 0.76x |
| 0.25 | 0.60x |
| 0.50 | 0.54x |
| 1.00 | 0.50x |
| 2.00 | 0.49x |
| 4.00 | 0.49x |

The benefit *grows* where feedback is worst and then saturates near 2x. That
shape is the result: it is what a dead-time compensator is supposed to do, and
it is not something any tuning rule can deliver.

**Where it loses, and it loses badly.** The same predictor believing theta is a
third of its true value is **7x to 13x worse than the PI it replaced**, at every
ratio, with 4585 units of valve travel against the PI's 64. The aggressive inner
tuning is protected by a cancellation that holds only if the delay is right.

Gain errors do not do this -- a 30 % error in K is a mild penalty. Delay errors
do. And theta is usually the least well identified of the three FOPDT
parameters, and drifts with throughput and fouling. The engineering conclusion
is to tune the inner loop for the delay you might actually have, not the one you
measured on a good day, and to keep `model_error` on a trend somebody looks at.

---

## MPC: what optimisation actually buys

`experiments/exp11_mpc_constraints.py`

**Unconstrained, it ties.** Tuned from the *same* FOPDT model by cited rules --
SIMC (Skogestad 2003) for the PI, Shridhar & Cooper (1997) for the MPC -- the
two land at IAE 1028.3 against 1028.7, a difference of 0.04 %. Two unrelated
rules, two different control structures, one model, one answer. That coincidence
is the strongest available evidence that both are implemented correctly.

MPC gets there with 12 % *more* valve travel, so on the trade-off it is very
slightly the worse controller. It does deliver a quarter of the PI's largest
single move and a third of the overshoot, which is worth something on real
hardware but is not what the optimiser was asked for.

The move-suppression weight R brackets the PI in both directions -- 0.65x its
IAE at R = 0.05, 1.9x at R = 100, crossing 1.0 almost exactly where the tuning
rule lands. **"MPC beat the PI on IAE" is therefore a statement about a weight
somebody chose, not about MPC.** An unconstrained MPC is a linear controller;
`LinearMPC.linear_gain()` returns which one, and the QP uses zero iterations at
every sample of that run.

**Constrained, it does what a PI cannot.** Against a 50.5 % ceiling placed where
the well-tuned PI's own 5.1 % overshoot crosses it:

| | peak | violations | violation integral | TV_u | solve time |
|---|---:|---:|---:|---:|---:|
| SIMC PI | 51.03 % | 33 | 10.93 | 55.2 | 0.003 ms |
| MPC + y_max | 50.50 % | 0 | 0.00 | 150.5 | 0.716 ms |

MPC *rides* the limit rather than backing off it. The PI is not badly tuned --
it simply has no representation of a limit it is not allowed to cross, and
nothing in a PI's structure can express one. That is the entire structural
difference, and it costs **2.7x the valve travel and ~230x the computation**.

**What this does not show.** No nonlinear plant exists in the toolbox, so the
case for nonlinear MPC has not been made and no solver framework was adopted for
it. There is no terminal cost -- the stability argument is a horizon covering
the settling time, which is the industrial DMC position. And nothing here
contradicts finding 3: peak deviation at high theta/tau is still mostly physics.

**Architecturally**, MPC arrived as one more `Controller`: `Plant` unchanged,
`simulate()` one keyword richer and one branch poorer, zero new required
dependencies. The QP is built here and solved by a dense active-set solver in
numpy and scipy; only the numerical solve sits behind a boundary where an
optional accelerator could go. That split is what keeps a suspicious result
checkable.

---

## Running summary: what the toolbox demonstrates

1. **Tuning is a frontier, not an optimum.** Ten published PI rules on one
   plant span 4x in load-disturbance IAE and 5.7x in valve travel, and they
   line up monotonically against maximum sensitivity Ms. Picking a rule is a
   decision about how much model error the plant will develop, and the toolbox
   reports both axes so the decision is visible.
2. **Structure beats tuning, repeatedly.** Anti-windup is worth 16x on recovery
   IAE, cascade 7.7x on the disturbance it is designed for, feedforward 5.4x —
   each larger than the entire spread of ten tuning rules on the same plant.
   When a loop is not performing, the question is usually which structure is
   missing, not which gain is wrong.
3. **Some of the loss is physics.** After a load step, no controller can act
   for one dead time. In the dead-time-dominant regime a SIMC PI is already
   within 2 %% of that floor on peak deviation, while the *recovery* degrades
   8x. Knowing which part of a loss is recoverable is what stops effort being
   spent where it cannot pay.
4. **The metric decides the winner.** On a surge tank, tight and averaging
   level control rank exactly opposite depending on whether level deviation or
   outflow variability is the objective — and the controller with the best
   robustness number is the one that breaches the alarm band. No single number
   is sufficient.
5. **Everything costs valve travel.** Cascade 2.1x, feedforward 5.8x,
   aggressive tuning 5.7x, Smith predictor 2.7x, MPC 2.7x. Nothing in this
   repository is reported without it.
6. **A model is leverage in both directions.** The Smith predictor is worth 2x
   where feedback is worst and 10x *against* you when theta is wrong by a
   factor of three. Everything a model-based controller gains, it gains on the
   strength of the model being right -- and dead time is the parameter most
   likely to be wrong.
7. **Optimisation buys constraints, not tracking.** A properly tuned MPC ties a
   properly tuned PI to 0.04 % when nothing binds, and its move weight spans
   the PI in both directions. What it adds is a representation of a limit it
   may not cross -- 33 violations to zero. That is a structural difference and
   it is the only one; the tracking comparison is a tuning difference and is
   available to both.

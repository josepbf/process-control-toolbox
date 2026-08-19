# FINDINGS

Running record of what won, what lost, and under what conditions the result
flipped. Numbers are reproduced by the scripts named against each section.

---

## Phase 1 — ON/OFF vs PID on a FOPDT tank

Plant: `Tank(K=1.5, tau=60 s, theta=15 s, Kd=1.0)`, valve 0–100 %, measurement
noise sigma = 0.15 %, seed 7, dt = 1 s. Dead-time ratio theta/tau = 0.25.
Scenario: setpoint 30 → 50 % at t = 100 s, unmeasured load step d = −12 at
t = 700 s. Script: `experiments/exp01_onoff_vs_pid.py`.

Both PI tunings come from published rules applied to the **true** plant
parameters — the most generous possible setting for the baselines. Model
mismatch is phase 4's job.

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
comfortably, and a tracking error already down at the noise floor. Phase 2 has
to construct a scenario with an *active output constraint* before MPC can show
anything a PI cannot already do, and phase 3 has to introduce loop interaction.
Expect PID to remain the right answer for fast SISO loops throughout.

## Phase 1 — integral windup

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

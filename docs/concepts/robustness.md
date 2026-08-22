# Robustness

Comparing tuning rules on IAE alone is exactly the trap that
[fairness rule 1](fairness.md) is written to avoid. Any rule can be made to
look good on a tracking metric by turning the gain up, and the cost of that
gain is invisible in the time-domain metrics of a *nominal* simulation. It is
visible as reduced robustness — and it gets paid later, when the process gain
drifts with tonnage, moisture or liner wear.

## Maximum sensitivity

The standard single number is

\[
M_s = \max_\omega \left| \frac{1}{1 + L(j\omega)} \right|
\]

the peak of the sensitivity function — equivalently, the inverse of the
shortest distance from the Nyquist curve of the loop transfer function
$L = GC$ to the −1 point.

Why it is preferred to gain and phase margin taken separately: **Ms bounds
both at once.**

\[
GM \ge \frac{M_s}{M_s - 1}, \qquad PM \ge 2\arcsin\!\left(\frac{1}{2M_s}\right)
\]

So $M_s = 1.4$ guarantees GM ≥ 3.5 and PM ≥ 42°.

| Ms | reading |
|---|---|
| 1.2 – 1.6 | comfortable — process practice |
| 1.6 – 2.0 | aggressive |
| > 2.0 | will oscillate the first time something about the plant changes |

## Computing it

```python
from src.tuning.analysis import pid_on_fopdt
from src.tuning.rules import simc_pi

t = simc_pi(K=1.5, tau=60.0, theta=15.0)
m = pid_on_fopdt(K=1.5, tau=60.0, theta=15.0, Kc=t.Kc, Ti=t.Ti)

m["Ms"]      # 1.590
m["GM"]      # 3.142
m["PM_deg"]  # 61.35
```

`loop_metrics()` also returns `Mt` (the complementary sensitivity peak), the
gain and phase crossover frequencies `wc` and `w180`, the frequency `w_Ms` at
which the sensitivity peaks, and the two bounds `GM_from_Ms` /
`PM_from_Ms_deg` read straight off Ms with no extra assumptions.

`pid_on_integrator()` does the same for an integrating plant.

!!! note "The derivative filter is included"
    `pid_response()` models the `Td/N` filter that `PIDController` actually
    implements. This matters: an unfiltered derivative has unbounded
    high-frequency gain, which makes Ms meaningless and flatters PID rules
    that lean on derivative action.

## The ultimate gain, analytically

`ultimate_gain_period(K, tau, theta)` solves the phase condition
$-\omega\theta - \arctan(\omega\tau) = -\pi$ for the crossover $\omega_u$, then

\[
K_u = \frac{\sqrt{1 + (\omega_u\tau)^2}}{K}, \qquad P_u = \frac{2\pi}{\omega_u}
\]

These are the two numbers the ZN and Tyreus–Luyben closed-loop rules consume.
Having them in closed form lets those rules be applied without running a relay
experiment — useful as a reference, but note that on a real plant only the
relay test can produce them, and it produces them from a describing-function
approximation, so the two will not agree exactly.
[Experiment 4](../articles/04-relay-autotune.md) measures the gap: −24 % on
$K_u$, +7 % on $P_u$.

## The counterexample: why Ms is never read alone

From [experiment 6](../articles/06-averaging-level.md):

| controller | Ms | peak level dev | alarm violations |
|---|---:|---:|---:|
| SIMC tight | 1.70 | 10.1 | 0 |
| SIMC loose | 1.17 | 24.0 | 0 |
| averaging P-only | 1.14 | 20.5 | 0 |
| lambda, integrator ignored | **1.09** | 35.4 | **890** |

The worst controller in the table has the **lowest** Ms. It earns that by
being detuned into uselessness — it spends 890 samples outside the alarm band.

Ms bounds how close a loop is to instability. It says nothing whatsoever about
whether the loop is doing its job. Robustness and performance both have to be
reported, and that row is the counterexample to reading either one alone.

## References

- Åström, K.J., Hägglund, T. *Advanced PID Control* (2006), chapter 4.
- Skogestad, S., Postlethwaite, I. *Multivariable Feedback Control* (2005),
  chapter 2.

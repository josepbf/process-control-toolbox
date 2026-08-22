# Tuning rules

Thirteen named, citable rules. [Fairness rule 1](fairness.md) says no baseline
is ever hand-tuned, so every PID in every reported comparison comes out of one
of these functions, and the rule travels with the numbers it produced.

Every rule returns a `PIDTuning` dataclass:

```python
from src.tuning.rules import simc_pi

t = simc_pi(K=1.5, tau=60.0, theta=15.0)
t.Kc, t.Ti, t.Td      # 1.333, 60.0, 0.0
t.rule                # 'SIMC PI (Skogestad 2003), tau_c=15'
t.as_kwargs()         # {'Kc': ..., 'Ti': ..., 'Td': ...} -> straight into PIDController
```

All rules are stated for the **ideal / parallel (ISA)** form that
`PIDController` implements:

\[
u = K_c \left( \beta \cdot sp - y + \frac{1}{T_i}\int e\,dt - T_d \frac{d y_f}{dt} \right)
\]

Rules published in the series (cascade) form are converted with
`series_to_ideal()`.

---

## Self-regulating processes (FOPDT)

These take the three FOPDT numbers: gain $K$, time constant $\tau$, dead time
$\theta$.

### SIMC — the project's baseline

`simc_pi(K, tau, theta, tau_c=None)`

\[
K_c = \frac{1}{K}\cdot\frac{\tau}{\tau_c + \theta}, \qquad
T_i = \min\!\left(\tau,\; 4(\tau_c + \theta)\right)
\]

$\tau_c$ is the desired closed-loop time constant and is the *only* knob;
Skogestad's recommended default is $\tau_c = \theta$, giving roughly GM ≈ 3
and PM ≈ 60°. Larger $\tau_c$ is slower and more robust.

!!! tip "The `min` is the important half of the rule"
    For a lag-dominant process the pure IMC choice $T_i = \tau$ gives
    beautiful setpoint tracking and dreadful **load disturbance** rejection,
    because the integral is far too slow. Capping $T_i$ at $4(\tau_c+\theta)$
    fixes that. Compare Lambda tuning below, which does not do this and pays
    2.3× the load IAE for it.

`simc_pid(K, tau1, tau2, theta, tau_c=None)` handles the second-order case.

### IMC

`imc_pi(K, tau, theta, tau_c=None)` / `imc_pid(...)`

\[
T_i = \tau + \frac{\theta}{2}, \qquad K_c = \frac{1}{K}\cdot\frac{T_i}{\tau_c + \theta/2}
\]

Uses a first-order Padé approximation of the delay. Keeps $T_i$ at the
Padé-corrected process lag — clean setpoint tracking, slow load rejection on
lag-dominant processes.

### Lambda (Dahlin) — the DCS default

`lambda_tuning(K, tau, theta, lam=None)`

\[
K_c = \frac{\tau}{K(\lambda + \theta)}, \qquad T_i = \tau
\]

Plant practice is $\lambda = \tau$ for a "safe" loop and $\lambda = 3\theta$
for a faster one. Included precisely because it is so widely deployed in pulp,
paper and cement — and because [experiment 3](../articles/03-tuning-shootout.md)
shows the price it pays for cancelling the lag exactly.

### AMIGO — the modern robustness-constrained rule

`amigo_pi(K, tau, theta)` / `amigo_pid(...)`

Fitted by Åström & Hägglund (2004) to a large batch of representative process
models under an explicit robustness constraint, $M_s \le 1.4$, rather than to
a decay-ratio taste. **The fairest single "modern classical" baseline
available**, and the project's robust alternative to SIMC.

### Ziegler–Nichols, open loop

`ziegler_nichols_open_loop(K, tau, theta, kind="PI")`

The 1942 reaction-curve rules, targeting quarter-amplitude decay. Included as
a deliberately aggressive reference point: ZN is famously oscillatory and is
**not** a fair "best classical" baseline — but it is the rule most people
picture when they hear "tuned PID", so it is worth having in the table.

### Cohen–Coon

`cohen_coon(K, tau, theta, kind="PI")`

Also 1953-vintage quarter-amplitude decay, but derived for processes where the
dead time is a large fraction of the lag, where ZN becomes wildly aggressive.
Gentler than ZN at large $\theta/\tau$, comparable at small — and still
oscillatory by modern standards, because quarter-amplitude decay is a taste,
not a robustness specification.

---

## Closed-loop rules (ultimate gain and period)

These take $K_u$ and $P_u$: the proportional gain at which the loop oscillates
with constant amplitude, and the period of that oscillation. Get them from
[relay auto-tuning](../tutorials/03-relay-autotuning.md), or analytically with
`ultimate_gain_period()`.

### Ziegler–Nichols, closed loop

`ziegler_nichols_closed_loop(Ku, Pu, kind="PI")` → $K_c = 0.45K_u$, $T_i = P_u/1.2$

### Tyreus–Luyben

`tyreus_luyben(Ku, Pu, kind="PI")` → $K_c = K_u/3.2$, $T_i = 2.2 P_u$

The same experiment, a chemical engineer's taste: roughly **half the gain and
twice the integral time** of ZN, because quarter-amplitude decay is
unacceptable on real process plant where loops are coupled and the model
drifts with operating point. This is the rule most industrial autotuners
actually ship.

---

## Integrating processes

A surge tank, a drum level, a silo. There is no $\tau$ to cancel — the output
does not come back on its own — so the FOPDT rules do not apply. Applying one
anyway sets the integral time from a lag that does not exist, and produces the
slow rolling oscillation seen in badly tuned level loops in every plant.

### SIMC for integrators

`simc_integrating(k_prime, theta, tau_c=None)`

\[
K_c = \frac{1}{k'(\tau_c + \theta)}, \qquad T_i = 4(\tau_c + \theta)
\]

### Averaging level control

`averaging_level_pi(k_prime, v_max, y_max_dev)` → P-only,
$K_c = \operatorname{sign}(k') \cdot v_{max}/y_{max,dev}$

Not a detuned version of tight control — a **different objective**. A surge
tank exists to absorb flow variation so the downstream unit sees a steady
feed; a controller that holds level perfectly passes every inlet upset
straight through and defeats the equipment it is installed on. The right
design is the loosest proportional gain that still keeps the level inside its
alarm band for the largest expected upset.

[Experiment 6](../articles/06-averaging-level.md) shows the two rankings
coming out **opposite**.

---

## Model reduction

### The half rule

`half_rule(taus, theta=0.0, inverse_zeros=None, dt=0.0)`

Reduces a high-order model to FOPDT so a two-parameter rule can be applied
honestly. See [Dead time](dead-time.md#everything-a-pid-cannot-represent-becomes-dead-time)
for the formula and why RHP zeros belong in it.

---

## Where they land

All ten PI rules on the same plant ($K = 1.5$, $\tau = 60$, $\theta = 15$),
sorted by robustness — the full analysis is
[experiment 3](../articles/03-tuning-shootout.md):

| rule | Kc | Ti | Ms | IAE setpoint | IAE load | TV(u) |
|---|---:|---:|---:|---:|---:|---:|
| Lambda (λ=τ) | 0.53 | 60.0 | 1.19 | 1519 | 935 | 116 |
| AMIGO | 0.61 | 49.2 | 1.25 | 1209 | 704 | 132 |
| Lambda (λ=3θ) | 0.67 | 60.0 | 1.25 | 1229 | 761 | 144 |
| SIMC (τ_c=2θ) | 0.89 | 60.0 | 1.35 | 942 | 587 | 193 |
| Tyreus–Luyben | 1.44 | 120.9 | 1.58 | 1096 | 689 | 321 |
| **SIMC (τ_c=θ)** | **1.33** | **60.0** | **1.59** | **715** | **411** | **297** |
| IMC | 2.00 | 67.5 | 2.09 | 717 | 321 | 465 |
| ZN closed-loop | 2.08 | 45.8 | 2.37 | 837 | 244 | 499 |
| ZN open-loop | 2.40 | 50.0 | 2.78 | 902 | 234 | 593 |
| Cohen–Coon | 2.46 | 33.0 | 3.66 | 1266 | 247 | 663 |

A 4.6× spread in gain between rules that are all, individually, "a named
published tuning rule".

## References

- Skogestad, S. (2003). "Simple analytic rules for model reduction and PID
  controller tuning." *Journal of Process Control* 13(4), 291–309. **[SIMC]**
- Rivera, D.E., Morari, M., Skogestad, S. (1986). "Internal model control: PID
  controller design." *Ind. Eng. Chem. Process Des. Dev.* 25(1), 252–265. **[IMC]**
- Ziegler, J.G., Nichols, N.B. (1942). "Optimum settings for automatic
  controllers." *Trans. ASME* 64, 759–768. **[ZN]**
- Cohen, G.H., Coon, G.A. (1953). "Theoretical consideration of retarded
  control." *Trans. ASME* 75, 827–834.
- Tyreus, B.D., Luyben, W.L. (1992). "Tuning PI controllers for
  integrator/dead time processes." *Ind. Eng. Chem. Res.* 31(11), 2625–2628.
- Åström, K.J., Hägglund, T. (2004). "Revisiting the Ziegler–Nichols step
  response method for PID control." *Journal of Process Control* 14(6),
  635–650. **[AMIGO]**

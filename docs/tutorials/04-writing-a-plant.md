# Tutorial 4: writing a plant

A `Plant` subclass supplies two things: continuous-time dynamics, and an
output map. Integration, saturation, three independent delay lines and
measurement noise are handled once in the base class, so a new process is
usually 30 lines.

## The contract

```python
def dynamics(self, x, u, d) -> np.ndarray:   # xdot = f(x, u, d)   -- required
def output(self, x) -> np.ndarray:           # y = g(x)            -- optional
```

!!! important "`u` is not the controller's output"
    The `u` handed to `dynamics()` is the **delayed, saturated** input that
    actually reaches the process. The base class has already clipped it to
    `[u_min, u_max]` and pushed it through the transport-delay FIFO. Do not
    saturate or delay again inside `dynamics()` — and do not skip declaring
    the limits in the hope that a controller will respect them, because
    [that is the plant's job](../concepts/fairness.md), not the controller's.

## Worked example: a steam-jacketed vessel

Two lags in series with a transport delay on the steam line. The jacket is
fast, the contents are slow.

\[
\begin{aligned}
\tau_j \frac{dT_j}{dt} &= -T_j + K_j\, u(t - \theta) \\
\tau_c \frac{dT_c}{dt} &= -T_c + T_j + K_d\, d(t - \theta_d) \\
y &= T_c
\end{aligned}
\]

### 1. Declare the shape in `__init__`

```python
class JacketedVessel(Plant):
    def __init__(self, K_j=0.8, tau_j=25.0, tau_c=180.0, theta=8.0,
                 Kd=1.0, theta_d=0.0, T0=40.0, u_min=0.0, u_max=100.0,
                 noise_std=0.0, y_min=None, y_max=None, seed=0):
        self.K_j, self.tau_j, self.tau_c = float(K_j), float(tau_j), float(tau_c)
        self.theta, self.Kd = float(theta), float(Kd)

        super().__init__(
            n_states=2, n_inputs=1, n_outputs=1,
            u_min=u_min, u_max=u_max,
            x0=[T0, T0],                 # both temperatures start at T0
            dead_time=theta,             # manipulated path
            d_dead_time=theta_d,         # disturbance path
            noise_std=noise_std,
            y_min=y_min, y_max=y_max,    # reporting-only band
            seed=seed,
        )
        self.u_init = self.saturate(np.array([T0 / self.K_j]))
```

Set your own attributes **before** calling `super().__init__()` — the base
class calls `output(self.x)` while priming the measurement delay line, so
anything `output()` needs must already exist.

!!! tip "`u_init` is easy to forget and always bites"
    It is the value the input delay line is primed with. Leave it at the
    default of zero and a plant with a non-zero holding valve position starts
    every run with a hidden step already in the pipeline: the first `theta`
    seconds of every trace are an artefact. Set it to the input that holds the
    initial state.

### 2. Write the dynamics

```python
    def dynamics(self, x, u, d):
        T_j, T_c = x
        return np.array([
            (-T_j + self.K_j * u[0]) / self.tau_j,
            (-T_c + T_j + self.Kd * d[0]) / self.tau_c,
        ])

    def output(self, x):
        return np.array([x[1]])          # only the contents are measured
```

`dynamics()` is called four times per sub-step by the RK4 integrator, and
`n_substeps` defaults to 4 — so sixteen times per sample. Keep it pure: no
state mutation, no I/O, no RNG. All randomness belongs to the plant's own
`self.rng`, which the base class re-seeds on `reset()`.

### 3. Add the properties a tuning rule needs

This is the part that makes a plant *usable* rather than merely simulatable.
A classical rule needs FOPDT parameters, and this process is second order —
so hand it the half-rule reduction, and be explicit that that is what you are
doing.

```python
    def steady_input(self, y_target: float) -> float:
        return y_target / self.K_j

    @property
    def fopdt(self) -> dict:
        """What a two-parameter tuning rule is actually given, via the half rule."""
        tau_eff, theta_eff = half_rule([self.tau_c, self.tau_j], theta=self.theta)
        return {"K": self.K_j, "tau": tau_eff, "theta": theta_eff}
```

```pycon
>>> JacketedVessel().fopdt
{'K': 0.8, 'tau': 192.5, 'theta': 20.5}
```

The 25 s jacket lag has become 12.5 s of extra time constant and 12.5 s of
extra dead time. That is the half rule doing exactly what it says: everything
the two-parameter model cannot represent turns into effective dead time. See
[Dead time](../concepts/dead-time.md#everything-a-pid-cannot-represent-becomes-dead-time).

### 4. Use it

Nothing else changes — the new plant drops straight into the harness.

```python
tuning = simc_pi(**vessel.fopdt)
# SIMC PI (Skogestad 2003), tau_c=20.5: Kc=5.869, Ti=164.0
# Ms = 1.604
```

```text
                             IAE  settling_2pct_s  overshoot_pct  peak_dev  ss_offset      TV_u  reversals
controller window
PI (SIMC)  full         2259.158         1660.000          8.766    20.152      0.014  1536.100        919
           setpoint     1923.650          265.000          8.720    20.152      0.007   695.137        421
           disturbance   311.510          288.000            NaN     1.912      0.011   670.196        414
```

Ms lands at 1.60 — right where SIMC is supposed to put it — so the reduction
was honest. The reversal count is high because $K_c = 5.9$ against a
measurement carrying 0.1 °C of noise; that is the effort column doing its job,
and it is an argument for either filtering the measurement or moving to
[cascade control](../articles/07-cascade.md) on the jacket temperature.

## Checklist for a new plant

- [ ] Attributes set **before** `super().__init__()`
- [ ] `n_states`, `n_inputs`, `n_outputs`, `n_disturbances` all declared
- [ ] `u_min` / `u_max` are the **real** actuator limits
- [ ] `u_init` primes the delay line at the holding input
- [ ] `dynamics()` is pure and takes the *already delayed and saturated* `u`
- [ ] `output()` returns only what is actually measured
- [ ] `steady_input()` implemented, so experiments can start bumplessly
- [ ] A property exposing the model a tuning rule needs, with a docstring
      saying it is the *true* model
- [ ] A test: check the steady-state gain, check the delay shows up where it
      should

## The five plants already in the project

| plant | what it is for |
|---|---|
| [`Tank`](../reference/plants.md#tank) | FOPDT workhorse; separate disturbance path |
| [`IntegratingTank`](../reference/plants.md#integratingtank) | surge tank — no self-regulation |
| [`SeriesTanks`](../reference/plants.md#seriestanks) | N lags — the half rule's honest test |
| [`InverseResponseTank`](../reference/plants.md#inverseresponsetank) | drum level — RHP zero |
| [`CascadeProcess`](../reference/plants.md#cascadeprocess) | fast inner stage feeding a slow outer stage |

## The complete script

```python
--8<-- "docs/snippets/tutorial04.py"
```

## Next

[Tutorial 5: writing a controller](05-writing-a-controller.md).

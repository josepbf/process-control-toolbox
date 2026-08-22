# Metrics

Every reported comparison carries a **tracking** metric and a **control-effort**
metric. That is the project's rule, and it is why an ON/OFF controller can look
merely mediocre on IAE while being obviously unusable the moment you look at
the reversal count.

`compute_metrics(df, window)` returns all of them for one run over one time
window. `summarize(runs, windows)` builds the full table.

## Tracking

| metric | definition | when to read it |
|---|---|---|
| `IAE` | $\int \lvert e \rvert \, dt$ | the default "how well did it track" number |
| `ISE` | $\int e^2 dt$ | punishes large excursions harder than IAE |
| `ITAE` | $\int t\,\lvert e \rvert \, dt$ | punishes long tails — dislikes sluggish settling and offset |
| `peak_dev` | $\max \lvert y - sp \rvert$ | the right headline for a **load disturbance** |
| `overshoot_pct` | peak beyond setpoint, as % of the step | the right headline for a **setpoint change** |
| `settling_2pct_s` | time until $\lvert y - sp\rvert$ stays inside the band | how long the transient lasted |
| `ss_offset` | mean residual error over the last 10 % of the window | catches a missing or crippled integrator |

### The three that need care

Three of these deliberately return `NaN` rather than a comforting number, and
understanding when is the difference between reading the table and being
misled by it.

=== "`settling_2pct_s`"

    The band is built in two steps.

    **Scale.** The nominal band is 2 % of *the excursion being settled* — the
    setpoint step where there is one, otherwise the largest deviation in the
    window. That generalisation is what lets the same metric describe a load
    recovery, where the setpoint never moves and a band defined as a fraction
    of the step would be zero.

    **Noise floor.** A "last sample outside the band" statistic is fragile if
    the band sits at a fixed few sigma of measurement noise: a clean Gaussian
    sequence of length $n$ wanders out to about $\sqrt{2\ln n}$ sigma by
    chance, so on a run of a few thousand samples a perfectly settled loop
    would be reported as never settling. The band is raised to that expected
    extreme plus a sigma of margin.

    Settling is declared only if the loop then *stays* in the band for at
    least 5 % of the window. Without that dwell requirement, a slow
    oscillation gets a settling time whenever its last sample happens to land
    near the setpoint — an artefact of where the run was cut off.

    **`NaN` means never settled**, and for a limit-cycling ON/OFF controller
    that is the honest answer.

    !!! warning "Known limitation"
        The noise estimator works on one-sample differences, so an oscillation
        whose period approaches a few samples is indistinguishable from white
        noise and gets absorbed into the band. Real limit cycles on a lagged
        process are many samples long and are caught correctly. A settling
        time reported for a near-Nyquist oscillation should not be trusted.

=== "`overshoot_pct`"

    Returns `NaN` when the window contains no meaningful setpoint change — the
    test being that the step is small compared with the excursions actually
    seen. Overshoot of a step that never happened is not a number worth
    printing. For a load-disturbance window, read `peak_dev` instead.

    This is why the disturbance rows in every results table have an empty
    overshoot column.

=== "`noise_sigma`"

    A robust estimate of the measurement noise σ, from the median absolute
    deviation of one-sample differences. Real process motion is smooth and
    contributes only a few large differences while noise contributes many
    small ones, so the *median* is dominated by noise even when the window
    contains a transient. The 1.4826 factor converts MAD to σ for Gaussian
    data; the $\sqrt{2}$ removes the variance doubling introduced by
    differencing.

## Control effort

| metric | definition | why it is here |
|---|---|---|
| `TV_u` | $\sum \lvert \Delta u \rvert$ | total distance the actuator travelled |
| `max_du` | $\max \lvert \Delta u \rvert$ | largest single-sample move |
| `reversals` | number of direction changes | **valve wear tracks reversals more closely than travel** |

`reversals` is the metric that makes ON/OFF limit cycling impossible to hide.
Moves smaller than `eps` are ignored — on a noisy measurement any proportional
controller dithers by a fraction of a percent every sample, and counting that
as wear would swamp the comparison with something no plant operator would ever
notice. `compute_metrics` sets `eps` to **0.5 % of the actuator span**.

!!! quote "The effort column in one line"
    Phase 1 record: cascade cost 2.1× the valve travel, feedforward 5.8×,
    aggressive tuning 5.7×. Every single improvement was paid for.

## Constraint

| metric | meaning |
|---|---|
| `n_violations` | samples with the output outside its declared band |
| `violation_integral` | $\int$ magnitude outside the band $dt$ |

Output limits (`y_min`, `y_max`) are **reporting-only** in phase 1 — the plant
does not enforce them. They exist so the harness can count how often a
controller drives the process past a limit it was supposed to respect. This is
the column MPC is eventually going to be judged on, so it is instrumented from
the start.

[Experiment 6](../articles/06-averaging-level.md) shows why it matters: the
controller with the *best* robustness number in that table spends 890 samples
outside the alarm band.

## Computation

| metric | meaning |
|---|---|
| `solve_ms_mean` | mean wall-clock time inside `compute()` |
| `solve_ms_p95` | 95th percentile of the same |

Logged for every controller, trivial ones included. See
[fairness rule 5](fairness.md#5-computational-cost-appears-in-the-comparison-table).

## Windows

Metrics are almost always computed per **window**, not over the whole run. A
`Scenario` declares them:

```python
windows={
    "setpoint":    (t_step, t_load),
    "disturbance": (t_load, t_final),
}
```

Tracking a setpoint and rejecting a load are the two jobs a regulatory loop
has, and they are in tension — tuning that tracks elegantly is often sluggish
against load upsets. (This is exactly what the `min` in the SIMC `Ti` rule is
there to fix.) Scoring them in one lump hides the trade.

```python
table = summarize(runs, windows=scenario.windows)   # adds a "full" window too
print(format_table(table))
```

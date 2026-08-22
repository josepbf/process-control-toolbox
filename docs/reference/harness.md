# Harness

One closed loop, used by every controller and every plant in the project.
Keeping this single-sourced is what makes the comparisons trustworthy: nobody
gets a different integrator, a different sample time, or a different noise
stream.

---

## Simulation

::: src.harness.simulate.simulate

::: src.harness.simulate.run_all

---

## Scenarios

::: src.harness.scenarios.Scenario

::: src.harness.scenarios.setpoint_and_load

### Signal builders

::: src.harness.scenarios.constant

::: src.harness.scenarios.staircase

::: src.harness.scenarios.pulse

---

## Metrics

See [Concepts → Metrics](../concepts/metrics.md) for what each number means
and when it deliberately returns `NaN`.

::: src.harness.metrics.compute_metrics

::: src.harness.metrics.summarize

::: src.harness.metrics.format_table

### Tracking

::: src.harness.metrics.iae

::: src.harness.metrics.ise

::: src.harness.metrics.itae

::: src.harness.metrics.settling_time

::: src.harness.metrics.overshoot

::: src.harness.metrics.peak_deviation

::: src.harness.metrics.steady_state_offset

::: src.harness.metrics.noise_sigma

### Control effort

::: src.harness.metrics.total_variation

::: src.harness.metrics.max_move

::: src.harness.metrics.reversals

### Signal access

::: src.harness.metrics.signal

---

## Plotting

::: src.harness.plotting.plot_runs

::: src.harness.plotting.plot_tradeoff

::: src.harness.plotting.plot_sweep

::: src.harness.plotting.save_table

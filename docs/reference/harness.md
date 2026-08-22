# Harness

One closed loop, used by every controller and every plant in the project.
Keeping this single-sourced is what makes the comparisons trustworthy: nobody
gets a different integrator, a different sample time, or a different noise
stream.

---

## Simulation

::: process_control.harness.simulate.simulate

::: process_control.harness.simulate.run_all

---

## Scenarios

::: process_control.harness.scenarios.Scenario

::: process_control.harness.scenarios.setpoint_and_load

### Signal builders

::: process_control.harness.scenarios.constant

::: process_control.harness.scenarios.staircase

::: process_control.harness.scenarios.pulse

---

## Metrics

See [Concepts → Metrics](../concepts/metrics.md) for what each number means
and when it deliberately returns `NaN`.

::: process_control.harness.metrics.compute_metrics

::: process_control.harness.metrics.summarize

::: process_control.harness.metrics.format_table

### Tracking

::: process_control.harness.metrics.iae

::: process_control.harness.metrics.ise

::: process_control.harness.metrics.itae

::: process_control.harness.metrics.settling_time

::: process_control.harness.metrics.overshoot

::: process_control.harness.metrics.peak_deviation

::: process_control.harness.metrics.steady_state_offset

::: process_control.harness.metrics.noise_sigma

### Control effort

::: process_control.harness.metrics.total_variation

::: process_control.harness.metrics.max_move

::: process_control.harness.metrics.reversals

### Signal access

::: process_control.harness.metrics.signal

---

## Plotting

::: process_control.harness.plotting.plot_runs

::: process_control.harness.plotting.plot_tradeoff

::: process_control.harness.plotting.plot_sweep

::: process_control.harness.plotting.save_table

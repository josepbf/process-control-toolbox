# Tuning

Named, citable tuning rules; frequency-domain robustness analysis; and the
relay auto-tuning experiment. See
[Concepts → Tuning rules](../concepts/tuning-rules.md) for the formulas side
by side and [Concepts → Robustness](../concepts/robustness.md) for what Ms
means.

---

## The tuning record

::: process_control.tuning.rules.PIDTuning

---

## Rules for self-regulating processes

::: process_control.tuning.rules.simc_pi

::: process_control.tuning.rules.simc_pid

::: process_control.tuning.rules.imc_pi

::: process_control.tuning.rules.imc_pid

::: process_control.tuning.rules.lambda_tuning

::: process_control.tuning.rules.amigo_pi

::: process_control.tuning.rules.amigo_pid

::: process_control.tuning.rules.ziegler_nichols_open_loop

::: process_control.tuning.rules.cohen_coon

---

## Rules from the ultimate gain and period

::: process_control.tuning.rules.ziegler_nichols_closed_loop

::: process_control.tuning.rules.tyreus_luyben

---

## Rules for integrating processes

::: process_control.tuning.rules.simc_integrating

::: process_control.tuning.rules.averaging_level_pi

---

## Model reduction and form conversion

::: process_control.tuning.rules.half_rule

::: process_control.tuning.rules.series_to_ideal

---

## Robustness analysis

::: process_control.tuning.analysis.pid_on_fopdt

::: process_control.tuning.analysis.pid_on_integrator

::: process_control.tuning.analysis.loop_metrics

::: process_control.tuning.analysis.ultimate_gain_period

### Frequency responses

::: process_control.tuning.analysis.fopdt_response

::: process_control.tuning.analysis.integrator_response

::: process_control.tuning.analysis.pid_response

---

## Relay auto-tuning

::: process_control.tuning.relay.relay_autotune

::: process_control.tuning.relay.RelayResult

---

## MPC tuning

Fairness rule 1 applies to model predictive control exactly as it applies to a
PI, and an MPC has more knobs than a PI rather than fewer.

::: process_control.tuning.mpc_rules.MPCTuning

::: process_control.tuning.mpc_rules.shridhar_cooper

::: process_control.tuning.mpc_rules.settling_horizon

# Tuning

Named, citable tuning rules; frequency-domain robustness analysis; and the
relay auto-tuning experiment. See
[Concepts → Tuning rules](../concepts/tuning-rules.md) for the formulas side
by side and [Concepts → Robustness](../concepts/robustness.md) for what Ms
means.

---

## The tuning record

::: src.tuning.rules.PIDTuning

---

## Rules for self-regulating processes

::: src.tuning.rules.simc_pi

::: src.tuning.rules.simc_pid

::: src.tuning.rules.imc_pi

::: src.tuning.rules.imc_pid

::: src.tuning.rules.lambda_tuning

::: src.tuning.rules.amigo_pi

::: src.tuning.rules.amigo_pid

::: src.tuning.rules.ziegler_nichols_open_loop

::: src.tuning.rules.cohen_coon

---

## Rules from the ultimate gain and period

::: src.tuning.rules.ziegler_nichols_closed_loop

::: src.tuning.rules.tyreus_luyben

---

## Rules for integrating processes

::: src.tuning.rules.simc_integrating

::: src.tuning.rules.averaging_level_pi

---

## Model reduction and form conversion

::: src.tuning.rules.half_rule

::: src.tuning.rules.series_to_ideal

---

## Robustness analysis

::: src.tuning.analysis.pid_on_fopdt

::: src.tuning.analysis.pid_on_integrator

::: src.tuning.analysis.loop_metrics

::: src.tuning.analysis.ultimate_gain_period

### Frequency responses

::: src.tuning.analysis.fopdt_response

::: src.tuning.analysis.integrator_response

::: src.tuning.analysis.pid_response

---

## Relay auto-tuning

::: src.tuning.relay.relay_autotune

::: src.tuning.relay.RelayResult

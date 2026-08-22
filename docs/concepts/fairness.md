# Fairness rules

Comparing control strategies is one of the main things this toolbox gets used
for, and it is trivially easy to do dishonestly — tune one contender badly,
give another information the rest do not have, report tracking error and stay
quiet about valve travel.

The defence is to fix the rules in advance, write them down, and build the
enforcement into the code rather than into good intentions. Five rules, each
with a specific mechanism behind it.

---

## 1. Every baseline is tuned by a named, cited rule

No silent hand-tuning, ever. Every PID in every reported comparison comes out
of a function in [`process_control/tuning/rules.py`](../reference/tuning.md), and the rule
— with its citation and its tuning knob — is recorded in the results table
with the run.

**Mechanism.** `Controller.tuning_note` is a required attribute, defaulting to
the deliberately embarrassing string `"unspecified"`. Every rule returns a
`PIDTuning` dataclass carrying a `rule` field, and the experiment scripts pass
it straight through:

```python
tuning = simc_pi(**plant.fopdt)
pi = PIDController(**tuning.as_kwargs(), ..., tuning_note=tuning.rule)
```

`df.attrs["tuning"]` then travels with the numbers, into the CSV.

The default baseline was chosen on stated grounds in
[experiment 3](../articles/03-tuning-shootout.md): **SIMC with
τ<sub>c</sub> = θ**, with AMIGO as the robust alternative. Both appear in later
comparisons, rather than one being quietly selected after the fact.

!!! warning "Why this rule is not paranoia"
    The ten published rules tested in experiment 3 disagree with each other by
    a factor of **4.6× in gain** (Kc from 0.53 to 2.46) on the same plant.
    "We compared against a tuned PID" is not, by itself, a statement that
    means anything.

---

## 2. Identical plant, sample time, seed, disturbance and limits

Every controller in a comparison is run against the *same* `Scenario` object
and the same RNG seed.

**Mechanism.** `run_all()` takes one plant and one scenario and iterates over
controllers; `simulate()` deep-copies the plant so no run can contaminate the
next; `Plant.reset()` rebuilds the RNG from the seed. Feedforward controllers
that read a disturbance measurement get their measurement noise from a
*separate* RNG stream (`seed + 104729`), so switching feedforward on or off
cannot change the noise realisation seen on `y`.

---

## 3. A controller's internal model is never the exact plant

Unless explicitly labelled as an idealised upper bound.

**Mechanism.** The plant exposes its true parameters through properties —
`Tank.fopdt`, `CascadeProcess.inner_fopdt`, and so on — and their docstrings
say what they are:

> This is the *true* model; a controller is only entitled to it when we
> deliberately grant a no-mismatch upper bound.

The current experiments grant it everywhere, which is the **most generous
possible setting for the baselines**, and every experiment header says so.
Deliberate model mismatch is a separate study, for when identification lands.

There is a preview of the honest version already, in
[experiment 8](../articles/08-feedforward.md): the feedforward compensator is
run with a 30 % error in its gain, and gives back half its benefit.

---

## 4. Scenarios where the sophisticated method loses get reported

Fast SISO loops with no active constraint are expected to favour PID, and
saying so up front is what makes the wins credible.

**Mechanism.** Editorial, but consistently applied and already visible:

- [Experiment 1](../articles/01-onoff-vs-pid.md) closes with *"on this loop
  there is nothing for a more sophisticated controller to do"*.
- [Experiment 7](../articles/07-cascade.md) reports cascade at **0.9×** —
  worse — on the disturbance it was not designed for, in the same table as the
  7.7× win.
- [Experiment 8](../articles/08-feedforward.md) reports the unrealisable case,
  where the textbook design delivers 1.4× instead of 5.4×.

---

## 5. Computational cost appears in the comparison table

`solve_ms_mean` and `solve_ms_p95` are logged for every controller, including
the trivial ones.

**Mechanism.** `simulate()` wraps every `compute()` call in `perf_counter()`
and stores the result in the `solve_time` column; `compute_metrics()` reduces
it to a mean and a 95th percentile. An ON/OFF controller comes out around
1 µs, a PID around 3 µs. The column looks pointless today — that is
precisely why it is there from the start: when an optimisation-based
controller's solve time eventually lands in the same column, nobody has to
argue about whether to include it.

---

## The sixth rule, which is really about honesty

> Every reported comparison carries a tracking metric **and** a control-effort
> metric.

Tracking bought with violent actuator movement is not a win — in a real plant
it destroys valves. `compute_metrics()` always returns `TV_u`, `max_du` and
`reversals` alongside IAE, and the standard figure always draws the
manipulated variable underneath the controlled one.

The record on this point is unambiguous: **every improvement so far cost valve
travel.** Cascade 2.1×, feedforward 5.8×, aggressive tuning 5.7×.

See [Metrics](metrics.md) for what each of those numbers means.

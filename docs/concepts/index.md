# Concepts

The explanation half of the documentation: not *how to call* the code, but why
it is shaped the way it is, and what the ideas behind it mean.

<div class="grid cards" markdown>

-   __[Architecture](architecture.md)__

    The plant / controller / harness split, the timing convention of a sample,
    and the schema of the DataFrame every run produces.

-   __[Fairness rules](fairness.md)__

    Five rules that decide what counts as a legitimate comparison, and the
    specific mechanism in the code that enforces each one.

-   __[Dead time](dead-time.md)__

    Why θ/τ is the single best predictor of how hard a loop is, the three
    independent delay paths the plant models, and the physical floor no
    controller can beat.

-   __[Metrics](metrics.md)__

    What every number in the results table means, how it is computed, and the
    cases where it deliberately returns `NaN` instead of a comforting figure.

-   __[Tuning rules](tuning-rules.md)__

    Thirteen published rules, their formulas, the process class each was
    designed for, and where each one lands on the robustness scale.

-   __[Robustness](robustness.md)__

    Maximum sensitivity Ms, what it bounds, and the counterexample that shows
    why it must never be read on its own.

</div>

# Tutorials

Six hands-on walkthroughs, in order. Each one is a complete, runnable script
built up a few lines at a time, with the output shown so you can check you are
in the same place.

<div class="grid cards" markdown>

-   __[1. Your first loop](01-your-first-loop.md)__

    Build a plant, tune a PI, run it, read the metrics table, draw the figure.
    The 20 lines everything else in the project is made of.

-   __[2. Tuning a loop](02-tuning-a-loop.md)__

    From a step test to FOPDT parameters to a tuning rule to a robustness
    check — and how to compare several rules on the same loop.

-   __[3. Relay auto-tuning](03-relay-autotuning.md)__

    Identify the ultimate gain and period without a model, the way an
    industrial autotune button does, and see how far off it is.

-   __[4. Writing a plant](04-writing-a-plant.md)__

    Subclass `Plant` with a worked example: a jacketed vessel with two states,
    a delay and a load path.

-   __[5. Writing a controller](05-writing-a-controller.md)__

    Subclass `Controller` with a worked example: a valve slew-rate limiter
    that wraps any other control law.

-   __[6. Designing an experiment](06-designing-an-experiment.md)__

    Parameter sweeps, trade-off frontiers, and the conventions an experiment
    script in this repository follows.

</div>

!!! note "Run these from the repository root"
    The `src` package is imported as `src.plants.tank` and so on, so scripts
    expect the repository root on `sys.path`. The experiment scripts handle
    this themselves with a two-line preamble; if you are working in a REPL,
    start it from the root.

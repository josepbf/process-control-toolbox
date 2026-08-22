# Articles

One article per experiment. Each is a narrative walkthrough of a reproducible
result: what was set up, what came out, what it means, and — where it applies
— what it implies for the phases that have not happened yet.

Every number on these pages comes from a script in `experiments/` that you can
run yourself in a few seconds. The condensed running record is in
[`FINDINGS.md`](https://github.com/josepbf/process-control-toolbox/blob/main/FINDINGS.md).

<div class="grid cards" markdown>

-   __[1. ON/OFF vs PID](01-onoff-vs-pid.md)__

    The baseline everything else has to beat, and why a limit cycle is a
    structure rather than a tuning fault.

    `exp01_onoff_vs_pid.py`

-   __[2. Integral windup](02-integral-windup.md)__

    16× on recovery IAE from a structural fix, with the tuning held identical.

    `exp02_antiwindup.py`

-   __[3. The tuning shootout](03-tuning-shootout.md)__

    Ten published rules, scored on performance *and* robustness. Where the
    project's PID baseline was chosen, and on what stated grounds.

    `exp03_tuning_shootout.py`

-   __[4. Relay auto-tuning](04-relay-autotune.md)__

    How good is the autotune button? Period to +7 %, gain 24 % low — and it
    errs safe.

    `exp04_relay_autotune.py`

-   __[5. The dead-time sweep](05-dead-time-sweep.md)__

    Where feedback runs out of road, measured against a physical floor no
    controller can beat.

    `exp05_deadtime_sweep.py`

-   __[6. Averaging level control](06-averaging-level.md)__

    Two rankings, exactly opposite, on the same four controllers. The metric
    decides the winner.

    `exp06_averaging_level.py`

-   __[7. Cascade control](07-cascade.md)__

    Buying information rather than gain: 7.7× on one disturbance, 0.9× on
    another, 2.1× the valve travel throughout.

    `exp07_cascade.py`

-   __[8. Feedforward](08-feedforward.md)__

    The cheapest possible prediction — 5.4× on IAE — and the three
    qualifications that apply directly to MPC.

    `exp08_feedforward.py`

-   __[9. Inverse response](09-inverse-response.md)__

    A right-half-plane zero, where more gain digs the hole deeper. An RHP zero
    costs what dead time costs.

    `exp09_inverse_response.py`

</div>

## The through-line

Read in order, the nine articles build one argument:

1. A well-implemented PI is already very good on an easy loop
   ([1](01-onoff-vs-pid.md)), and *implementation* — not tuning — is what
   separates usable from unusable ([2](02-integral-windup.md)).
2. "Tuned PID" is not one thing; the published rules span 4.6× in gain, and
   choosing among them is a robustness decision ([3](03-tuning-shootout.md)),
   which you can make without a model at all if you have to
   ([4](04-relay-autotune.md)).
3. What makes loops hard is dead time and its relatives, and there is a
   physical floor that no controller reaches past
   ([5](05-dead-time-sweep.md), [9](09-inverse-response.md)).
4. Before any of that, someone has to say what the loop is *for*
   ([6](06-averaging-level.md)).
5. And the largest wins available in phase 1 come from **structure** —
   an extra measurement ([7](07-cascade.md)), an instrumented disturbance
   ([8](08-feedforward.md)) — not from tuning.

Which sets up the question phase 2 onward has to answer: not *can MPC beat a
PID*, but **can MPC beat a well-structured classical scheme**.

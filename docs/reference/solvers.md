# Solvers

The one place this toolbox calls out to somebody else's numerics.

Building an MPC's prediction matrices, disturbance model and constraint rows is
toolbox code, deliberately — that is what has to be read when a result looks too
good, and a controller you cannot open is a controller you cannot check. But
nobody learns anything from the iteration bookkeeping inside a QP solver, and a
good one is real numerical-analysis work whose failure modes have nothing to do
with process control. So the *numerical solve* is delegated, behind one
function.

$$
\min_z \; \tfrac12 z^\top H z + g^\top z
\quad\text{s.t.}\quad
lb \le A z \le ub, \qquad z_{lb} \le z \le z_{ub}
$$

which is OSQP's form and maps to `qpsolvers` and cvxpy without loss, so the
adaptation cost lands on our own backend rather than on the caller.

## Two rules, because this is a benchmark and not a product

**`"auto"` resolves to the dependency-free solver, always. It never probes.**
A controller whose numerics change with whatever happens to be pip-installed is
a controller whose results are not comparable across machines, and
[fairness rule 2](../concepts/fairness.md) is that every controller in a
comparison runs under identical conditions. An accelerator is opt-in, by name,
and the backend that actually solved a run is recorded in
`df.attrs["controller_info"]["solver"]`.

**A missing optional dependency is loud, never a silent fallback.** Someone who
asked for OSQP and quietly got the dense solver has a differently-solved run and
no way to know it.

## Why a dual active-set method

The default is Goldfarb–Idnani: dense, numpy and scipy only. It starts from the
**unconstrained minimiser** and admits constraints only as they are violated,
which buys three things worth more here than raw speed:

1. With nothing active it terminates immediately at the exact answer — so
   "an unconstrained MPC is a linear controller, and here is which one" is
   testable to machine precision rather than to a solver tolerance.
2. It terminates finitely and exactly, which makes it the *reference* the
   optional backends are checked against, not the other way round.
3. Its iteration count reads as **how many limits the process forced on the
   controller this sample** — which is worth publishing as a diagnostic, and an
   ADMM iteration count is not.

At twenty to sixty variables, dense is correct and sparsity machinery would be
dead code. A representative MPC solve takes well under a millisecond.

---

## The boundary

::: process_control.solvers.solve_qp

::: process_control.solvers.QPResult

::: process_control.solvers.QPSolver

::: process_control.solvers.get_solver

::: process_control.solvers.available_backends

---

## The default backend

::: process_control.solvers.active_set.solve_dense_qp

# Multi-Echelon Inventory Optimization (Python) — Serial Guaranteed-Service Safety-Stock Placement

A Python 3 port of the C++ `multi_echelon_inventory` project. It
implements the classical **Guaranteed-Service Model (GSM)** for
safety-stock placement (Graves & Willems, 2000), **specialized to a
single serial chain** of echelons — e.g. Plant → Regional DC → Local
DC/Customer — a straight line, not a general branching tree/network
where a stage could feed multiple downstream successors.

That's a deliberate scope limitation: the general tree/network GSM (a
DAG rather than a simple path) is substantially harder — it loses the
left-to-right optimal substructure the serial case has, and solving it
exactly typically needs branch-and-bound or a full MIP over the tree,
not a one-dimensional DP. A real deployment spanning plants, DCs, and
customers is often reducible to (or well approximated by) a serial
chain along the primary flow path, which is the scope this project
targets.

## Model

**Stages.** `N` stages `1..N`, upstream (stage 1, e.g. Plant) to
downstream (stage `N`, customer-facing). Each stage `i` has an integer
lead time `L_i` and a holding cost per unit `h_i`. Demand variability
is a property of the whole chain, not any one stage, so the model uses
one shared `demand_std_dev_per_period` (σ) and `service_level_z` (z).

**Decision variables.** Outbound service time `S_i` for `i = 1..N-1`
only: `SI_1 = 0` fixed (stage 1 has immediate input access); `S_N = 0`
fixed (make-to-stock — stage `N` always serves the customer from
safety stock); for `i = 2..N-1`, `SI_i = S_{i-1}` (serial linkage).

**Net replenishment time (NRT).**

```
NRT_1 = L_1 - S_1                       (must be >= 0)
NRT_i = S_{i-1} + L_i - S_i             for i = 2..N-1  (must be >= 0)
NRT_N = S_{N-1} + L_N                   (always >= 0 automatically)
```

**Objective.** Each stage's safety-stock cost is
`h_i * z * sigma * sqrt(NRT_i)` (standard square-root safety-stock
formula under normal demand). Minimize the sum over all stages:

```
minimize   sum_i  h_i * z * sigma * sqrt(NRT_i)
subject to NRT_i >= 0                    for i = 1..N-1
           NRT_1 = L_1 - S_1
           NRT_i = S_{i-1} + L_i - S_i   for i = 2..N-1
           NRT_N = S_{N-1} + L_N
           S_i integer, 0 <= S_i <= cumulative_lead_time(i)
```

`cumulative_lead_time(i) = L_1 + ... + L_i` bounds any useful `S_i`:
pushing outbound service time past accumulated lead time never helps,
since NRT can't go negative upstream.

**Special case `N = 1`.** The single stage is both first and last, so
its outbound service time is fixed at 0 — no free decision variables
at all. `NRT_1 = L_1`, cost `= h_1 * z * sigma * sqrt(L_1)`.

## Design

- `meio/echelon_node.py` — `EchelonNode`, a frozen dataclass: id, name,
  integer lead time, holding cost.
- `meio/problem.py` — `MultiEchelonProblem`, owning the ordered stage
  list plus shared σ/z; exposes `cumulative_lead_time(i)`,
  `safety_stock_cost(stage_index, nrt)`, and `validate(solution)`,
  which recomputes every stage's NRT from a candidate
  `outbound_service_time` list and sets feasibility/cost in place.
  Also defines the `EchelonSolution` dataclass.
- `meio/solver.py` — `EchelonSolver`, an `ABC` strategy interface so
  the placement algorithm can be swapped freely.
- `meio/all_local_solver.py` — `AllLocalHeuristicSolver`, the
  classical "no coordination" baseline: every free `S_i = 0`. Always
  feasible, fast (`O(N)`).
- `meio/exact_solver.py` — `ExactDPSolver`, solving to guaranteed
  global optimality via a left-to-right **dynamic program**, not
  branch-and-bound. `dp[i][s]` = cheapest cost of stages `1..i` given
  stage `i` chose service time `s`, built from `dp[i-1][*]`.

Both `AllLocalHeuristicSolver` and `ExactDPSolver` are dependency-free
— standard library only (`math.sqrt`) — and are what `tests/` exercise
directly.

- `meio/pulp_solver.py` — `PulpMipSolver`, an alternative solver
  mirroring the C++ project's `CbcMipSolver.h`: it formulates the same
  DP recurrence as a layered-graph shortest-path MIP (binary arc
  variables per `(stage, incoming S, outgoing S)`, flow-conservation
  constraints tying layers together) and solves it with PuLP's bundled
  CBC backend. Since the serial GSM is polynomial-time solvable via
  `ExactDPSolver`, this MIP formulation isn't needed for correctness —
  it's included for portfolio-consistency and to document the
  alternative formulation, the same rationale the C++ README gives for
  `CbcMipSolver.h`. `pulp` is imported lazily inside `solve()`, so
  importing the `meio` package never requires it to be installed; its
  test is skipped automatically when `pulp` is absent.

## Build & run

```bash
pip install -r requirements.txt   # optional -- only needed for the PuLP/CBC solver
python3 -m unittest discover -s tests -v
python3 main.py
```

## Tests

`tests/test_echelon.py` ports every case from the C++ suite with the
same hand-verified numbers: the exact DP against a hand-verified
2-stage optimum (worked by hand from the closed-form cost function);
the DP never doing worse than the AllLocal heuristic, with a case
where coordination matters a lot; the `N = 1` edge case; a 3-stage
smoke test plus a cross-check against brute-force search;
`validate()` detecting an infeasible (negative-NRT) hand-built
solution; and edge cases like all-zero lead times and symmetric
two-stage costs. One additional test (skipped if `pulp` isn't
installed) checks `PulpMipSolver` agrees with `ExactDPSolver` on cost.

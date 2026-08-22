# Multi-Echelon Inventory Optimization (C++) — Serial Guaranteed-Service Safety-Stock Placement

A C++ reimplementation of the multi-echelon inventory/network optimization
model from my resume: *"Built multi-echelon inventory/network optimization
models spanning plants, distribution centers, and customers to balance
service level against inventory and transportation cost."* This rewrite
demonstrates hands-on C++ — object-oriented design, dynamic programming,
and unit testing — applied to a problem I've already formulated and
solved, not a generic textbook exercise.

## Scope: the serial special case, not the general network

This implements the classical **Guaranteed-Service Model (GSM)** for
safety-stock placement (Graves & Willems, 2000), **specialized to a
single serial chain** of echelons — e.g. Plant → Regional DC → Local
DC/Customer — a straight line, not a general branching tree/network where
a stage could feed multiple downstream successors.

That's a deliberate scope limitation: the general tree/network GSM (a DAG
rather than a simple path) is substantially harder — it loses the
left-to-right optimal substructure the serial case has, and solving it
exactly typically needs branch-and-bound or a full MIP over the tree, not
a one-dimensional DP. A real deployment spanning plants, DCs, and
customers is often reducible to (or well approximated by) a serial chain
along the primary flow path, which is the scope this project targets.

## Model

**Stages.** `N` stages `1..N`, upstream (stage 1, e.g. Plant) to
downstream (stage `N`, customer-facing). Each stage `i` has an integer
lead time `L_i` and a holding cost per unit `h_i`. Demand variability is
a property of the whole chain, not any one stage, so the model uses one
shared `demandStdDevPerPeriod` (σ) and `serviceLevelZ` (z).

**Decision variables.** Outbound service time `S_i` for `i = 1..N-1`
only: `SI_1 = 0` fixed (stage 1 has immediate input access); `S_N = 0`
fixed (make-to-stock — stage `N` always serves the customer from safety
stock); for `i = 2..N-1`, `SI_i = S_{i-1}` (serial linkage).

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
           S_i integer, 0 <= S_i <= cumulativeLeadTime(i)
```

`cumulativeLeadTime(i) = L_1 + ... + L_i` bounds any useful `S_i`:
pushing outbound service time past accumulated lead time never helps,
since NRT can't go negative upstream.

**Special case `N = 1`.** The single stage is both first and last, so its
outbound service time is fixed at 0 — no free decision variables at all.
`NRT_1 = L_1`, cost `= h_1 * z * sigma * sqrt(L_1)`.

## Design

- `EchelonNode` — one stage: id, name, integer lead time, holding cost.
- `MultiEchelonProblem` — owns the ordered stage list plus shared σ/z;
  exposes `cumulativeLeadTime(i)`, `safetyStockCost(stageIndex, nrt)`,
  and `validate(EchelonSolution&)`, which recomputes every stage's NRT
  from a candidate `outboundServiceTime` vector and sets feasibility/cost.
- `EchelonSolver` — abstract `Strategy` interface so the placement
  algorithm can be swapped freely. Two implementations:
  - `AllLocalHeuristicSolver` — the classical "no coordination" baseline:
    every free `S_i = 0`. Always feasible, fast (`O(N)`), ignores any
    benefit from letting one stage's service time flex to help another.
  - `ExactDPSolver` — solves to guaranteed global optimality via a
    left-to-right **dynamic program**, not branch-and-bound. The serial
    chain gives `S_i` optimal substructure with only its neighbors:
    `dp[i][s]` = cheapest cost of stages `1..i` given stage `i` chose
    service time `s`, built from `dp[i-1][*]`. Polynomial time — no
    exponential search or pruning needed, unlike the NP-hard problems
    branch-and-bound solves elsewhere in this portfolio.

## Solver backends

This repo ships the from-scratch `ExactDPSolver` so it builds and runs
with **no external dependencies**. Since the serial GSM is
polynomial-time solvable, this DP is already provably optimal — not an
approximation standing in for a "real" MIP solver.

`include/CbcMipSolver.h` is included anyway, in the spirit of this
portfolio's other projects, to document how the same DP can be
linearized into a MIP for **COIN-OR CBC's C++ API**
(`OsiClpSolverInterface` + `CbcModel`) — a layered-graph shortest-path
formulation with binary arc variables `w_i(a,b)` and flow-conservation
constraints between layers (see the file's comments; Humair & Willems
covers the general tree-network extension). Compiled only when
`MEIO_USE_CBC` is defined:

```bash
sudo apt-get install coinor-libcbc-dev coinor-libclp-dev coinor-libosi-dev coinor-libcoinutils-dev
cmake -DUSE_CBC=ON -B build && cmake --build build
```

This file was never compiled in the environment this project was
developed in (no CBC installed) and isn't exercised by the test suite —
kept well-formed and API-plausible as documentation of the
production-scale path.

## Build & run

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/meio_demo        # sample 3-stage chain, both solvers' results
./build/meio_tests       # unit test suite
```

## Tests

A small, dependency-free unit test harness covers: the exact DP against
a hand-verified 2-stage optimum (worked by hand from the closed-form
cost function); the DP never doing worse than the AllLocal heuristic,
with a case where coordination matters a lot; the `N = 1` edge case; a
3-stage smoke test plus a cross-check against brute-force search;
`validate()` detecting an infeasible (negative-NRT) hand-built solution;
and edge cases like all-zero lead times and symmetric two-stage costs.

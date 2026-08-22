"""ExactDPSolver: exact serial GSM solver via left-to-right DP."""

import math

from .problem import EchelonSolution, MultiEchelonProblem
from .solver import EchelonSolver


class ExactDPSolver(EchelonSolver):
    """Exact solver for the serial guaranteed-service safety-stock
    placement problem. This is NOT a branch-and-bound search: because
    the chain is serial (a straight line of stages, not a general
    tree), the outbound service time of stage i only interacts with its
    immediate neighbors (SI_i = S_{i-1}, and S_i feeds into stage i+1's
    NRT), so the problem has optimal substructure and can be solved to
    guaranteed global optimality in polynomial time with a
    straightforward left-to-right dynamic program over
    (stage, candidate outbound service time) states -- see README.md
    for the full recurrence. (The general branching-network version of
    this problem, where a stage can have multiple downstream
    successors, is materially harder and is not what this class
    solves.)

    Complexity: O(sum_i cumulative_lead_time(i)^2), i.e. polynomial in
    both the number of stages and the lead times -- no exponential
    blow-up, no pruning heuristics needed.
    """

    def solve(self, problem: MultiEchelonProblem) -> EchelonSolution:
        n = problem.stage_count()
        if n <= 0:
            raise ValueError("ExactDPSolver.solve: problem has no stages")

        solution = EchelonSolution()

        # Special case: a single stage is both the first and the last
        # stage, so its outbound service time is fixed at 0 too -- there
        # are no free decision variables and no search is needed.
        if n == 1:
            solution.outbound_service_time = []
            problem.validate(solution)
            return solution

        inf = math.inf

        # dp_cost[j][s] / dp_backptr[j][s] describe the state "stage j
        # (0-based) chose outbound service time s": the cheapest total
        # safety-stock cost of stages 0..j given that choice, and (for
        # j >= 1) the outbound service time chosen by stage j-1 that
        # achieves it. Only stages 0..n-2 get a layer here -- stage
        # n-1's outbound service time is fixed at 0 (make-to-stock) and
        # is folded into the final step below rather than given its own
        # decision layer.
        num_layers = n - 1
        dp_cost = [None] * num_layers
        dp_backptr = [None] * num_layers

        # Base case: dp[0][s] = h_0 * z * sigma * sqrt(L_0 - s), for
        # s = 0..L_0 (= cumulative_lead_time(0), since it's just stage
        # 0's own lead time). SI_0 = 0 is fixed, so NRT_0 = L_0 - s
        # directly.
        cap0 = problem.cumulative_lead_time(0)
        dp_cost[0] = [0.0] * (cap0 + 1)
        for s in range(cap0 + 1):
            nrt = problem.stages[0].lead_time - s
            dp_cost[0][s] = problem.safety_stock_cost(0, nrt)

        # Forward recurrence over the remaining decision-bearing stages
        # 1..n-2 (0-based): dp[j][s] = min over feasible s_prev of
        # dp[j-1][s_prev] + cost_j(s_prev + L_j - s).
        for j in range(1, num_layers):
            cap_j = problem.cumulative_lead_time(j)
            cap_prev = len(dp_cost[j - 1]) - 1
            dp_cost[j] = [inf] * (cap_j + 1)
            dp_backptr[j] = [-1] * (cap_j + 1)

            for s_prev in range(cap_prev + 1):
                prev_cost = dp_cost[j - 1][s_prev]
                for s in range(cap_j + 1):
                    nrt = s_prev + problem.stages[j].lead_time - s
                    if nrt < 0.0:
                        continue  # infeasible transition: NRT_j must be >= 0
                    cost = prev_cost + problem.safety_stock_cost(j, nrt)
                    if cost < dp_cost[j][s]:
                        dp_cost[j][s] = cost
                        dp_backptr[j][s] = s_prev

        # Final step: fold in the last stage (index n-1), whose outbound
        # service time is fixed at 0, so its NRT is simply
        # s_prev + L_{n-1} and is always feasible (>= 0) for any
        # s_prev >= 0.
        last_decision_stage = num_layers - 1  # = n - 2
        cap_last = len(dp_cost[last_decision_stage]) - 1

        best_cost = inf
        best_s_prev = -1
        for s_prev in range(cap_last + 1):
            nrt = s_prev + problem.stages[n - 1].lead_time
            cost = dp_cost[last_decision_stage][s_prev] + problem.safety_stock_cost(
                n - 1, nrt
            )
            if cost < best_cost:
                best_cost = cost
                best_s_prev = s_prev

        # Reconstruct S_0..S_{n-2} by walking the backpointers right to
        # left.
        outbound = [0] * num_layers
        outbound[last_decision_stage] = best_s_prev
        for j in range(last_decision_stage, 0, -1):
            outbound[j - 1] = dp_backptr[j][outbound[j]]
        solution.outbound_service_time = outbound

        problem.validate(solution)
        return solution

    def name(self) -> str:
        return "ExactDP-Optimal"

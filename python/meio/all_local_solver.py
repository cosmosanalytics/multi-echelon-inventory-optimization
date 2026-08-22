"""AllLocalHeuristicSolver: the classical no-coordination baseline."""

from .problem import EchelonSolution, MultiEchelonProblem
from .solver import EchelonSolver


class AllLocalHeuristicSolver(EchelonSolver):
    """The classical "no coordination" baseline from the
    guaranteed-service safety-stock literature: every stage quotes an
    outbound service time of 0 (i.e. promises immediate availability
    downstream), regardless of what that costs it. This is always
    feasible -- S_i = 0 never forces a negative net replenishment time
    -- but ignores the fact that letting an upstream stage carry a bit
    of extra service time can let a downstream stage (often cheaper to
    hold stock at, or vice versa) shed cost. It is the fast O(N)
    baseline the exact solver is compared against.
    """

    def solve(self, problem: MultiEchelonProblem) -> EchelonSolution:
        solution = EchelonSolution()
        n = problem.stage_count()
        if n > 1:
            solution.outbound_service_time = [0] * (n - 1)
        problem.validate(solution)
        return solution

    def name(self) -> str:
        return "AllLocal-Heuristic"

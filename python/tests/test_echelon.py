"""Port of tests/EchelonTests.cpp -- same hand-verified numeric assertions."""

import math
import unittest

from meio import (
    AllLocalHeuristicSolver,
    EchelonNode,
    EchelonSolution,
    ExactDPSolver,
    MultiEchelonProblem,
)

try:
    import pulp  # noqa: F401

    PULP_AVAILABLE = True
except ImportError:
    PULP_AVAILABLE = False

# z*sigma = 1 everywhere below so the hand-verified spec numbers come clean.
SIGMA = 1.0
Z = 1.0


def _two_stage():
    # stage1 L=4 h=5; stage2 L=1 h=1.
    return [EchelonNode(1, "Stage1", 4, 5.0), EchelonNode(2, "Stage2", 1, 1.0)]


class EchelonTests(unittest.TestCase):
    def test_exact_dp_achieves_known_optimum(self):
        # Free var S_1 in {0..4}. Cost(S_1)=5*sqrt(4-S_1)+1*sqrt(S_1+1):
        # 0->11.0 1->10.0744676 2->8.8031186 3->7.0 4->sqrt(5)=2.236 <- min
        problem = MultiEchelonProblem(_two_stage(), SIGMA, Z)
        solver = ExactDPSolver()
        solution = solver.solve(problem)
        self.assertTrue(solution.feasible)
        self.assertEqual(len(solution.outbound_service_time), 1)
        self.assertEqual(solution.outbound_service_time[0], 4)
        self.assertTrue(2.235 < solution.total_cost < 2.237)

    def test_exact_dp_never_worse_than_all_local(self):
        # Same setup. AllLocal gives S_1=0, cost=5*sqrt(4)+1*sqrt(1)=11.0
        # exactly, vs exact optimum ~2.236 -- a large coordination gain.
        problem = MultiEchelonProblem(_two_stage(), SIGMA, Z)
        heuristic = AllLocalHeuristicSolver()
        exact = ExactDPSolver()
        all_local = heuristic.solve(problem)
        optimal = exact.solve(problem)
        self.assertTrue(all_local.feasible)
        self.assertTrue(optimal.feasible)
        self.assertTrue(10.999 < all_local.total_cost < 11.001)
        self.assertLessEqual(optimal.total_cost, all_local.total_cost + 1e-9)
        self.assertGreater(all_local.total_cost - optimal.total_cost, 8.0)

    def test_edge_case_single_stage_no_free_variables(self):
        # N=1: L=6 h=2. No free vars. Cost=2*z*sigma*sqrt(6)=2*sqrt(6)=4.898979.
        stages = [EchelonNode(1, "OnlyStage", 6, 2.0)]
        problem = MultiEchelonProblem(stages, SIGMA, Z)
        solver = ExactDPSolver()
        solution = solver.solve(problem)
        self.assertTrue(solution.feasible)
        self.assertEqual(solution.outbound_service_time, [])
        self.assertTrue(4.898 < solution.total_cost < 4.900)

    def test_three_stage_smoke_test(self):
        stages = [EchelonNode(1, "Plant", 3, 4.0), EchelonNode(2, "RegionalDC", 2, 2.0),
                  EchelonNode(3, "LocalDC", 2, 1.0)]
        problem = MultiEchelonProblem(stages, SIGMA, Z)
        solver = ExactDPSolver()
        solution = solver.solve(problem)
        self.assertTrue(solution.feasible)
        self.assertGreater(solution.total_cost, 0.0)
        self.assertEqual(len(solution.outbound_service_time), 2)

    def test_feasibility_detects_negative_nrt(self):
        # S_1=5 exceeds stage 1's lead time (4) -> NRT_1 = 4-5 = -1 < 0.
        problem = MultiEchelonProblem(_two_stage(), SIGMA, Z)
        bad = EchelonSolution(outbound_service_time=[5])
        problem.validate(bad)
        self.assertFalse(bad.feasible)

    def test_edge_case_all_zero_lead_times(self):
        # Every stage has zero lead time -> only feasible S is 0
        # everywhere, every NRT (and safety-stock cost) collapses to 0.
        stages = [EchelonNode(1, "Plant", 0, 3.0), EchelonNode(2, "DC", 0, 2.0),
                  EchelonNode(3, "Customer", 0, 1.0)]
        problem = MultiEchelonProblem(stages, SIGMA, Z)
        solver = ExactDPSolver()
        solution = solver.solve(problem)
        self.assertTrue(solution.feasible)
        self.assertEqual(len(solution.outbound_service_time), 2)
        for s in solution.outbound_service_time:
            self.assertEqual(s, 0)
        self.assertAlmostEqual(solution.total_cost, 0.0, delta=1e-9)

    def test_edge_case_symmetric_two_stage_costs(self):
        # Two identical stages: no cost advantage to shifting service time,
        # but the solver must still return a feasible, sane result.
        stages = [EchelonNode(1, "StageA", 3, 2.0), EchelonNode(2, "StageB", 3, 2.0)]
        problem = MultiEchelonProblem(stages, SIGMA, Z)
        solver = ExactDPSolver()
        solution = solver.solve(problem)
        self.assertTrue(solution.feasible)
        self.assertGreater(solution.total_cost, 0.0)
        heuristic = AllLocalHeuristicSolver()  # AllLocal must never beat exact optimum
        all_local = heuristic.solve(problem)
        self.assertLessEqual(solution.total_cost, all_local.total_cost + 1e-9)

    def test_exact_dp_matches_brute_force_on_small_instance(self):
        # Cross-check the DP against exhaustive brute force over all
        # (S_1, S_2) combos on a small 3-stage instance.
        stages = [EchelonNode(1, "Plant", 4, 3.0), EchelonNode(2, "RegionalDC", 3, 5.0),
                  EchelonNode(3, "LocalDC", 2, 6.0)]
        problem = MultiEchelonProblem(stages, 2.0, 1.5)
        solver = ExactDPSolver()
        dp_solution = solver.solve(problem)
        self.assertTrue(dp_solution.feasible)

        brute_best = math.inf
        cap0 = problem.cumulative_lead_time(0)
        cap1 = problem.cumulative_lead_time(1)
        for s0 in range(cap0 + 1):
            for s1 in range(cap1 + 1):
                candidate = EchelonSolution(outbound_service_time=[s0, s1])
                problem.validate(candidate)
                if candidate.feasible and candidate.total_cost < brute_best:
                    brute_best = candidate.total_cost
        self.assertAlmostEqual(dp_solution.total_cost, brute_best, delta=1e-9)

    @unittest.skipUnless(PULP_AVAILABLE, "pulp not installed")
    def test_pulp_solver_matches_exact_dp(self):
        from meio.pulp_solver import PulpMipSolver

        problem = MultiEchelonProblem(_two_stage(), SIGMA, Z)
        exact_solution = ExactDPSolver().solve(problem)
        pulp_solution = PulpMipSolver().solve(problem)
        self.assertTrue(pulp_solution.feasible)
        self.assertAlmostEqual(pulp_solution.total_cost, exact_solution.total_cost, delta=1e-6)


if __name__ == "__main__":
    unittest.main()

#include <cmath>
#include <limits>
#include <vector>

#include "AllLocalHeuristicSolver.h"
#include "EchelonNode.h"
#include "ExactDPSolver.h"
#include "MultiEchelonProblem.h"
#include "TestFramework.h"

using namespace meio;

namespace {
// z*sigma = 1 everywhere below so the hand-verified spec numbers come clean.
constexpr double kSigma = 1.0;
constexpr double kZ = 1.0;
} // namespace

TEST(ExactDP_AchievesKnownOptimum) {
    // 2-stage: stage1 L=4 h=5; stage2 L=1 h=1. Free var S_1 in {0..4}.
    // Cost(S_1)=5*sqrt(4-S_1)+1*sqrt(S_1+1): 0->11.0 1->10.0744676
    // 2->8.8031186 3->7.0 4->sqrt(5)=2.2360680 <- minimum
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Stage1", 4, 5.0), EchelonNode(2, "Stage2", 1, 1.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    ExactDPSolver solver;
    EchelonSolution solution = solver.solve(problem);
    CHECK(solution.feasible);
    CHECK(solution.outboundServiceTime.size() == 1);
    CHECK(solution.outboundServiceTime[0] == 4);
    CHECK(solution.totalCost > 2.235 && solution.totalCost < 2.237);
}

TEST(ExactDP_NeverWorseThanAllLocal) {
    // Same setup. AllLocal gives S_1=0, cost=5*sqrt(4)+1*sqrt(1)=11.0
    // exactly, vs exact optimum ~2.236 -- a large coordination gain.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Stage1", 4, 5.0), EchelonNode(2, "Stage2", 1, 1.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    AllLocalHeuristicSolver heuristic;
    ExactDPSolver exact;
    EchelonSolution allLocal = heuristic.solve(problem);
    EchelonSolution optimal = exact.solve(problem);
    CHECK(allLocal.feasible);
    CHECK(optimal.feasible);
    CHECK(allLocal.totalCost > 10.999 && allLocal.totalCost < 11.001);
    CHECK(optimal.totalCost <= allLocal.totalCost + 1e-9);
    CHECK(allLocal.totalCost - optimal.totalCost > 8.0); // gap should be large
}

TEST(EdgeCase_SingleStageNoFreeVariables) {
    // N=1: L=6 h=2. No free vars. Cost=2*z*sigma*sqrt(6)=2*sqrt(6)=4.898979.
    std::vector<EchelonNode> stages = {EchelonNode(1, "OnlyStage", 6, 2.0)};
    MultiEchelonProblem problem(stages, kSigma, kZ);
    ExactDPSolver solver;
    EchelonSolution solution = solver.solve(problem);
    CHECK(solution.feasible);
    CHECK(solution.outboundServiceTime.empty());
    CHECK(solution.totalCost > 4.898 && solution.totalCost < 4.900);
}

TEST(ThreeStage_SmokeTest) {
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Plant", 3, 4.0), EchelonNode(2, "RegionalDC", 2, 2.0),
        EchelonNode(3, "LocalDC", 2, 1.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    ExactDPSolver solver;
    EchelonSolution solution = solver.solve(problem);
    CHECK(solution.feasible);
    CHECK(solution.totalCost > 0.0);
    CHECK(solution.outboundServiceTime.size() == 2);
}

TEST(Feasibility_DetectsNegativeNRT) {
    // S_1=5 exceeds stage 1's lead time (4) -> NRT_1 = 4-5 = -1 < 0.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Stage1", 4, 5.0), EchelonNode(2, "Stage2", 1, 1.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    EchelonSolution bad;
    bad.outboundServiceTime = {5};
    problem.validate(bad);
    CHECK(!bad.feasible);
}

TEST(EdgeCase_AllZeroLeadTimes) {
    // Every stage has zero lead time -> only feasible S is 0 everywhere,
    // every NRT (and safety-stock cost) collapses to 0.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Plant", 0, 3.0), EchelonNode(2, "DC", 0, 2.0),
        EchelonNode(3, "Customer", 0, 1.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    ExactDPSolver solver;
    EchelonSolution solution = solver.solve(problem);
    CHECK(solution.feasible);
    CHECK(solution.outboundServiceTime.size() == 2);
    for (int s : solution.outboundServiceTime) CHECK(s == 0);
    CHECK(std::fabs(solution.totalCost) < 1e-9);
}

TEST(EdgeCase_SymmetricTwoStageCosts) {
    // Two identical stages: no cost advantage to shifting service time,
    // but the solver must still return a feasible, sane result.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "StageA", 3, 2.0), EchelonNode(2, "StageB", 3, 2.0),
    };
    MultiEchelonProblem problem(stages, kSigma, kZ);
    ExactDPSolver solver;
    EchelonSolution solution = solver.solve(problem);
    CHECK(solution.feasible);
    CHECK(solution.totalCost > 0.0);
    AllLocalHeuristicSolver heuristic; // AllLocal must never beat exact optimum
    EchelonSolution allLocal = heuristic.solve(problem);
    CHECK(solution.totalCost <= allLocal.totalCost + 1e-9);
}

TEST(ExactDP_MatchesBruteForceOnSmallInstance) {
    // Cross-check the DP against exhaustive brute force over all
    // (S_1, S_2) combos on a small 3-stage instance.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Plant", 4, 3.0), EchelonNode(2, "RegionalDC", 3, 5.0),
        EchelonNode(3, "LocalDC", 2, 6.0),
    };
    MultiEchelonProblem problem(stages, /*sigma=*/2.0, /*z=*/1.5);
    ExactDPSolver solver;
    EchelonSolution dpSolution = solver.solve(problem);
    CHECK(dpSolution.feasible);

    double bruteBest = std::numeric_limits<double>::infinity();
    const int cap0 = problem.cumulativeLeadTime(0);
    const int cap1 = problem.cumulativeLeadTime(1);
    for (int s0 = 0; s0 <= cap0; ++s0) {
        for (int s1 = 0; s1 <= cap1; ++s1) {
            EchelonSolution candidate;
            candidate.outboundServiceTime = {s0, s1};
            problem.validate(candidate);
            if (candidate.feasible && candidate.totalCost < bruteBest) {
                bruteBest = candidate.totalCost;
            }
        }
    }
    CHECK(std::fabs(dpSolution.totalCost - bruteBest) < 1e-9);
}

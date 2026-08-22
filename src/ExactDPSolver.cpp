#include "ExactDPSolver.h"

#include <limits>
#include <stdexcept>
#include <vector>

namespace meio {

EchelonSolution ExactDPSolver::solve(const MultiEchelonProblem& problem) {
    const int n = problem.stageCount();
    if (n <= 0) {
        throw std::invalid_argument("ExactDPSolver::solve: problem has no stages");
    }

    EchelonSolution solution;

    // Special case: a single stage is both the first and the last stage,
    // so its outbound service time is fixed at 0 too -- there are no free
    // decision variables and no search is needed.
    if (n == 1) {
        solution.outboundServiceTime.clear();
        problem.validate(solution);
        return solution;
    }

    const double kInfinity = std::numeric_limits<double>::infinity();

    // dpCost[j][s] / dpBackptr[j][s] describe the state "stage j (0-based)
    // chose outbound service time s": the cheapest total safety-stock cost
    // of stages 0..j given that choice, and (for j >= 1) the outbound
    // service time chosen by stage j-1 that achieves it. Only stages
    // 0..n-2 get a layer here -- stage n-1's outbound service time is
    // fixed at 0 (make-to-stock) and is folded into the final step below
    // rather than given its own decision layer.
    const int numLayers = n - 1;
    std::vector<std::vector<double>> dpCost(numLayers);
    std::vector<std::vector<int>> dpBackptr(numLayers);

    // Base case: dp[0][s] = h_0 * z * sigma * sqrt(L_0 - s), for
    // s = 0..L_0 (= cumulativeLeadTime(0), since it's just stage 0's own
    // lead time). SI_0 = 0 is fixed, so NRT_0 = L_0 - s directly.
    const int cap0 = problem.cumulativeLeadTime(0);
    dpCost[0].resize(cap0 + 1);
    for (int s = 0; s <= cap0; ++s) {
        const double nrt = problem.stages()[0].leadTime() - s;
        dpCost[0][s] = problem.safetyStockCost(0, nrt);
    }

    // Forward recurrence over the remaining decision-bearing stages
    // 1..n-2 (0-based): dp[j][s] = min over feasible s_prev of
    // dp[j-1][s_prev] + cost_j(s_prev + L_j - s).
    for (int j = 1; j < numLayers; ++j) {
        const int capJ = problem.cumulativeLeadTime(j);
        const int capPrev = static_cast<int>(dpCost[j - 1].size()) - 1;
        dpCost[j].assign(capJ + 1, kInfinity);
        dpBackptr[j].assign(capJ + 1, -1);

        for (int sPrev = 0; sPrev <= capPrev; ++sPrev) {
            const double prevCost = dpCost[j - 1][sPrev];
            for (int s = 0; s <= capJ; ++s) {
                const double nrt = sPrev + problem.stages()[j].leadTime() - s;
                if (nrt < 0.0) continue; // infeasible transition: NRT_j must be >= 0
                const double cost = prevCost + problem.safetyStockCost(j, nrt);
                if (cost < dpCost[j][s]) {
                    dpCost[j][s] = cost;
                    dpBackptr[j][s] = sPrev;
                }
            }
        }
    }

    // Final step: fold in the last stage (index n-1), whose outbound
    // service time is fixed at 0, so its NRT is simply
    // s_prev + L_{n-1} and is always feasible (>= 0) for any s_prev >= 0.
    const int lastDecisionStage = numLayers - 1; // = n - 2
    const int capLast = static_cast<int>(dpCost[lastDecisionStage].size()) - 1;

    double bestCost = kInfinity;
    int bestSPrev = -1;
    for (int sPrev = 0; sPrev <= capLast; ++sPrev) {
        const double nrt = sPrev + problem.stages()[n - 1].leadTime();
        const double cost = dpCost[lastDecisionStage][sPrev] + problem.safetyStockCost(n - 1, nrt);
        if (cost < bestCost) {
            bestCost = cost;
            bestSPrev = sPrev;
        }
    }

    // Reconstruct S_0..S_{n-2} by walking the backpointers right to left.
    solution.outboundServiceTime.assign(numLayers, 0);
    solution.outboundServiceTime[lastDecisionStage] = bestSPrev;
    for (int j = lastDecisionStage; j >= 1; --j) {
        solution.outboundServiceTime[j - 1] = dpBackptr[j][solution.outboundServiceTime[j]];
    }

    problem.validate(solution);
    return solution;
}

} // namespace meio

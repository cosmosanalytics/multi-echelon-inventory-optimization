#include "AllLocalHeuristicSolver.h"

namespace meio {

EchelonSolution AllLocalHeuristicSolver::solve(const MultiEchelonProblem& problem) {
    EchelonSolution solution;
    const int n = problem.stageCount();
    if (n > 1) {
        solution.outboundServiceTime.assign(n - 1, 0);
    }
    problem.validate(solution);
    return solution;
}

} // namespace meio

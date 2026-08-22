#pragma once

#include "EchelonSolver.h"

namespace meio {

// Exact solver for the serial guaranteed-service safety-stock placement
// problem. This is NOT a branch-and-bound search: because the chain is
// serial (a straight line of stages, not a general tree), the outbound
// service time of stage i only interacts with its immediate neighbors
// (SI_i = S_{i-1}, and S_i feeds into stage i+1's NRT), so the problem has
// optimal substructure and can be solved to guaranteed global optimality
// in polynomial time with a straightforward left-to-right dynamic program
// over (stage, candidate outbound service time) states -- see README.md
// for the full recurrence. (The general branching-network version of this
// problem, where a stage can have multiple downstream successors, is
// materially harder and is not what this class solves.)
//
// Complexity: O(sum_i cumulativeLeadTime(i)^2), i.e. polynomial in both
// the number of stages and the lead times -- no exponential blow-up, no
// pruning heuristics needed, unlike the branch-and-bound solvers used
// elsewhere in this portfolio for genuinely NP-hard problems.
class ExactDPSolver : public EchelonSolver {
public:
    EchelonSolution solve(const MultiEchelonProblem& problem) override;
    std::string name() const override { return "ExactDP-Optimal"; }
};

} // namespace meio

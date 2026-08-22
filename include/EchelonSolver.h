#pragma once

#include <string>

#include "MultiEchelonProblem.h"

namespace meio {

// Strategy interface: any algorithm capable of placing safety stock on a
// MultiEchelonProblem implements this. Lets the demo / tests swap solvers
// (a fast heuristic, an exact solver, or an external MIP solver such as
// CBC) without changing any calling code.
class EchelonSolver {
public:
    virtual ~EchelonSolver() = default;
    virtual EchelonSolution solve(const MultiEchelonProblem& problem) = 0;
    virtual std::string name() const = 0;
};

} // namespace meio

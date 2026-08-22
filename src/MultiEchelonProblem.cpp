#include "MultiEchelonProblem.h"

#include <cmath>
#include <stdexcept>

namespace meio {

int MultiEchelonProblem::cumulativeLeadTime(int stageIndex) const {
    if (stageIndex < 0 || stageIndex >= stageCount()) {
        throw std::out_of_range("MultiEchelonProblem::cumulativeLeadTime: stageIndex out of range");
    }
    int total = 0;
    for (int i = 0; i <= stageIndex; ++i) {
        total += stages_[i].leadTime();
    }
    return total;
}

double MultiEchelonProblem::safetyStockCost(int stageIndex, double nrt) const {
    if (stageIndex < 0 || stageIndex >= stageCount()) {
        throw std::out_of_range("MultiEchelonProblem::safetyStockCost: stageIndex out of range");
    }
    // nrt is expected to be >= 0; clamp tiny negative floating-point noise
    // from a feasible-but-boundary NRT rather than feeding sqrt() a
    // negative number, which would return NaN.
    const double safeNrt = nrt > 0.0 ? nrt : 0.0;
    return stages_[stageIndex].holdingCostPerUnit() * serviceLevelZ_ * demandStdDevPerPeriod_ *
           std::sqrt(safeNrt);
}

void MultiEchelonProblem::validate(EchelonSolution& solution) const {
    const int n = stageCount();
    const int expectedFreeVars = n > 1 ? n - 1 : 0;

    if (n == 0 || static_cast<int>(solution.outboundServiceTime.size()) != expectedFreeVars) {
        solution.feasible = false;
        solution.totalCost = 0.0;
        return;
    }

    // Outbound service time of stage i (0-based): the decision variable
    // for i = 0..n-2, fixed at 0 for the last stage (make-to-stock: it
    // must always be ready to serve the customer immediately).
    auto outboundServiceTimeOf = [&](int i) -> int {
        return (i < n - 1) ? solution.outboundServiceTime[i] : 0;
    };

    bool feasible = true;
    double totalCost = 0.0;
    int incomingServiceTime = 0; // SI_0 = 0: stage 0 has immediate access to inputs

    for (int i = 0; i < n; ++i) {
        const int outbound = outboundServiceTimeOf(i);
        const double nrt = incomingServiceTime + stages_[i].leadTime() - outbound;
        if (nrt < -1e-9) {
            feasible = false;
        }
        totalCost += safetyStockCost(i, nrt);
        incomingServiceTime = outbound; // SI_{i+1} = S_i
    }

    solution.feasible = feasible;
    solution.totalCost = totalCost;
}

} // namespace meio

#pragma once

#include <vector>

#include "EchelonNode.h"

namespace meio {

// The result of solving a MultiEchelonProblem: the outbound service time
// chosen for each stage that has a free decision (all stages except the
// last, whose outbound service time is fixed at 0 by the make-to-stock
// assumption), plus the resulting feasibility and total safety-stock cost.
struct EchelonSolution {
    bool feasible = true;
    double totalCost = 0.0;

    // Outbound service time S_i for stage i, i = 0..N-2 (0-based stage
    // index). Size is stages().size() - 1, or empty when there is only
    // one stage (no free decision variables at all in that case).
    std::vector<int> outboundServiceTime;
};

// A serial (linear-chain) multi-echelon inventory problem in the style of
// the Graves & Willems (2000) guaranteed-service model (GSM), specialized
// to a single serial path of stages -- see README.md for the full
// mathematical model. Stages are stored upstream (index 0, e.g. Plant) to
// downstream (index N-1, the customer-facing node).
//
// Demand variability is a single property of the end-customer demand
// stream propagating through the chain, so it is shared across all stages
// rather than modeled per stage: one `demandStdDevPerPeriod` (sigma) and
// one `serviceLevelZ` (z) apply to every stage's safety-stock formula.
class MultiEchelonProblem {
public:
    MultiEchelonProblem(std::vector<EchelonNode> stages,
                         double demandStdDevPerPeriod,
                         double serviceLevelZ)
        : stages_(std::move(stages)),
          demandStdDevPerPeriod_(demandStdDevPerPeriod),
          serviceLevelZ_(serviceLevelZ) {}

    const std::vector<EchelonNode>& stages() const { return stages_; }
    int stageCount() const { return static_cast<int>(stages_.size()); }

    double demandStdDevPerPeriod() const { return demandStdDevPerPeriod_; }
    double serviceLevelZ() const { return serviceLevelZ_; }

    // Sum of leadTime over stages[0..stageIndex] inclusive (0-based). Used
    // as a safe, simple upper bound on the candidate range for outbound
    // service time S_stageIndex: any feasible S_i can never usefully
    // exceed this, since net replenishment time can't go negative anywhere
    // upstream of it.
    int cumulativeLeadTime(int stageIndex) const;

    // Safety-stock holding cost contribution of a single stage, given its
    // net replenishment time (NRT): holdingCost_i * z * sigma * sqrt(nrt).
    // `nrt` must be >= 0 (a negative NRT is an infeasible plan and has no
    // meaningful cost -- callers must check feasibility separately).
    double safetyStockCost(int stageIndex, double nrt) const;

    // Recomputes net replenishment times from `solution.outboundServiceTime`
    // and (re)sets `solution.feasible` and `solution.totalCost` in place.
    // A solution is feasible iff every stage's net replenishment time is
    // >= 0 and outboundServiceTime has the expected size (stageCount()-1,
    // or empty for a single-stage problem).
    void validate(EchelonSolution& solution) const;

private:
    std::vector<EchelonNode> stages_;
    double demandStdDevPerPeriod_;
    double serviceLevelZ_;
};

} // namespace meio

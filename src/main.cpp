#include <iomanip>
#include <iostream>
#include <vector>

#include "AllLocalHeuristicSolver.h"
#include "EchelonNode.h"
#include "ExactDPSolver.h"
#include "MultiEchelonProblem.h"

using namespace meio;

namespace {

void printSolution(const MultiEchelonProblem& problem, const std::string& solverName,
                    const EchelonSolution& solution) {
    std::cout << "\n--- " << solverName << " ---\n";
    std::cout << "Feasible: " << (solution.feasible ? "yes" : "no") << "\n";
    std::cout << "Total safety-stock cost: $" << std::fixed << std::setprecision(4)
              << solution.totalCost << "\n";

    std::cout << "Outbound service times:";
    if (solution.outboundServiceTime.empty()) {
        std::cout << " (none -- single stage, no free decision variables)\n";
    } else {
        std::cout << "\n";
        for (std::size_t i = 0; i < solution.outboundServiceTime.size(); ++i) {
            std::cout << "  " << problem.stages()[i].name() << " -> S = "
                      << solution.outboundServiceTime[i] << " day(s)\n";
        }
        std::cout << "  " << problem.stages().back().name()
                  << " -> S = 0 day(s) (fixed: make-to-stock, must serve customer immediately)\n";
    }
}

} // namespace

int main() {
    // A representative 3-echelon chain: a Plant that manufactures, a
    // Regional DC that consolidates, and a Local DC that faces the
    // customer directly. Lead times and holding costs both increase
    // moving downstream, which is typical: downstream nodes add their own
    // processing/transportation time on top of upstream lead time, and
    // carry higher per-unit holding cost because more value has been
    // added to the product by the time it sits there.
    std::vector<EchelonNode> stages = {
        EchelonNode(1, "Plant", /*leadTime=*/7, /*holdingCostPerUnit=*/2.0),
        EchelonNode(2, "Regional DC", /*leadTime=*/3, /*holdingCostPerUnit=*/4.0),
        EchelonNode(3, "Local DC / Customer", /*leadTime=*/2, /*holdingCostPerUnit=*/7.0),
    };
    const double demandStdDevPerPeriod = 25.0; // units/day, end-customer demand
    const double serviceLevelZ = 1.65;          // ~95% cycle service level

    MultiEchelonProblem problem(stages, demandStdDevPerPeriod, serviceLevelZ);

    std::cout << "Multi-Echelon Inventory Optimization (C++) -- Serial Guaranteed-Service Model\n";
    std::cout << stages.size() << " stages, demand sigma=" << demandStdDevPerPeriod
              << ", service level z=" << serviceLevelZ << "\n";
    for (const auto& s : stages) {
        std::cout << "  " << s.name() << ": leadTime=" << s.leadTime()
                  << " day(s), holdingCost=$" << s.holdingCostPerUnit() << "/unit\n";
    }

    AllLocalHeuristicSolver heuristic;
    printSolution(problem, heuristic.name() + " (baseline, no coordination)",
                  heuristic.solve(problem));

    ExactDPSolver exact;
    EchelonSolution exactSolution = exact.solve(problem);
    printSolution(problem, exact.name() + " (exact, polynomial-time DP)", exactSolution);

    EchelonSolution heuristicSolution = heuristic.solve(problem);
    if (heuristicSolution.totalCost > 0.0) {
        const double savingsPct =
            100.0 * (heuristicSolution.totalCost - exactSolution.totalCost) / heuristicSolution.totalCost;
        std::cout << "\nCoordinating service times across the chain reduces safety-stock cost by "
                  << std::fixed << std::setprecision(1) << savingsPct
                  << "% versus the no-coordination baseline.\n";
    }

    return 0;
}

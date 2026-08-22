#pragma once

// CbcMipSolver -- production-scale path using COIN-OR CBC's C++ API.
// ExactDPSolver.h is the zero-dependency exact solver used by default
// (already optimal in polynomial time -- no MIP solver strictly
// needed). This documents linearizing the same DP into a MIP for CBC,
// the standard approach when a shared MIP toolchain is preferred (see
// Humair & Willems for the general tree-network extension).
//
// Compiled only when MEIO_USE_CBC is defined:
//   sudo apt-get install coinor-libcbc-dev coinor-libclp-dev \
//                         coinor-libosi-dev coinor-libcoinutils-dev
//   cmake -DUSE_CBC=ON -B build && cmake --build build
//
// Never compiled in this dev environment (no CBC installed); kept
// well-formed as documentation, not exercised by the test suite.

#ifdef MEIO_USE_CBC

#include <CbcModel.hpp>
#include <CoinPackedMatrix.hpp>
#include <OsiClpSolverInterface.hpp>

#include <cmath>
#include <vector>

#include "EchelonSolver.h"

namespace meio {

// Formulates the serial GSM DP as a shortest path on a layered graph,
// linearized into a MIP: layer j holds one node per candidate outbound
// service time for stage j; binary arc w_j(a,b) = "stage j-1 chose a AND
// stage j chose b", costed via stage j's safety-stock formula (same as
// ExactDPSolver's recurrence; infeasible/negative-NRT arcs are never
// generated). Flow-conservation ties layers together so a solution is
// one path from layer 0 to a sink representing stage n-1 (fixed S=0).
class CbcMipSolver : public EchelonSolver {
public:
    EchelonSolution solve(const MultiEchelonProblem& problem) override {
        const int n = problem.stageCount();
        EchelonSolution result;
        if (n <= 0) return result;

        if (n == 1) {
            // No free variables at all -- nothing to hand to CBC.
            problem.validate(result);
            return result;
        }

        // Each arc is (layerIndex, a, b, cost): `a` incoming service
        // time, `b` chosen outbound service time; layerIndex identifies
        // which stage's cost formula priced it. Layer 0 fixes a = 0.
        // The final arc set (into the sink) fixes stage n-1's outbound
        // service time at 0.
        struct Arc {
            int layer;
            int a, b;
            double cost;
            int varIndex;
        };
        std::vector<Arc> arcs;

        const double z = problem.serviceLevelZ();
        const double sigma = problem.demandStdDevPerPeriod();

        const int cap0 = problem.cumulativeLeadTime(0);
        for (int b = 0; b <= cap0; ++b) {
            const double nrt = problem.stages()[0].leadTime() - b;
            arcs.push_back({0, 0, b, problem.safetyStockCost(0, nrt), -1});
        }

        for (int j = 1; j <= n - 2; ++j) {
            const int capPrev = problem.cumulativeLeadTime(j - 1);
            const int capJ = problem.cumulativeLeadTime(j);
            for (int a = 0; a <= capPrev; ++a) {
                for (int b = 0; b <= capJ; ++b) {
                    const double nrt = a + problem.stages()[j].leadTime() - b;
                    if (nrt < 0.0) continue;
                    arcs.push_back({j, a, b, problem.safetyStockCost(j, nrt), -1});
                }
            }
        }

        const int capLast = problem.cumulativeLeadTime(n - 2);
        const int sinkLayer = n - 1;
        for (int a = 0; a <= capLast; ++a) {
            const double nrt = a + problem.stages()[n - 1].leadTime();
            arcs.push_back({sinkLayer, a, 0, problem.safetyStockCost(sinkLayer, nrt), -1});
        }

        const int numVars = static_cast<int>(arcs.size());
        for (int i = 0; i < numVars; ++i) arcs[i].varIndex = i;

        OsiClpSolverInterface solver;

        std::vector<double> objective(numVars);
        std::vector<double> colLower(numVars, 0.0);
        std::vector<double> colUpper(numVars, 1.0);
        for (int i = 0; i < numVars; ++i) objective[i] = arcs[i].cost;

        CoinPackedMatrix matrix(false, 0, 0);
        matrix.setDimensions(0, numVars);
        std::vector<double> rowLower, rowUpper;

        // Exactly one arc leaves layer 0.
        {
            CoinPackedVector row;
            for (const Arc& arc : arcs) {
                if (arc.layer == 0) row.insert(arc.varIndex, 1.0);
            }
            matrix.appendRow(row);
            rowLower.push_back(1.0);
            rowUpper.push_back(1.0);
        }

        // Flow conservation at every node b of layer j (j = 1..n-2).
        for (int j = 1; j <= n - 2; ++j) {
            const int capJ = problem.cumulativeLeadTime(j);
            for (int b = 0; b <= capJ; ++b) {
                CoinPackedVector row;
                for (const Arc& arc : arcs) {
                    if (arc.layer == j && arc.a == b) row.insert(arc.varIndex, 1.0);
                    if (arc.layer == j - 1 && arc.b == b) row.insert(arc.varIndex, -1.0);
                }
                matrix.appendRow(row);
                rowLower.push_back(0.0);
                rowUpper.push_back(0.0);
            }
        }

        // Exactly one arc enters the sink.
        {
            CoinPackedVector row;
            for (const Arc& arc : arcs) {
                if (arc.layer == sinkLayer) row.insert(arc.varIndex, 1.0);
            }
            matrix.appendRow(row);
            rowLower.push_back(1.0);
            rowUpper.push_back(1.0);
        }

        solver.loadProblem(matrix, colLower.data(), colUpper.data(), objective.data(),
                            rowLower.data(), rowUpper.data());
        for (int v = 0; v < numVars; ++v) solver.setInteger(v);

        CbcModel model(solver);
        model.setLogLevel(0);
        model.branchAndBound();

        const double* sol = model.solver()->getColSolution();

        result.outboundServiceTime.assign(n - 1, 0);
        for (const Arc& arc : arcs) {
            if (sol[arc.varIndex] > 0.5 && arc.layer < sinkLayer) {
                result.outboundServiceTime[arc.layer] = arc.b;
            }
        }

        problem.validate(result);
        return result;
    }

    std::string name() const override { return "Cbc-MIP"; }
};

} // namespace meio

#endif // MEIO_USE_CBC

"""PulpMipSolver: layered-graph shortest-path MIP via PuLP/CBC.

Mirrors the C++ project's CbcMipSolver.h documentary formulation:
ExactDPSolver (exact_solver.py) is the zero-dependency solver used by
default and is already provably optimal in polynomial time, since the
serial GSM has optimal substructure -- no MIP solver is strictly needed
for correctness. This module linearizes the same DP into a MIP as an
alternative formulation, for portfolio-consistency with the other
projects that do need a MIP solver. Requires `pip install pulp`.
"""

from typing import List, NamedTuple

from .problem import EchelonSolution, MultiEchelonProblem
from .solver import EchelonSolver


class _Arc(NamedTuple):
    layer: int
    a: int
    b: int
    cost: float


def _build_arcs(problem: MultiEchelonProblem) -> List[_Arc]:
    # layer j holds one node per candidate outbound service time for
    # stage j; arc (layer, a, b) = "stage j-1 chose a AND stage j chose
    # b", costed via stage j's safety-stock formula (same recurrence as
    # ExactDPSolver). Infeasible/negative-NRT arcs are never generated.
    n = problem.stage_count()
    arcs: List[_Arc] = []

    cap0 = problem.cumulative_lead_time(0)
    for b in range(cap0 + 1):
        nrt = problem.stages[0].lead_time - b
        arcs.append(_Arc(0, 0, b, problem.safety_stock_cost(0, nrt)))

    for j in range(1, n - 1):
        cap_prev = problem.cumulative_lead_time(j - 1)
        cap_j = problem.cumulative_lead_time(j)
        for a in range(cap_prev + 1):
            for b in range(cap_j + 1):
                nrt = a + problem.stages[j].lead_time - b
                if nrt < 0.0:
                    continue
                arcs.append(_Arc(j, a, b, problem.safety_stock_cost(j, nrt)))

    cap_last = problem.cumulative_lead_time(n - 2)
    sink_layer = n - 1
    for a in range(cap_last + 1):
        nrt = a + problem.stages[n - 1].lead_time
        arcs.append(_Arc(sink_layer, a, 0, problem.safety_stock_cost(sink_layer, nrt)))

    return arcs


class PulpMipSolver(EchelonSolver):
    """Formulates the serial GSM DP as a shortest path on a layered
    graph, solved as a binary MIP via PuLP + CBC.
    """

    def solve(self, problem: MultiEchelonProblem) -> EchelonSolution:
        try:
            import pulp
        except ImportError as exc:
            raise RuntimeError(
                "PulpMipSolver requires the 'pulp' package: pip install pulp"
            ) from exc

        n = problem.stage_count()
        result = EchelonSolution()
        if n <= 0:
            return result

        if n == 1:
            # No free variables at all -- nothing to hand to the MIP.
            problem.validate(result)
            return result

        arcs = _build_arcs(problem)
        sink_layer = n - 1

        model = pulp.LpProblem("meio_gsm_shortest_path", pulp.LpMinimize)
        x = [
            pulp.LpVariable(f"arc_{i}", cat="Binary") for i in range(len(arcs))
        ]

        model += pulp.lpSum(arcs[i].cost * x[i] for i in range(len(arcs)))

        # Exactly one arc leaves layer 0.
        model += pulp.lpSum(x[i] for i, a in enumerate(arcs) if a.layer == 0) == 1

        # Flow conservation at every node b of layer j (j = 1..n-2).
        for j in range(1, n - 1):
            cap_j = problem.cumulative_lead_time(j)
            for b in range(cap_j + 1):
                inflow = pulp.lpSum(
                    x[i] for i, a in enumerate(arcs) if a.layer == j - 1 and a.b == b
                )
                outflow = pulp.lpSum(
                    x[i] for i, a in enumerate(arcs) if a.layer == j and a.a == b
                )
                model += outflow - inflow == 0

        # Exactly one arc enters the sink.
        model += pulp.lpSum(x[i] for i, a in enumerate(arcs) if a.layer == sink_layer) == 1

        model.solve(pulp.PULP_CBC_CMD(msg=0))

        outbound = [0] * (n - 1)
        for i, a in enumerate(arcs):
            if x[i].value() is not None and x[i].value() > 0.5 and a.layer < sink_layer:
                outbound[a.layer] = a.b
        result.outbound_service_time = outbound

        problem.validate(result)
        return result

    def name(self) -> str:
        return "Pulp-MIP"

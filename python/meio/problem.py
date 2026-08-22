"""MultiEchelonProblem: serial GSM safety-stock placement problem."""

import math
from dataclasses import dataclass, field
from typing import List

from .echelon_node import EchelonNode


@dataclass
class EchelonSolution:
    """The result of solving a MultiEchelonProblem: the outbound service
    time chosen for each stage that has a free decision (all stages
    except the last, whose outbound service time is fixed at 0 by the
    make-to-stock assumption), plus the resulting feasibility and total
    safety-stock cost.
    """

    feasible: bool = True
    total_cost: float = 0.0

    # Outbound service time S_i for stage i, i = 0..N-2 (0-based stage
    # index). Length is len(stages) - 1, or empty when there is only
    # one stage (no free decision variables at all in that case).
    outbound_service_time: List[int] = field(default_factory=list)


class MultiEchelonProblem:
    """A serial (linear-chain) multi-echelon inventory problem in the
    style of the Graves & Willems (2000) guaranteed-service model (GSM),
    specialized to a single serial path of stages -- see README.md for
    the full mathematical model. Stages are stored upstream (index 0,
    e.g. Plant) to downstream (index N-1, the customer-facing node).

    Demand variability is a single property of the end-customer demand
    stream propagating through the chain, so it is shared across all
    stages rather than modeled per stage: one `demand_std_dev_per_period`
    (sigma) and one `service_level_z` (z) apply to every stage's
    safety-stock formula.
    """

    def __init__(
        self,
        stages: List[EchelonNode],
        demand_std_dev_per_period: float,
        service_level_z: float,
    ) -> None:
        self.stages: List[EchelonNode] = list(stages)
        self.demand_std_dev_per_period = demand_std_dev_per_period
        self.service_level_z = service_level_z

    def stage_count(self) -> int:
        return len(self.stages)

    def cumulative_lead_time(self, stage_index: int) -> int:
        """Sum of lead_time over stages[0..stage_index] inclusive
        (0-based). Used as a safe, simple upper bound on the candidate
        range for outbound service time S_stage_index: any feasible S_i
        can never usefully exceed this, since net replenishment time
        can't go negative anywhere upstream of it.
        """
        if stage_index < 0 or stage_index >= self.stage_count():
            raise IndexError(
                "MultiEchelonProblem.cumulative_lead_time: stageIndex out of range"
            )
        return sum(self.stages[i].lead_time for i in range(stage_index + 1))

    def safety_stock_cost(self, stage_index: int, nrt: float) -> float:
        """Safety-stock holding cost contribution of a single stage,
        given its net replenishment time (NRT):
        holding_cost_i * z * sigma * sqrt(nrt). `nrt` must be >= 0 (a
        negative NRT is an infeasible plan and has no meaningful cost --
        callers must check feasibility separately).
        """
        if stage_index < 0 or stage_index >= self.stage_count():
            raise IndexError(
                "MultiEchelonProblem.safety_stock_cost: stageIndex out of range"
            )
        # nrt is expected to be >= 0; clamp tiny negative floating-point
        # noise from a feasible-but-boundary NRT rather than feeding
        # sqrt() a negative number, which would raise.
        safe_nrt = nrt if nrt > 0.0 else 0.0
        return (
            self.stages[stage_index].holding_cost_per_unit
            * self.service_level_z
            * self.demand_std_dev_per_period
            * math.sqrt(safe_nrt)
        )

    def validate(self, solution: EchelonSolution) -> None:
        """Recomputes net replenishment times from
        `solution.outbound_service_time` and (re)sets
        `solution.feasible` and `solution.total_cost` in place. A
        solution is feasible iff every stage's net replenishment time is
        >= 0 and outbound_service_time has the expected size
        (stage_count()-1, or empty for a single-stage problem).
        """
        n = self.stage_count()
        expected_free_vars = n - 1 if n > 1 else 0

        if n == 0 or len(solution.outbound_service_time) != expected_free_vars:
            solution.feasible = False
            solution.total_cost = 0.0
            return

        def outbound_service_time_of(i: int) -> int:
            # Outbound service time of stage i (0-based): the decision
            # variable for i = 0..n-2, fixed at 0 for the last stage
            # (make-to-stock: it must always be ready to serve the
            # customer immediately).
            return solution.outbound_service_time[i] if i < n - 1 else 0

        feasible = True
        total_cost = 0.0
        incoming_service_time = 0  # SI_0 = 0: stage 0 has immediate access to inputs

        for i in range(n):
            outbound = outbound_service_time_of(i)
            nrt = incoming_service_time + self.stages[i].lead_time - outbound
            if nrt < -1e-9:
                feasible = False
            total_cost += self.safety_stock_cost(i, nrt)
            incoming_service_time = outbound  # SI_{i+1} = S_i

        solution.feasible = feasible
        solution.total_cost = total_cost

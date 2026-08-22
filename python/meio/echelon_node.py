"""EchelonNode: a single stage in a serial supply chain."""

from dataclasses import dataclass


@dataclass(frozen=True)
class EchelonNode:
    """One stage ("echelon") in a serial supply chain, ordered upstream
    (e.g. Plant) to downstream (e.g. Local DC / customer-facing node).
    """

    id: int
    name: str
    lead_time: int
    """This stage's own replenishment/processing lead time, in days."""
    holding_cost_per_unit: float
    """Holding cost per unit of safety stock held at this stage."""

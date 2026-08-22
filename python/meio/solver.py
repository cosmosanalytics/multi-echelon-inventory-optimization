"""EchelonSolver: abstract strategy interface for safety-stock placement."""

from abc import ABC, abstractmethod

from .problem import EchelonSolution, MultiEchelonProblem


class EchelonSolver(ABC):
    """Any algorithm capable of placing safety stock on a
    MultiEchelonProblem implements this. Lets the demo / tests swap
    solvers (a fast heuristic, an exact solver, or an external MIP
    solver) without changing any calling code.
    """

    @abstractmethod
    def solve(self, problem: MultiEchelonProblem) -> EchelonSolution:
        ...

    @abstractmethod
    def name(self) -> str:
        ...

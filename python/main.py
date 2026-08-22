"""Demo: sample 3-stage chain, compare solvers. Mirrors src/main.cpp."""

from meio import AllLocalHeuristicSolver, EchelonNode, ExactDPSolver, MultiEchelonProblem


def print_solution(problem: MultiEchelonProblem, solver_name: str, solution) -> None:
    print(f"\n--- {solver_name} ---")
    print(f"Feasible: {'yes' if solution.feasible else 'no'}")
    print(f"Total safety-stock cost: ${solution.total_cost:.4f}")

    print("Outbound service times:", end="")
    if not solution.outbound_service_time:
        print(" (none -- single stage, no free decision variables)")
    else:
        print()
        for i, s in enumerate(solution.outbound_service_time):
            print(f"  {problem.stages[i].name} -> S = {s} day(s)")
        print(
            f"  {problem.stages[-1].name} -> S = 0 day(s) "
            "(fixed: make-to-stock, must serve customer immediately)"
        )


def main() -> None:
    # A representative 3-echelon chain: a Plant that manufactures, a
    # Regional DC that consolidates, and a Local DC that faces the
    # customer directly. Lead times and holding costs both increase
    # moving downstream, which is typical: downstream nodes add their
    # own processing/transportation time on top of upstream lead time,
    # and carry higher per-unit holding cost because more value has
    # been added to the product by the time it sits there.
    stages = [
        EchelonNode(1, "Plant", lead_time=7, holding_cost_per_unit=2.0),
        EchelonNode(2, "Regional DC", lead_time=3, holding_cost_per_unit=4.0),
        EchelonNode(3, "Local DC / Customer", lead_time=2, holding_cost_per_unit=7.0),
    ]
    demand_std_dev_per_period = 25.0  # units/day, end-customer demand
    service_level_z = 1.65  # ~95% cycle service level

    problem = MultiEchelonProblem(stages, demand_std_dev_per_period, service_level_z)

    print("Multi-Echelon Inventory Optimization (Python) -- Serial Guaranteed-Service Model")
    print(
        f"{len(stages)} stages, demand sigma={demand_std_dev_per_period}, "
        f"service level z={service_level_z}"
    )
    for s in stages:
        print(
            f"  {s.name}: leadTime={s.lead_time} day(s), "
            f"holdingCost=${s.holding_cost_per_unit}/unit"
        )

    heuristic = AllLocalHeuristicSolver()
    heuristic_solution = heuristic.solve(problem)
    print_solution(
        problem, heuristic.name() + " (baseline, no coordination)", heuristic_solution
    )

    exact = ExactDPSolver()
    exact_solution = exact.solve(problem)
    print_solution(problem, exact.name() + " (exact, polynomial-time DP)", exact_solution)

    if heuristic_solution.total_cost > 0.0:
        savings_pct = (
            100.0
            * (heuristic_solution.total_cost - exact_solution.total_cost)
            / heuristic_solution.total_cost
        )
        print(
            "\nCoordinating service times across the chain reduces safety-stock "
            f"cost by {savings_pct:.1f}% versus the no-coordination baseline."
        )

    try:
        import pulp  # noqa: F401
        from meio.pulp_solver import PulpMipSolver

        pulp_solver = PulpMipSolver()
        pulp_solution = pulp_solver.solve(problem)
        print_solution(
            problem, pulp_solver.name() + " (layered-graph MIP via CBC)", pulp_solution
        )
    except ImportError:
        print("\n(Skipping PuLP-MIP solver: 'pulp' is not installed.)")


if __name__ == "__main__":
    main()

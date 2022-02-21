using Revise
using gtpmip

grounded_plan = solve_hpd("experiments/real/domain.hpd", 
                        "experiments/real/serve_problem.hpd"; max_levels=400)
grounded_plan.plan
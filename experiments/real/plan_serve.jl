using Revise
using gtpmip

grounded_plan = solve_hpd("experiments/real/domain.hpd", 
                        "experiments/real/serve_problem.hpd"; max_levels=400, has_placement_constraint=false)
grounded_plan.plan
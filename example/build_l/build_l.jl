using Revise
using gtpmip

grounded_plan = solve_hpd("example/build_l/domain.hpd", "example/build_l/problem.hpd"; max_levels=200)
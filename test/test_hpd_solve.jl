using Revise
using gtpmip

graph = solve_hpd("test/hpd/domain.hpd", "test/hpd/problem.hpd"; max_levels=200)
1
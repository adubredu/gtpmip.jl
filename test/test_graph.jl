using Revise
using mitamp

graph = solve_hfg("test/pddl/hdomain.pddl", "test/pddl/hp1.pddl"; max_levels=100)

# 1
using Revise
using mitamp

sol = gpip_solve("test/pddl/domain.pddl", "test/pddl/hard_problem.pddl"; max_levels=1000)
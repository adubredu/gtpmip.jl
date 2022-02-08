using Revise
using gtpmip
using westbrick

grounded_plan = solve_hpd("example/build_u/domain.hpd", "example/build_u/problem.hpd"; max_levels=300)
grounded_plan.plan

num_objects = 19
inits = grounded_plan.graph.initprops[:continuous][1]
xinits = [Symbol("xb"*string(i)) for i=1:num_objects]
yinits = [Symbol("yb"*string(i)) for i=1:num_objects]
init_poses = [[inits[x], inits[y]] for (x,y) in zip(xinits, yinits)]

objects = init_environment(num_objects, init_poses)
obs_dict, ax, fig = visualize_environment!(objects) 
bobby = load_robot()
traj, obj_traj = init_visualization()
objname_dict = Dict()
for i=1:num_objects objname_dict[Symbol("b"*string(i))]=i end

for i = 1:grounded_plan.size
    pose = grounded_plan.robot_poses[i]
    action = grounded_plan.plan[i]
    move!(bobby, pose, traj, obj_traj) 
    if action[1] == :pick 
        # object_id = parse(Int, string(action[2][1])[end]) 
        object_id = objname_dict[action[2][1].name]
        pick!(bobby, -π/2., objects[object_id], traj, obj_traj)
    elseif action[1] == :place 
        object_id = string(action[2][1])[end]
        place!(bobby, -π/2., traj, obj_traj)
    end
end

println("visualizing...")
visualize_trajectory!(bobby, traj, obj_traj, obs_dict, ax, fig;name="media/build_u.gif")

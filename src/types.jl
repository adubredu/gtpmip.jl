mutable struct Soln 
    plan::Vector{Tuple{Symbol,Vector{Any}}} 
    robot_poses::Vector{Vector{Float64}}  
    graph::Graph
end
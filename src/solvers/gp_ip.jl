#= 
f is per t in planning graph 
x0 goes with y1, xt-1 goes with yt
no-op are part of y


1. init and goal constraints 
=#

function get_fluent_terminal_indices(graph, max_nf; loc=:init) 
    indices = []
    not_ind = [] 
    T = graph.num_levels - 1 
    if loc == :init
        for i=1:length(graph.initprops) 
            ind = findall(x->x == graph.initprops[i], graph.props[0])[1]
            push!(indices, ind)
        end
    else
        for i=1:length(graph.goalprops) 
            ind = findall(x->x == graph.goalprops[i], graph.props[T])[1]
            push!(indices, ind)
        end 
    end
    not_ind = [i for i=1:max_nf if !(i in indices)]
    return indices, not_ind    
end


function get_actions_precondition_sat(fluent, level, graph)
    actions = graph.acts[level]
    # actions = [act for act in graph.acts[level] if act.name != :NoOp]
    act_inds = []
    for i=1:length(actions)
        act = graph.acts[level][i]
        if fluent in act.pos_prec 
            push!(act_inds, i)
        end
    end
    return act_inds
end


function get_actions_effect_sat(fluent, level, graph)
    actions = graph.acts[level]
    # actions = [act for act in graph.acts[level] if act.name != :NoOp]
    act_inds = []
    for i=1:length(actions)
        act = graph.acts[level][i]
        if fluent in act.pos_eff 
            push!(act_inds, i)
        end
    end
    return act_inds
end


function get_fluent_index(fluent, level, graph)
    for i=1:length(graph.props[level])
        if graph.props[level][i] == fluent 
            return i 
        end
    end
    return nothing
end
 

function get_mutex_actions(fluent, level, graph) 
    pref = get_actions_precondition_sat(fluent, level, graph)
    preneff = union(pref, get_actions_effect_sat(fluent, level, graph))
    act_mutexes = []
    [push!(act_mutexes, [i,j]) for i in pref for j in preneff if i!=j && !([j,i] in act_mutexes)]

    return act_mutexes
end


function get_mutex_indices(level, graph)
    act_mutexes = []
    for pair in graph.μacts[level]
        i = findall(x->x == pair[1], graph.acts[level])[1]
        j = findall(x->x == pair[2], graph.acts[level])[1]
         
        if !([j,i] in act_mutexes) push!(act_mutexes, [i,j]) end
    end
    return act_mutexes
end


function test_mutexes(domain_name, problem_name; max_levels=10)
    domain = load_domain(domain_name)
    problem = load_problem(problem_name)
    graph = create_graph(domain, problem; max_levels=max_levels)
    T = graph.num_levels
    for t = 1:T-1 #xt, yt, bk: xt+1
        println("\n\nLevel: ",t)
        println("------------------")
        for f=1:length(graph.props[t-1])
            fluent = graph.props[t-1][f]
            # println("fluent: ",fluent)
            m1 = get_mutex_actions(fluent, t, graph)
            m2 = get_mutex_actions2(fluent, t, graph)
            # println("m1: ",m1)
            println("m2: ",m2)
            println(" ")
        end
        println("gp mutexes:")
        println(get_mutex_indices(t, graph))
    end
    
end



function gpip_solve(domain_name, problem_name; max_levels=10)
    domain = load_domain(domain_name)
    problem = load_problem(problem_name)
    graph = create_graph(domain, problem; max_levels=max_levels)

    model = Model(Gurobi.Optimizer)
    set_silent(model)

    T = graph.num_levels 
    max_nf = max([length(graph.props[i-1]) for i=1:T]...) 
    max_na = max([length(graph.acts[i]) for i=1:T-1]...) 
    # max_na = max([length([act for act in graph.acts[i] if act.name != :NoOp]) for i=1:T-1]...) 

    @variable(model, x[i=1:max_nf, j=1:T], Bin)
    @variable(model, y[i=1:max_na, j=1:T-1], Bin)

    inits, notinits = get_fluent_terminal_indices(graph, max_nf; loc=:init)
    [@constraint(model, x[f, 1] == 1) for f in inits]
    [@constraint(model, x[f, 1] == 0) for f in notinits]

    goals, _ = get_fluent_terminal_indices(graph, max_nf; loc=:goal) 
    println("goals: ",goals)
    [@constraint(model, x[f, T] == 1) for f in goals]
    [@constraint(model, x[f, T] == 0) for f = 1:max_nf if !(f in goals)]

    

    for t = 1:T-1 #xt, yt, bk: xt+1
        for f=1:length(graph.props[t-1])
            fluent = graph.props[t-1][f]
            act_inds = get_actions_precondition_sat(fluent, t, graph)
            [@constraint(model, y[a,t] <= x[f, t]) for a in act_inds]
        end
    end

    for t = 1:T-1 #xt, yt, bk: xt+1
        for f=1:length(graph.props[t]) 
            fluent = graph.props[t][f]
            act_inds = get_actions_effect_sat(fluent, t, graph)
            fnext = get_fluent_index(fluent, t, graph) 
            if !isnothing(fnext) && !isempty(act_inds)
                @constraint(model, x[fnext, t+1] <= sum([y[a, t] for a in act_inds])) 
            end  

            # act_pairs = get_mutex_actions(fluent, t, graph)
            # [@constraint(model, y[a[1], t] + y[a[2], t] <= 1) for a in act_pairs]
        end 
        # na = length([act for act in graph.acts[t] if act.name != :NoOp])
        na = length(graph.acts[t])
        @constraint(model, y[na+1:max_na, t] .== 0) 
    end

    for t = 1:T-1 #xt, yt, bk: xt+1
        for f=1:length(graph.props[t])
            fluent = graph.props[t][f]
            act_pairs = get_mutex_actions(fluent, t, graph)
            [@constraint(model, y[a[1], t] + y[a[2], t] <= 1) for a in act_pairs]
        end
    end  

    @objective(model, Min, sum(y))
    optimize!(model)
    sol = value.(y)
    render_action(sol, graph)
    # graph

end

function render_action(y, graph)
    T = graph.num_levels-1
    actions = []
    for t=1:T 
        # try
        ind = findall(x->x == 1.0, y[1:end,t])[1]
        act  = graph.acts[t][ind] 
        # act = all_actions[ind]
        push!(actions, (act.name, act.args))
        # catch 
        # end
    end
    # println(actions)
    return actions

end
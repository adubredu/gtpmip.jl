function solve_hpd(domain_name, problem_name; max_levels=20)
    graph = create_funnel_graph(domain_name, problem_name; max_levels=max_levels)

    model = Model(Gurobi.Optimizer)
    set_silent(model)

    T = graph.num_levels
    max_nf = max([length([prop for prop in graph.props[i][:discrete]]) for i=1:T]...)  
    max_na = max([length([act for act in graph.acts[i] ]) for i=1:T-1]...) 

    @variable(model, p[i=1:max_nf, j=1:T], Bin)
    @variable(model, a[i=1:max_na, j=1:T-1], Bin)
    @variable(model, xᵣ[i=1:2, j=1:T])

    # init constraints
    num_init_props = length(graph.props[1][:discrete])
    [@constraint(model, p[i, 1] == 1) for i = 1: num_init_props]
    [@constraint(model, p[i, 1] == 0) for i = num_init_props+1 : max_nf]
    
    # goal constraints
    goalinds = get_fluent_terminal_indices(graph; loc=:goals)
    [@constraint(model, p[f, T] == 1) for f in goalinds]
 
    # precondition constraints
    for t = 1:T-1
        fluents = graph.props[t][:discrete]
        for f = 1:length(fluents)
            fluent = fluents[f]
            act_inds = get_actions_precondition_sat(fluent, t, graph)
            [@constraint(model, a[i,t] ≤ p[f,t]) for i in act_inds]
        end
    end

    # effect constraints
    for t = 1:T-1
        fluents = graph.props[t+1][:discrete]
        for f = 1:length(fluents)
            fluent = fluents[f]
            act_inds = get_actions_effect_sat(fluent, t, graph)
            fnext = get_fluent_index(fluent, t+1, graph)
            if !isnothing(fnext) && !isempty(act_inds)
                @constraint(model, p[fnext, t+1] ≤ sum([a[i,t] for i in act_inds]))
            end
        end
        na = length(graph.acts[t])
        @constraint(model, a[na+1:max_na, t] .== 0)
        # for i = 1:na
        #     if graph.acts[t][i].name == :noop  && i<=max_na 
        #         @constraint(model, a[i, t] == 0)
        #     end
        # end
        # @constraint(model, sum(a[:, t])==1)
    end

    # mutex constraints
    for t = 1:T-1  
        for f=1:length(graph.props[t][:discrete])
            fluent = graph.props[t][:discrete][f]
            act_pairs = get_mutex_actions(fluent, t, graph)
            [@constraint(model, a[i[1], t] + a[i[2], t] <= 1) for i in act_pairs]
        end
    end 
    #mutex constraint 
    @objective(model, Min, sum(a))
    optimize!(model)
    sol = value.(a)
    # graph
    render_action(sol, graph)
end

function get_fluent_terminal_indices(graph::Graph; loc=:init)
    indices = Set()  
    T = graph.num_levels 
    if loc == :init
        for i=1:length(graph.initprops[:discrete]) 
            ind = findall(x->x == graph.initprops[:discrete], graph.props[1][:discrete])[1]
            push!(indices, ind)
        end
    else
        for i=1:length(graph.goalprops[:discrete]) 
            ind = findall(x->x == graph.goalprops[:discrete][i], graph.props[T][:discrete])[1] 
            push!(indices, ind)
        end 
    end
    return collect(indices)
end

function get_actions_precondition_sat(fluent, level::Int, graph::Graph)
    actions = [act for act in graph.acts[level] ]
    act_inds = []
    for i = 1:length(actions)
        act = graph.acts[level][i]
        if fluent in act.pos_prec  push!(act_inds, i) end 
    end
    return act_inds
end

function get_actions_effect_sat(fluent, level::Int, graph::Graph)
    actions = [act for act in graph.acts[level] ]
    act_inds = []
    for i = 1:length(actions)
        act = graph.acts[level][i]
        if fluent in act.pos_eff push!(act_inds, i) end 
    end
    return act_inds
end

function get_fluent_index(fluent, level::Int, graph::Graph)
    fluents = graph.props[level][:discrete]
    for i = 1:length(fluents)
        if fluents[i] == fluent return i end 
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

function render_action(y, graph::Graph)
    T = graph.num_levels
    actions = []
    for t=1:T-1  
        ind = findall(x->x == 1.0, y[1:end,t])
        id = ind[1]
        # for i in ind if graph.acts[t][i].name != :noop id = i; break; end end
        act  = graph.acts[t][id]  
        push!(actions, (act.name, act.params)) 
    end 
    return actions

end
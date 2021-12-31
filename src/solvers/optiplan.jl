function get_action_indices(graph, fluent, level, set, except_set, all_actions) 
    actions = [act for act in graph.acts[level] if act.name != :NoOp]
    # actions = all_actions
    action_indices = []
    if set == :add && except_set == :pre
        for i = 1:length(actions)
            act = actions[i]  
            if fluent in act.pos_eff && !(fluent in act.pos_prec)
                push!(action_indices, i)
            end
        end
    elseif set == :del && except_set == :pre 
        for i = 1:length(actions)
            act = actions[i]  
            if fluent in act.neg_eff && !(fluent in act.pos_prec)
                push!(action_indices, i)
            end
        end
    elseif set == :pre && except_set == :del 
        for i = 1:length(actions)
            act = actions[i]  
            if fluent in act.pos_prec && !(fluent in act.neg_eff)
                push!(action_indices, i)
            end
        end
    else
        for i = 1:length(actions)
            act = actions[i]  
            if fluent in act.pos_prec && fluent in act.neg_eff
                push!(action_indices, i)
            end
        end
    end
    return action_indices 
end


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


function get_fluent_index(fluent, level, graph)
    for i=1:length(graph.props[level])
        if graph.props[level][i] == fluent 
            return i 
        end
    end
    return nothing
end


function optiplan_solve(domain_name, problem_name; max_levels=10)
    domain = load_domain(domain_name)
    problem = load_problem(problem_name)
    graph = create_graph(domain, problem; max_levels=max_levels)
    all_actions = mutex.get_all_actions(domain, problem)
    
    model = Model(Gurobi.Optimizer)
    set_silent(model)

    T = graph.num_levels 
    max_nf = max([length(graph.props[i-1]) for i=1:T]...) 
    max_na = max([length([act for act in graph.acts[i] if act.name != :NoOp]) for i=1:T-1]...) 
    # max_na = length(all_actions)
    xs = [] 
    pa, pd, ad, dl, mn = 1,2,3,4,5 
    
    #=
        VARIABLES 
    =#
    [push!(xs, @variable(model, [i=1:max_nf, j=1:5], Bin, start=0)) for _=1:T]
    @variable(model, y[i=1:max_na, j=1:T-1], Bin, start=0)
    
    #=
        CONSTRAINTS
    =#
    #Eq. 2
    inits, notinits = get_fluent_terminal_indices(graph, max_nf; loc=:init)
    t=1 
    #println("\ninits: ",inits)
    #println("notinits: ",notinits)
    for i in inits
        @constraint(model, xs[t][i, ad] == 1)
        @constraint(model, xs[t][i, pa] == 0)
        @constraint(model, xs[t][i, pd] == 0) 
        @constraint(model, xs[t][i, mn] == 0) 
        @constraint(model, xs[t][i, dl] == 0)
    end 
    
    #Eq. 3
    for i in notinits
        @constraint(model, xs[t][i, ad] == 0)
        @constraint(model, xs[t][i, mn] == 0) 
        @constraint(model, xs[t][i, pa] == 0) 
        @constraint(model, xs[t][i, dl] == 0) 
        @constraint(model, xs[t][i, pd] == 0)
    end

    #Eq. 4
    goals, _ = get_fluent_terminal_indices(graph, max_nf; loc=:goal)
    [@constraint(model, xs[T][i, ad] + xs[T][i, mn] + xs[T][i, pa] >= 1) for i in goals]

    # println("\nAll actions: ",all_actions)
    for t=1:T-1
        # println("\n\nLevel ",t)
        # println("\nAll actions: ",all_actions)  
        for f = 1:length(graph.props[t])
            fluent = graph.props[t][f]
            # println("\nFluent: ",fluent)
            
            # Eq. 5
            act_inds = get_action_indices(graph, fluent, t, :add, :pre, all_actions)
            # println("Eq. 5 indices: ",act_inds)
            # println("Eq.5 actions: ",[all_actions[i] for i in act_inds])
            if !isempty(act_inds)
                @constraint(model, sum([y[i, t] for i in act_inds]) >= xs[t+1][f, ad] )
                # Eq. 6
                # [@constraint(model, y[i, t] <= xs[t+1][f, ad]) for i in act_inds]
                for i in act_inds
                    @constraint(model, y[i, t] <= xs[t+1][f, ad])
                end
            end
            
            #Eq. 7
            act_inds = get_action_indices(graph, fluent, t, :del, :pre, all_actions)
            # println("Eq. 7 indices: ",act_inds)
            # println("Eq.7 actions: ",[all_actions[i] for i in act_inds])
            if !isempty(act_inds)
                @constraint(model, sum([y[i, t] for i in act_inds]) >= xs[t+1][f, dl] )
                # Eq. 8
                # [@constraint(model, y[i, t] <= xs[t+1][f, dl]) for i in act_inds]
                for i in act_inds
                    @constraint(model, y[i, t] <= xs[t+1][f, dl])
                end
            end

            #Eq. 9
            act_inds = get_action_indices(graph, fluent, t, :pre, :del, all_actions)
            # println("Eq. 9 indices: ",act_inds)
            # println("Eq.9 actions: ",[all_actions[i] for i in act_inds])
            if !isempty(act_inds)
                @constraint(model, sum([y[i, t] for i in act_inds]) >= xs[t+1][f, pa] )
                # Eq. 10
                # [@constraint(model, y[i, t] <= xs[t+1][f, pa]) for i in act_inds]
                for i in act_inds
                    @constraint(model, y[i, t] <= xs[t+1][f, pa])
                end
            end

            #Eq. 11
            act_inds = get_action_indices(graph, fluent, t, :pred, :anddel, all_actions)

            # println("Eq. 11 indices: ",act_inds)
            # println("Eq.11 actions: ",[all_actions[i] for i in act_inds])
            if !isempty(act_inds)
                @constraint(model, sum([y[i, t] for i in act_inds]) == xs[t+1][f, pd] ) 
            end

            #Eq. 12
            @constraint(model, xs[t+1][f, ad] + xs[t+1][f, mn] + xs[t+1][f, dl] + xs[t+1][f, pd] <= 1)

            #Eq 13
            @constraint(model, xs[t+1][f, pa] + xs[t+1][f, mn] + xs[t+1][f, dl] + xs[t+1][f, pd] <= 1)

            #Eq. 14
            fprev = get_fluent_index(fluent, t-1, graph)
            if !isnothing(fprev)
            @constraint(model, xs[t+1][f, pa] + xs[t+1][f, mn] + xs[t+1][f, pd] <= xs[t][fprev, pa] + xs[t][fprev, ad] + xs[t][fprev, mn]) 
            end
        end 
        num_actions = length([act for act in graph.acts[t] if act.name != :NoOp])
        @constraint(model, sum(y[1:num_actions, t ]) == 1) 

        na = length([act for act in graph.acts[t] if act.name != :NoOp])
        println(na)
        for i=na+1:max_na 
            @constraint(model, y[i,t] == 0)
        end
    end 
    @objective(model, Min, sum(y))
    optimize!(model)
    sol = value.(y)
    # println(model)
     
    
    # (graph, value.(x), value.(y))
    # for x in xs
    #     @show value.(x)
    # end
    # render_action(sol, graph, all_actions)
    # graph
end

function render_action(y, graph, all_actions)
    T = graph.num_levels-1
    actions = []
    for t=1:T 
        try
        ind = findall(x->x == 1.0, y[1:end,t])[1]
        act  = graph.acts[t][ind] 
        # act = all_actions[ind]
        push!(actions, (act.name, act.args))
        catch 
        end
    end
    # println(actions)
    return actions

end
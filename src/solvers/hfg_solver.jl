import Base: ==
function solve_hfg(domain_name, problem_name; max_levels=10)
    graph = create_funnel_graph(domain_name, problem_name; max_levels=max_levels)

    model = Model(Gurobi.Optimizer)
    set_silent(model)

    T = graph.num_levels-1
    max_nf = max([length(graph.props[i][:discrete])+length(graph.props[i][:continuous]) for i=1:T]...) 
    max_na = max([length(graph.acts[i]) for i=1:T-1]...)

    @variable(model, p[i=1:max_nf, j=1:T], Bin)
    @variable(model, a[i=1:max_na, j=1:T-1], Bin)

    @variable(model, xᵣ[i=1:2, j=1:T])
    @variable(model, xₒ[i=1:2, j=1:T])

    #inits 
    num_init_props = length(graph.initprops)
    [@constraint(model, p[i, 1] == 1) for i = 1 : num_init_props]
    [@constraint(model, p[i, 1] == 0) for i = num_init_props+1 : max_nf]
    @constraint(model, xᵣ[1, 1] == 0.25)
    @constraint(model, xᵣ[2, 1] == 0.25)
    @constraint(model, xₒ[1, 1] == 10.5)
    @constraint(model, xₒ[2, 1] == 10.5)

    #goals 
    goalinds = get_fluent_terminal_indices(graph; loc=:goals) 
    [@constraint(model, p[f, T] == 1) for f in goalinds]
    [@constraint(model, p[f, T] == 0) for f=1:max_nf if !(f in goalinds)]
    @constraint(model, xᵣ[1, T] == 40.5)
    @constraint(model, xᵣ[2, T] == 40.5)
    @constraint(model, xₒ[1, T] == 20.5)
    @constraint(model, xₒ[2, T] == 20.5)

    #constraints
    #precondition constraint
    for t=1:T-1
        fluents = get_fluents(graph, t)
        for f=1:length(fluents)
            fluent = fluents[f]
            act_inds = get_actions_precondition_sat(fluent, t, graph)
            [@constraint(model, a[i,t] ≤ p[f,t]) for i in act_inds]
        end
    end

    #effect constraint 
    for t=1:T-1
        fluents = get_fluents(graph, t+1)
        for f=1:length(fluents)
            fluent = fluents[f]
            act_inds = get_actions_effect_sat(fluent, t, graph)
            fnext = get_fluent_index(fluent, t, graph)
            if !isnothing(fnext) && !isempty(act_inds)
                @constraint(model, p[fnext, t+1] ≤ sum([a[i,t] for i in act_inds]))
            end
        end
        na = length(graph.acts[t])
        @constraint(model, a[na+1:max_na, t] .== 0)
    end

    #mutex constraint 
    for t=1:T-1
        fluents = get_fluents(graph, t)
        for f=1:length(fluents)
            fluent = fluents[f]
            act_pairs = get_mutex_actions(fluent, t, graph)
            [@constraint(model, a[i[1], t] + a[i[2], t] ≤ 1) for i in act_pairs]
        end
    end

    #continuous constraints 
    #
    #

    @objective(model, Min, sum([norm(xᵣ[:,t]-xᵣ[:,t+1]) for t=1:T-2]))
    @time optimize!(model)
    sol = value.(y)
end


==(reg1::Region, reg2::Region) = reg1.r[1].ϕ₁ == reg2.r[1].ϕ₁ && reg1.r[1].ϕ₂ == reg2.r[1].ϕ₂ && reg1.r[1].ϕ₃ == reg2.r[1].ϕ₃ && reg1.r[1].þ == reg2.r[1].þ && 
reg1.r[2].ϕ₁ == reg2.r[2].ϕ₁ && reg1.r[2].ϕ₂ == reg2.r[2].ϕ₂ && reg1.r[2].ϕ₃ == reg2.r[2].ϕ₃ && reg1.r[2].þ == reg2.r[2].þ && 
reg1.r[3].ϕ₁ == reg2.r[3].ϕ₁ && reg1.r[3].ϕ₂ == reg2.r[3].ϕ₂ && reg1.r[3].ϕ₃ == reg2.r[3].ϕ₃ && reg1.r[3].þ == reg2.r[3].þ &&  
reg1.r[4].ϕ₁ == reg2.r[4].ϕ₁ && reg1.r[4].ϕ₂ == reg2.r[4].ϕ₂ && reg1.r[4].ϕ₃ == reg2.r[4].ϕ₃ && reg1.r[4].þ == reg2.r[4].þ

function get_fluent_terminal_indices(graph; loc=:init) 
    indices = []
    not_ind = [] 
    T = graph.num_levels - 1 
    if loc == :init
        for i=1:length(graph.initprops[:discrete]) 
            ind = findall(x->x == graph.initprops[:discrete], graph.props[0][:discrete])[1]
            push!(indices, ind)
        end
    else
        for i=1:length(graph.goalprops[:discrete]) 
            ind = findall(x->x == graph.goalprops[:discrete][i], graph.props[T][:discrete])[1] 
            push!(indices, ind)
        end 
        nf = length(graph.goalprops[:discrete])
        for i=nf+1:nf+length(graph.goalprops[:continuous])
            ind = findall(x-> intersects(x, graph.goalprops[:continuous][i-nf]), graph.props[T][:continuous])[1]
            push!(indices, ind+nf)
        end
    end 
    return indices   
end


function get_fluents(graph, level)
    fluents=[graph.props[level][:discrete]...,graph.props[level][:continuous]...]
    return fluents
end


function get_fluent_index(fluent, t, graph)
    fluents = get_fluents(graph, t)
    for i=1:length(fluents)
        if fluents[i] == fluent 
            return i 
        end
    end
    return nothing 
end


#gets discrete actions as well as funnels too
function get_actions_precondition_sat(fluent, level, graph)
end


#gets discrete actions as well as funnels too 
function get_actions_effect_sat(fluent, level, graph)
end


function get_mutex_actions(fluent, level, graph)
end


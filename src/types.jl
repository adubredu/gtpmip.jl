 function extract!(gr, g, plan, index, layered_plan) 
    if isempty(g)
        new_goals = Set()
        for action in plan 
            for proposition in action.pos_prec 
                push!(new_goals, proposition)
            end
        end
        extracted_plan = extract!(gr, new_goals, [], index-1, layered_plan)
        if isempty(extracted_plan)
            return nothing
        else
            layered_plan[index-1] = extracted_plan
            layered_plan[index] = plan 
            return plan 
        end
    else
        proposition = pop!(goal)
        resolvers = []
        for action in gr.acts[index]
            if proposition in action.pos_eff 
                if !isempty(plan)
                    mutex = false
                    for action2 in plan 
                        if (action, action2) in gr.μacts[index]
                            mutex = true 
                            break 
                        end
                    end
                    if !mutex push!(resolvers, action) end          
                else
                    push!(resolvers, action)
                end
            end
        end

        if isempty(resolvers) return nothing  end

        while !isempty(resolvers)
            resolver = pop!(resolvers)
            push!(plan, resolver)
            [delete!(g, eff) for eff in resolver.pos_eff]
            plan_result = extract!(gr, g, plan, index, layered_plan)
            if !isnothing(plan_result) 
                return plan_result 
            else
                deleteat!(plan, findall(x->x==resolver))
                push!(g, proposition)
            end
        end
        return nothing
    end
 end
 
 function plan(gr, g, domain, problem)
    index = gr.num_levels - 1
    layered_plan = Dict()
    if !issubset(g, gr.props[index]) return nothing end
    plan = extract!(gr, g, [], index, layered_plan)
    if !isnothing(plan) return layered_plan end

    while true 
        index +=1
        gr = expand!(domain, problem, gr)
        plan = extract!(gr, g, [], index, layered_plan)
        if !isnothing(plan)
            return layered_plan
        elseif gr.leveled
            return nothing 
        end
    end
end
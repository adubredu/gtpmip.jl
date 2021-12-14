function get_domain_problem_objects(domain_path, problem_path)
    domain = load_domain(domain_path)
    problem = load_problem(problem_path)

    return domain, problem 
end


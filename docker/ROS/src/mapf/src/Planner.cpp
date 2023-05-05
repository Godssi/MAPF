#include <mapf/Planner.h>

vec2PInt Planner::plan(vecPInt starts, vecPInt goals, int high_level_max_iter, int low_level_max_iter)
{
    this->starts = starts;
    this->goals = goals;
    return plan(high_level_max_iter, low_level_max_iter);
}

vec2PInt Planner::plan(int high_level_max_iter, int low_level_max_iter)
{
    aStarPlanner.set_low_level_max_iter(low_level_max_iter);
    this->agents = _assign(starts, goals);
    // this->agents = min_cost(starts, goals);

    Constraints constraints;
    bool tf = true;
    map<Agent, vecPInt> solution;

    modify_potential_map();
    for (Agent agent : agents) {
        solution[agent] = calculate_path(agent, constraints, {});
    }

    if (solution.size() == 1)
    {
        vec2PInt vP2Int;
        vP2Int.push_back(solution[*(agents.begin())]);
        return vP2Int;
    }

    vecCTNode open;
    for (Agent agent : agents)
    {
        if (solution[agent].size() == 0)
        {
            tf = false;
            break;
        }
    }
    if (tf == true)
    {
        CTNode node(constraints, solution);
        open.push_back(node);
    }

    int iter_ = 0;
    vector<thread> th;
    mutex mtx;

    while (!open.empty() && iter_ < high_level_max_iter) {
        iter_++;
        modify_potential_map();
        pair<vector<pairCTNode>, vector<vec2PInt>> results;

        for (auto iter = open.begin(); iter != open.end();) {
            th.clear();
            for (int core = 0; core < max_core && iter != open.end(); core++, iter++)
            {
                th.push_back(thread(&Planner::search_node, this, ref(*iter), ref(results), ref(mtx)));
            }
            for (auto& t : th)
            {
                t.join();
            }
        }
        open.clear();

        if (results.second.size() != 0) return results.second[0];
        for (auto iter = results.first.begin(); iter != results.first.end(); iter++) {
            if (iter->first.tr == true)
            {
                open.push_back(iter->first);
            }
            if (iter->second.tr == true)
            {
                open.push_back(iter->second);
            }
        }
    }

    pair<vector<pairCTNode>, vector<vec2PInt>> results;
    return results.second[0];
}

void Planner::search_node(CTNode& best, pair<vector<pairCTNode>, vector<vec2PInt>>& results, mutex& mtx)
{
    mtx.lock();
    CTNode copyNode = best;
    mtx.unlock();

    pair<vecAgent, int> val = validate_paths(this->agents, copyNode);

    Agent agent_i = ((val.first)[0]);
    Agent agent_j = ((val.first)[1]);
    int time_of_conflict = val.second;
    if (time_of_conflict == -1)
    {
        mtx.lock();
        results.second.push_back(reformat(this->agents, copyNode.solution));
        mtx.unlock();
        return;
    }

    Constraints agent_i_constraint = calculate_constraints(copyNode, agent_i, agent_j, time_of_conflict);
    Constraints agent_j_constraint = calculate_constraints(copyNode, agent_j, agent_i, time_of_conflict);

    vecPInt agent_i_path = recalculate_path(agent_i, agent_i_constraint, calculate_goal_times(copyNode, agent_i, agents), time_of_conflict);
    vecPInt agent_j_path = recalculate_path(agent_j, agent_j_constraint, calculate_goal_times(copyNode, agent_j, agents), time_of_conflict);

    map<Agent, vecPInt> solution_i = copyNode.solution;
    map<Agent, vecPInt> solution_j = copyNode.solution;
    solution_i[agent_i] = agent_i_path;
    solution_j[agent_j] = agent_j_path;

    CTNode node_i;
    CTNode node_j;

    bool tf = true;
    for (auto iter = solution_i.begin(); iter != solution_i.end(); iter++) {
        if (iter->second.size() == 0) {
            tf = false;
            break;
        }
    }
    if (tf == true) {
        node_i.constraint = agent_i_constraint;
        node_i.solution = solution_i;
        node_i.tr = true;
    }


    tf = true;
    for (auto iter = solution_j.begin(); iter != solution_j.end(); iter++) {
        if (iter->second.size() == 0) {
            tf = false;
            break;
        }
    }
    if (tf == true) {
        node_j.constraint = agent_j_constraint;
        node_j.solution = solution_j;
        node_j.tr = true;
    }

    mtx.lock();
    results.first.push_back(pairCTNode{node_i, node_j});
    mtx.unlock();
    return;
}

vecPAgent Planner::combination(vecAgent total_agent)
{
    vecPAgent result;

    for (int i = 0; i < total_agent.size() - 1; i++) {
        for (int j = i + 1; j < total_agent.size(); j++) {
            pairAgent res = { total_agent[i], total_agent[j] };
            result.push_back(res);
        }
    }

    return result;
}

pair<vecAgent, int> Planner::validate_paths(vecAgent agents, CTNode node)
{
    vecPAgent combi = combination(agents);
    for (int i = 0; i < combi.size(); i++)
    {
        int time_of_conflict = safe_distance(node.solution, combi[i].first, combi[i].second);
        if (time_of_conflict == -1)
            continue;
        vecAgent re1{ combi[i].first,combi[i].second };
        pair<vecAgent, int> re2{ re1,time_of_conflict };
        return re2;
    }

    vecAgent re1{ combi[0].first,combi[0].second };
    pair<vecAgent, int>re2{ re1,-1 };
    return re2;
}

int Planner::safe_distance(map<Agent, vecPInt> solution, Agent agent_i, Agent agent_j)
{
    int idx = -1; 
    int size = 0;
    vecPInt paths_i = solution[agent_i];
    vecPInt paths_j = solution[agent_j];
    vec2PInt self{ paths_i, paths_j };
    if (self[0].size() > self[1].size())
        size = static_cast<int>(self[1].size());
    else
        size = static_cast<int>(self[0].size());

    for (int i = 0; i < size; i++)
    {
        idx++;
        if (paths_i[i].first == -1 || paths_j[i].first == -1)
            continue;
        if (dist(paths_i[i], paths_j[i]) >= robot_radius)
            continue;
        return idx;
    }
    return -1;
}

double Planner::dist(pairInt point1, pairInt point2)
{
    double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
    return result;
}

Constraints Planner::calculate_constraints(CTNode& node, Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict) {
    vecPInt constrained_path = node.solution[constrained_agent];
    vecPInt unchanged_path = node.solution[unchanged_agent];
    pairInt pivot = unchanged_path[time_of_conflict];
    int conflict_end_time = time_of_conflict;

    while (true)
    {
        if (conflict_end_time >= constrained_path.size())   break;
        if (dist(constrained_path[conflict_end_time], pivot) >= 2 * robot_radius) break;
        conflict_end_time++;
    }

    return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}

map<int, setPInt> Planner::calculate_goal_times(CTNode& node, Agent& agent, vecAgent& agents)
{
    map<Agent, vecPInt> solution = node.solution;
    map<int, setPInt> goal_times;

    for (Agent& other_agent : agents) {
        if (other_agent == agent) continue;

        int time = static_cast<int>(solution[other_agent].size()) - 1;
        if (goal_times.find(time) == goal_times.end())
        {
            setPInt val{ solution[other_agent][time] };
            goal_times.insert({ time, val });
        }
        else
        {
            goal_times[time].insert(solution[other_agent][time]);
        }
    }

    return goal_times;
}

vecPInt Planner::calculate_path(Agent agent, Constraints& constraints, map<int, setPInt> goal_times)
{
    map<int, setPInt> a;
    map<int, setPInt> constraint = constraints.setdefault(agent, a);
    planResults[agent] = aStarPlanner.aStarPlan(agent.start, agent.goal, constraint, goal_times, dynamic_obstacle, 0);
    return planResults[agent];
}

vecPInt Planner::recalculate_path(Agent agent, Constraints& constraints, map<int, setPInt> goal_times, int& time_of_conflict)
{
    map<int, setPInt> a;
    map<int, setPInt> constraint = constraints.setdefault(agent, a);
    vecPInt dfadf = aStarPlanner.aStarPlan(agent.start, agent.goal, constraint, goal_times, dynamic_obstacle, 0);
    if (time_of_conflict > robot_radius + 2)
    {
        vecPInt tmp = aStarPlanner.aStarPlan(planResults[agent][time_of_conflict - (robot_radius + 2)], agent.goal, constraint, goal_times, dynamic_obstacle, time_of_conflict - (robot_radius + 2));
        planResults[agent].erase(planResults[agent].begin() + time_of_conflict - (robot_radius + 2) + 1, planResults[agent].end());
        move(tmp.begin(), tmp.end(), back_inserter(planResults[agent]));
    }
    else
    {
        planResults[agent] = aStarPlanner.aStarPlan(agent.start, agent.goal, constraint, goal_times, dynamic_obstacle, 0);
    }
    return planResults[agent];
}

vec2PInt Planner::reformat(vecAgent agents, map<Agent, vecPInt>& solution)
{
    pad(solution);
    vec2PInt reformatted_solution;

    for (Agent agent : agents) {
        reformatted_solution.push_back(solution[agent]);
    }

    return reformatted_solution;
}

void Planner::pad(map<Agent, vecPInt>& solution)
{
    int _max = 0;
    for (auto& elem : solution)
    {
        int path_length = static_cast<int>(elem.second.size());
        _max = std::max(_max, path_length);
    }

    for (auto& elem : solution)
    {
        if (elem.second.size() == _max)
            continue;

        int added_len = _max - static_cast<int>(elem.second.size());
        pairInt end_value = elem.second.back();
        for (int i = 0; i < added_len; i++)
            solution[elem.first].push_back(end_value);
    }
}

void Planner::set_planner(vecPInt starts, vecPInt goals, int robot_radius, vecPInt static_obstacle, dynamicOb dynamic_obstacle, int low_level_max_iter)
{
    this->starts = starts;
    this->goals = goals;
    this->robot_radius = robot_radius;

    aStarPlanner.set_static_obstacle(static_obstacle);
    this->static_potential_map = aStarPlanner.get_static_potential_map();
    this->dynamic_obstacle = dynamic_obstacle;
}

void Planner::set_planner(int robot_radius, int low_level_max_iter)
{
    this->robot_radius = robot_radius;
    this->static_potential_map = aStarPlanner.get_static_potential_map();
    this->dynamic_obstacle = dynamic_obstacle;
}

void Planner::set_starts(vecPInt starts)
{
    this->starts = starts;
}

void Planner::set_goals(vecPInt goals)
{
    this->goals = goals;
}

vecPInt Planner::get_starts()
{
    return this->starts;
}

vecPInt Planner::get_goals()
{
    return this->goals;
}

void Planner::set_max_core()
{
    max_core = thread::hardware_concurrency();
    max_core = max_core > 5 ? max_core - 4 : max_core;
}

void Planner::set_max_core(int n_core)
{
    max_core = n_core;
}

bool Planner::validate_agent_position()
{
    Map map = aStarPlanner.get_origin_map();

    for (auto& p : starts)
    {
        if (map[p.first][p.second] != 0)
            return true;
    }
    for (auto& p : goals)
    {
        if (map[p.first][p.second] != 0)
            return true;
    }
    return false;
}

void Planner::modify_potential_map()
{
    aStarPlanner.modify_potential_map(dynamic_obstacle);
}
#include "planner.h"

vec2PInt Planner::plan(vecPInt starts, vecPInt goals, int max_iter, int low_level_max_iter, bool debug)
// ��ε��� ��ȯ���ִ� �Լ�
{
    this->low_level_max_iter = low_level_max_iter;
    this->debug = debug;
    this->agents = min_cost(starts, goals);

    Constraints constraints;
    bool tf = true;
    map<Agent, vecPInt> solution;

    for (Agent agent : agents) {
        solution[agent] = calculate_path(agent, constraints, {});
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
    while (!open.empty() && iter_ < max_iter) {
        iter_++;
        pair<vector<pairCTNode>, vector<vec2PInt>> results;
        for (auto iter = open.begin(); iter != open.end();) {
            search_node(*iter, results);
            iter=open.erase(iter);
        }
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



void print_Node(CTNode node)
{
    for (auto iter = node.solution.begin(); iter != node.solution.end(); iter++)
    {
        std::cout << "Agent:" << iter->first << "  %%%%%%%%%%%%\n";
        for (int i = 0; i < iter->second.size(); i++)
        {
            std::cout << "( " << iter->second[i].first << ", " << iter->second[i].second << " )\n";
        }
    }
}

#include <thread>

// result�� �ǹ� �ľ� �ʿ�, �浹�� �߻��� ��� ù��° ��ҿ� �� ������Ʈ�� ���� ����� ������ Ž�� ��, �浹�� ���� ��� �� ��° ��ҿ� ��ü�� ��θ� ����. 
void Planner::search_node(CTNode& best, pair<vector<pairCTNode>, vector<vec2PInt>>& results)
{
    pair<vecAgent, int> val = validate_paths(this->agents, best); // �浹 ������Ʈ�� �浹 �ε��� ���� ��ȯ 
    Agent agent_i = ((val.first)[0]);
    Agent agent_j = ((val.first)[1]);
    int time_of_conflict = val.second;
    if (time_of_conflict == -1) // �浹�� ���� ��
    {
        results.second.push_back(Planner::reformat(Planner::agents, best.solution));
        return;
    }

    Constraints agent_i_constraint = calculate_constraints(best, agent_i, agent_j, time_of_conflict);
    // (8,6)���� 10�� 11�� �浹��Ȳ�� �߻��Ѵٴ� �� calculate_constraints ���� �޾ƿ�
    Constraints agent_j_constraint = calculate_constraints(best, agent_j, agent_i, time_of_conflict);

    auto multi_calculate_path = [](Agent agent, Constraints constraints, map<int, setPInt> goal_times, int low_level_max_iter, bool debug, vecPInt& agent_i_path)
    {
        map<int, setPInt> a;
        agent_i_path = AStarPlanner(agent.start, agent.goal, constraints.setdefault(agent, a), goal_times, low_level_max_iter, debug);
        return;
    };

    vecPInt agent_i_path;
    vecPInt agent_j_path;

    thread t1 = thread(multi_calculate_path, agent_i, agent_i_constraint, calculate_goal_times(best, agent_i, agents), low_level_max_iter, debug, ref(agent_i_path));
    thread t2 = thread(multi_calculate_path, agent_j, agent_j_constraint, calculate_goal_times(best, agent_j, agents), low_level_max_iter, debug, ref(agent_j_path));

    t1.join();
    t2.join();

    //vecPInt agent_i_path = calculate_path(agent_i, agent_i_constraint, calculate_goal_times(best, agent_i, agents));
    //vecPInt agent_j_path = calculate_path(agent_j, agent_j_constraint, calculate_goal_times(best, agent_j, agents));

    map<Agent, vecPInt> solution_i = best.solution;
    map<Agent, vecPInt> solution_j = best.solution;
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

    results.first.push_back(pairCTNode{node_i, node_j});
    return;
}


/*
void Planner::search_node(CTNode& best, std::pair<std::vector<std::pair<CTNode, CTNode>>, std::vector<vector<std::vector<std::pair<int, int>>>>>& results)
// result�� �ǹ� �ľ� �ʿ�, �浹�� �߻��� ��� ù��° ��ҿ� �� ������Ʈ�� ���� ����� ������ Ž�� ��, �浹�� ���� ��� �� ��° ��ҿ� ��ü�� ��θ� ����.
{
   std::pair < std::vector<Agent>, int> val = Planner::validate_paths(Planner::agents, best);
   Agent agent_i = ((val.first)[0]);
   Agent agent_j = ((val.first)[1]);
   int time_of_conflict = val.second;
   if (time_of_conflict ==-1) {
      results.second.push_back(Planner::reformat(Planner::agents, best.solution));
      return;
   }
   Constraints agent_i_constraint = Planner::calculate_constraints(best, agent_i, agent_j, time_of_conflict);
   Constraints agent_j_constraint = Planner::calculate_constraints(best, agent_j, agent_i, time_of_conflict);
   std::vector<std::pair<int, int>> agent_i_path = Planner::calculate_path(agent_i, agent_i_constraint, Planner::calculate_goal_times(best, agent_i, Planner::agents));
   std::vector<std::pair<int, int>> agent_j_path = Planner::calculate_path(agent_j, agent_j_constraint, Planner::calculate_goal_times(best, agent_j, Planner::agents));

   std::map<Agent, std::vector<std::pair<int, int>>> solution_i = best.solution;
   std::map<Agent, std::vector<std::pair<int, int>>> solution_j = best.solution;
   solution_i[agent_i] = agent_i_path;
   solution_j[agent_j] = agent_j_path;
   CTNode* node_i = nullptr;
   CTNode* node_j = nullptr;
   bool tf = true;
   for (auto iter = solution_i.begin(); iter != solution_i.end(); iter++) {
      if (iter->second.size() == 0) {
         tf = false;
         break;
      }
   }
   if (tf == true) *node_i = CTNode(agent_i_constraint, solution_i);
   tf = true;
   for (auto iter = solution_j.begin(); iter != solution_j.end(); iter++) {
      if (iter->second.size() == 0) {
         tf = false;
         break;
      }
   }
   if (tf == true) *node_j = CTNode(agent_j_constraint, solution_j);
   results.first.push_back(std::pair<CTNode, CTNode>{*node_i, * node_j});
   return;
}*/


vecPAgent Planner::combination(vecAgent total_agent)
{
    vecPAgent result; // ex) {{a,b}, {a,c}...}

    for (int i = 0; i < total_agent.size() - 1; i++) {
        for (int j = i + 1; j < total_agent.size(); j++) {
            pairAgent res = { total_agent[i], total_agent[j] };
            result.push_back(res);
        }
    }

    return result;
}


//validate_paths////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pair<vecAgent, int> Planner::validate_paths(vecAgent agents, CTNode node)
// �浹�� �ִ� agent�� ����(�浹�� �����ϴ� ������Ʈ �����Ϳ� �ð� ����) ���� ��ȯ
{
    vecPAgent combi = combination(agents);
    for (int i = 0; i < combi.size(); i++)
    {
        int time_of_conflict = safe_distance(node.solution, combi[i].first, combi[i].second);
        if (time_of_conflict == -1) // �ƹ� �̻� ����
            continue;
        vecAgent re1{ combi[i].first,combi[i].second };
        pair<vecAgent, int> re2{ re1,time_of_conflict };
        return re2;
        // �浹 �߻��� for�� �ȵ��� ���⼭ ���� (������Ʈ �����Ϳ� �ð� ���� ��ȯ)
    }

    vecAgent re1{ combi[0].first,combi[0].second };
    pair<vecAgent, int>re2{ re1,-1 };
    return re2;
    // �浹 ������ for�� ������ ���� ���⼭ ����
    // -1�� �浹�� ������ ����ε� set�� ���� �浹�� ���� ��츦 �켱���� ����Ѵ�.
    // unorderd_set�� ����ϸ� ������ agent ������ �ٲ� �� �ִ�.
    // ���� �־��� ������� �ִ´�
}


int Planner::safe_distance(map<Agent, vecPInt> solution, Agent agent_i, Agent agent_j)
{
    int idx = -1; // �ε����� 0���� �����ϹǷ� 
    int size = 0;
    vecPInt paths_i = solution[agent_i];    // agent_i�� ���� ��� ex) {{1,2},{5,4}} 
    vecPInt paths_j = solution[agent_j];    // agent_j�� ���� ��� ex) {{4,6},{3,8}} 
    vec2PInt self{ paths_i, paths_j };      // paths_i�� paths_j�� ��� �ٲ������ for�� �Ἥ 
    if (self[0].size() > self[1].size())
        size = self[1].size();              // size ���� ������ ����
    else
        size = self[0].size();              // size ���� ������ ����

    for (int i = 0; i < size; i++)
    {
        idx++;                                              // �ε��� ��ȣ �˷��ֱ� ���� 
        if (dist(paths_i[i], paths_j[i]) > robot_radius)    // agent�� solution�� ��δ�� �̵��ϸ鼭 �浹 ���� Ȯ�� (���� �ð����� ��ġ ��)
            continue;
        return idx;                                         // �浹�ϴ� �ε��� ����
    }
    return -1;                              // �̻� ����
}


// �� �� ������ �Ÿ� ��Ŭ���� �Ÿ� ����Ͽ� ��ȯ�ϴ� �Լ�
// �̵������� ������ ���ڷ� ��
double Planner::dist(pairInt point1, pairInt point2)
{
    double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
    return result;
}
/////////////////////////////////////////////////////////////////////


Constraints Planner::calculate_constraints(CTNode& node, Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict) {
    // cbs�˰��� ���� �� ������Ʈ�� ��θ� ���� �浹�� �����ϸ� �� ��ġ�� �ٲ� ���ο� ��θ� �����.
    // Ư�� ������Ʈ�� ���� �������ǿ� �ٸ� ������Ʈ�� ���� ���������� �߰������ش�.
    // ��������: �ð��� ���� �� �� ���� ��ǥ��
    vecPInt constrained_path = node.solution[constrained_agent];
    vecPInt unchanged_path = node.solution[unchanged_agent];
    pairInt pivot = unchanged_path[time_of_conflict]; // �浹�� �߻��� �ð����� ��ǥ//o�ʺ��� ��ġ ����ȴٰ� �������� �� ����1�ʺ��͸� time_of_conflict-1�� �ֱ�
    int conflict_end_time = time_of_conflict;

    while (true) { //aaaaa
        if (conflict_end_time >= constrained_path.size())   break;
        if (dist(constrained_path[conflict_end_time], pivot) >= 2 * robot_radius) break;  // �浹 ��Ȳ
        conflict_end_time++;
    }

    return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}


//calculate_goal_times/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

map<int, setPInt> Planner::calculate_goal_times(CTNode& node, Agent& agent, vecAgent& agents)
{
    // �ϳ��� ������Ʈ�� ���� �ٸ� ������Ʈ���� ������(key = time, value = ���� ��ǥ)�� dictionary
    map<Agent, vecPInt> solution = node.solution;
    map<int, setPInt> goal_times;

    for (Agent& other_agent : agents) {
        if (other_agent == agent) continue;

        int time = solution[other_agent].size() - 1;
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


//////////////////////////////////////////////////////
// ������ multi agent path�� ��ȯ�ϴ� �ڵ�
vecPInt Planner::calculate_path(Agent agent, Constraints constraints, map<int, setPInt> goal_times) // �����̶� �ð��� ���� ��� ������Ʈ���� �浹�� ���� ��θ� ��ȯ
{
    map<int, setPInt> a;
    return AStarPlanner(agent.start, agent.goal, constraints.setdefault(agent, a), goal_times, low_level_max_iter, debug); // �浹���� ��� ��ȯ
}


//reformat/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// �� agent���� ���(2���� ����) ���� �Ȱ��� ����� ������ 3���� ���� ��ȯ
vec2PInt Planner::reformat(vecAgent agents, map<Agent, vecPInt>& solution) // pad�� �ø��� �ַ���� ���� �������
{
    pad(solution);
    vec2PInt reformatted_solution;
    for (Agent agent : agents) {
        reformatted_solution.push_back(solution[agent]);
    }
    return reformatted_solution;
}


void Planner::pad(map<Agent, vecPInt>& solution) // ��ε��� ������ ũ��� ����� (���� ������ ������Ʈ�� ���ڸ��� ������)
// ���⼭ solution���� agent�� �Բ� �ִ� path���� ������ vector�ε� path���� ��ġ�ε� �̰� ���� vector<int>�� �ؾߵǳ�?? �̷��� ������ ���� ���� ���� �ʳ�?
{
    // path�� �ִ� ���� �ľ�
    int max_ = 0;
    for (auto& elem : solution)
    {
        vecPInt path = elem.second;
        int path_length = path.size();
        max_ = std::max(max_, path_length);
    }

    // path�� ������ ��ġ�� �ִ� ���� ��ŭ �ݺ����� �̿��ؼ� �߰����ִ� �ڵ�
    for (auto& elem : solution) {
        Agent agent = elem.first;
        vecPInt path = elem.second;
        if (path.size() == max_)
            continue;

        vecPInt padded = path;
        int added_len = max_ - padded.size();
        pairInt end_value = padded.back();
        for (int i = 0; i < added_len; i++);
        padded.push_back(end_value);
        solution[agent] = padded;
    }
}
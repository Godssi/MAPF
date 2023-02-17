#include "planner.h"

vec2PInt Planner::plan(vecPInt starts, vecPInt goals, int max_iter, int low_level_max_iter, bool debug)
// 경로들을 반환해주는 함수
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

// result의 의미 파악 필요, 충돌이 발생한 경우 첫번째 요소에 두 에이전트의 정보 저장과 마지막 탐색 즉, 충돌이 없는 경우 두 번째 요소에 전체의 경로를 저장. 
void Planner::search_node(CTNode& best, pair<vector<pairCTNode>, vector<vec2PInt>>& results)
{
    pair<vecAgent, int> val = validate_paths(this->agents, best); // 충돌 에이전트와 충돌 인덱스 정상 반환 
    Agent agent_i = ((val.first)[0]);
    Agent agent_j = ((val.first)[1]);
    int time_of_conflict = val.second;
    if (time_of_conflict == -1) // 충돌이 없을 때
    {
        results.second.push_back(Planner::reformat(Planner::agents, best.solution));
        return;
    }

    Constraints agent_i_constraint = calculate_constraints(best, agent_i, agent_j, time_of_conflict);
    // (8,6)에서 10초 11초 충돌상황이 발생한다는 것 calculate_constraints 통해 받아옴
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
// result의 의미 파악 필요, 충돌이 발생한 경우 첫번째 요소에 두 에이전트의 정보 저장과 마지막 탐색 즉, 충돌이 없는 경우 두 번째 요소에 전체의 경로를 저장.
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
// 충돌이 있는 agent의 정보(충돌이 존재하는 에이전트 포인터와 시간 정보) 집합 반환
{
    vecPAgent combi = combination(agents);
    for (int i = 0; i < combi.size(); i++)
    {
        int time_of_conflict = safe_distance(node.solution, combi[i].first, combi[i].second);
        if (time_of_conflict == -1) // 아무 이상 없음
            continue;
        vecAgent re1{ combi[i].first,combi[i].second };
        pair<vecAgent, int> re2{ re1,time_of_conflict };
        return re2;
        // 충돌 발생시 for문 안들어가서 여기서 리턴 (에이전트 포인터와 시간 정보 반환)
    }

    vecAgent re1{ combi[0].first,combi[0].second };
    pair<vecAgent, int>re2{ re1,-1 };
    return re2;
    // 충돌 없으면 for문 끝까지 가서 여기서 리턴
    // -1은 충돌을 안했을 경우인데 set을 쓰면 충돌을 안할 경우를 우선으로 고려한다.
    // unorderd_set을 사용하면 집합의 agent 순서가 바뀔 수 있다.
    // 내가 넣어준 순서대로 넣는다
}


int Planner::safe_distance(map<Agent, vecPInt> solution, Agent agent_i, Agent agent_j)
{
    int idx = -1; // 인덱스는 0부터 시작하므로 
    int size = 0;
    vecPInt paths_i = solution[agent_i];    // agent_i에 대한 경로 ex) {{1,2},{5,4}} 
    vecPInt paths_j = solution[agent_j];    // agent_j에 대한 경로 ex) {{4,6},{3,8}} 
    vec2PInt self{ paths_i, paths_j };      // paths_i랑 paths_j를 계속 바꿔줘야함 for문 써서 
    if (self[0].size() > self[1].size())
        size = self[1].size();              // size 작은 것으로 설정
    else
        size = self[0].size();              // size 작은 것으로 설정

    for (int i = 0; i < size; i++)
    {
        idx++;                                              // 인덱스 번호 알려주기 위해 
        if (dist(paths_i[i], paths_j[i]) > robot_radius)    // agent가 solution의 경로대로 이동하면서 충돌 여부 확인 (같은 시간에서 위치 비교)
            continue;
        return idx;                                         // 충돌하는 인덱스 리턴
    }
    return -1;                              // 이상 없음
}


// 두 점 사이의 거리 유클리드 거리 계산하여 반환하는 함수
// 이동과정의 점들이 인자로 들어감
double Planner::dist(pairInt point1, pairInt point2)
{
    double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
    return result;
}
/////////////////////////////////////////////////////////////////////


Constraints Planner::calculate_constraints(CTNode& node, Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict) {
    // cbs알고리즘에 의해 두 에이전트의 경로를 비교해 충돌이 존재하면 그 위치를 바꿔 새로운 경로를 만든다.
    // 특정 에이전트에 대한 제약조건에 다른 에이전트에 의해 제약조건을 추가시켜준다.
    // 제약조건: 시간에 따른 갈 수 없는 좌표들
    vecPInt constrained_path = node.solution[constrained_agent];
    vecPInt unchanged_path = node.solution[unchanged_agent];
    pairInt pivot = unchanged_path[time_of_conflict]; // 충돌이 발생한 시간에서 좌표//o초부터 위치 저장된다고 생각했을 때 만약1초부터면 time_of_conflict-1로 넣기
    int conflict_end_time = time_of_conflict;

    while (true) { //aaaaa
        if (conflict_end_time >= constrained_path.size())   break;
        if (dist(constrained_path[conflict_end_time], pivot) >= 2 * robot_radius) break;  // 충돌 상황
        conflict_end_time++;
    }

    return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}


//calculate_goal_times/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

map<int, setPInt> Planner::calculate_goal_times(CTNode& node, Agent& agent, vecAgent& agents)
{
    // 하나의 에이전트에 대한 다른 에이전트들의 마지막(key = time, value = 현재 좌표)의 dictionary
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
// 안전한 multi agent path를 반환하는 코드
vecPInt Planner::calculate_path(Agent agent, Constraints constraints, map<int, setPInt> goal_times) // 공간이랑 시간에 대해 모든 에이전트들의 충돌이 없는 경로를 반환
{
    map<int, setPInt> a;
    return AStarPlanner(agent.start, agent.goal, constraints.setdefault(agent, a), goal_times, low_level_max_iter, debug); // 충돌없는 경로 반환
}


//reformat/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// 각 agent들의 경로(2차원 벡터) 길이 똑같이 만들고 저장한 3차원 벡터 반환
vec2PInt Planner::reformat(vecAgent agents, map<Agent, vecPInt>& solution) // pad로 늘리고 솔루션이 새로 만들어짐
{
    pad(solution);
    vec2PInt reformatted_solution;
    for (Agent agent : agents) {
        reformatted_solution.push_back(solution[agent]);
    }
    return reformatted_solution;
}


void Planner::pad(map<Agent, vecPInt>& solution) // 경로들을 동일한 크기로 만든다 (먼저 도착한 에이전트는 그자리에 가만히)
// 여기서 solution에서 agent와 함께 있는 path들의 집합이 vector인데 path들은 위치인데 이걸 굳이 vector<int>로 해야되나?? 이러면 오류가 생길 수도 있지 않나?
{
    // path의 최대 길이 파악
    int max_ = 0;
    for (auto& elem : solution)
    {
        vecPInt path = elem.second;
        int path_length = path.size();
        max_ = std::max(max_, path_length);
    }

    // path의 마지막 위치를 최대 길이 만큼 반복문을 이용해서 추가해주는 코드
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
#include <vector>
#include <tuple>
#include <unordered_map>
#include <functional>
#include <thread>
#include <queue>
#include <iostream>
#include <utility>
#include <mutex>
#include "planner.h"
#include <math.h>
#include <algorithm>

std::map<Agent, std::vector<std::pair<int, int>>> plan(std::vector<std::pair<int, int>> starts, std::vector<std::pair<int, int>> goals, std::function<vector<Agent>(std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>)> assign = min_cost, int max_iter = 200, int low_level_max_iter = 100, int max_process = 10, bool debug = false)
{
	vector<Agent> agents = assign(starts, goals);
	Constraints constraints;
	std::map < Agent, std::vector<std::pair<int, int>>> solution;
	Planner* p; // calculate_path에 접근하기 위해 생성
	for (Agent agent : agents) {
		solution[agent] = p->calculate_path(agent, constraints, {});
	}
	std::vector<std::pair<int, int>> open;
}

std::vector<Agent> res;
std::vector<std::vector<Agent>> result; // ex) {{a,b}, {a,c}...}
std::vector<Agent> combination(std::vector<Agent> total_agent, int n, int d, int cur)
{
	if (d == n)
	{
		/*for (int i = 0; i < res.size(); i++) {
		   cout << res[i] << " ";
		}
		cout << endl;*/
		result.push_back(res);
		return res;
	}
	else
	{
		for (int i = cur; i < total_agent.size(); i++)
		{
			res.push_back(total_agent[i]);
			combination(total_agent, n, d + 1, i + 1);
			res.pop_back();
		}
		return res;
	}
}
int combination_num(int n, int r)
{
	if (n == r || r == 0)
		return 1;
	else
		return combination_num(n - 1, r - 1) + combination_num(n - 1, r);
}

unordered_set<Agent, Agent, int> Planner::validate_paths(std::vector<Agent> agents, CTNode node) // 충돌이 있는 agent의 정보(에이전트명, 인덱스) 집합 반환
{
	for (int i = 0; i < combination_num(agents.size(), 2); i++)
	{
		int time_of_conflict = safe_distance(node.solution, result[i][0], result[i][1]);
		if (time_of_conflict == -1) // 아무 이상 없음
			continue;
		return { result[i][0] ,result[i][1] ,time_of_conflict }; // 이상있을때 인덱스 반환
	}
	return { nullptr, nullptr, -1 };
}
// -1은 충돌을 안했을 경우인데 set을 쓰면 충돌을 안할 경우를 우선으로 고려한다.
// unorderd_set을 사용하면 집합의 agent 순서가 바뀔 수 있다.
// 내가 넣어준 순서대로 넣는다

int Planner::safe_distance(std::map<Agent, std::vector<std::pair<int,int>>> solution, Agent agent_i, Agent agent_j)
{
	int idx = -1; // 인덱스는 0부터 시작하므로 
	int size = 0;
	std::vector<std::pair<int,int>> paths_i = solution[agent_i]; // agent_i에 대한 경로 ex) {{1,2},{5,4}} 
	std::vector<std::pair<int,int>> paths_j = solution[agent_j]; // agent_j에 대한 경로 ex) {{4,6},{3,8}} 
	std::vector< std::vector<std::pair<int,int>>, std::vector<std::pair<int,int>>> self = { paths_i, paths_j }; // paths_i랑 paths_j를 계속 바꿔줘야함 for문 써서 
	if (self[0].size() > self[1].size())
	{
		size = self[1].size(); // size 작은 것으로 설정
	}
	else
		size = self[0].size(); // size 작은 것으로 설정

	for (int i = 0; i < size; i++)
	{
		idx++; // 인덱스 번호 알려주기 위해 
		if (Planner::dist(paths_i[i], paths_j[i]) > 2 * robot_radius) // agent가 solution의 경로대로 이동하면서 충돌 여부 확인 (같은 시간에서 위치 비교)
			continue;
		return idx; // 충돌하는 인덱스 리턴
	}
	return -1; // 이상 없음
}
static double dist(std::pair<int, int> point1, std::pair<int, int> point2) // 두 점 사이의 거리 유클리드 거리 계산하여 반환하는 함수
// 이동과정의 점들이 인자로 들어감
{
	double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
	return result;
}



/////////////////////////////////////////////////////////////////////


Constraints Planner::calculate_constraints(CTNode& node, Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict){
// cbs알고리즘에 의해 두 에이전트의 경로를 비교해 충돌이 존재하면 그 위치를 바꿔 새로운 경로를 만든다.
// 특정 에이전트에 대한 제약조건에 다른 에이전트에 의해 제약조건을 추가시켜준다.
// 제약조건: 시간에 따른 갈 수 없는 좌표들
	std::vector<std::pair<int, int>> constrained_path = node.solution[constrained_agent];
	std::vector< std::pair<int, int>> unchanged_path = node.solution[unchanged_agent];
	std::pair<int, int> pivot= unchanged_path[time_of_conflict]; // 충돌이 발생한 시간에서 좌표//o초부터 위치 저장된다고 생각했을 때 만약1초부터면 time_of_conflict-1로 넣기
	int conflict_end_time = time_of_conflict;
	while (true) { //aaaaa
		if (conflict_end_time>=constrained_path.size())	break;
		if (Planner::dist(constrained_path[conflict_end_time], pivot) >= 2 * Planner::robot_radius) break;  // 충돌 상황
		conflict_end_time++;
	}
return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}

std::map<int, std::pair<int, int>> Planner::calculate_goal_times(CTNode& node, Agent& agent, std::vector<Agent>& agents)
// 하나의 에이전트에 대한 다른 에이전트들의 마지막(key = time, value = 현재 좌표)의 dictionary
{
	std::map<Agent, std::vector<std::pair<int, int>>> solution = node.solution();
	std::map<int, std::pair<int, int>> goal_times;
	for (Agent& other_agent : agents) {
		if (other_agent == agent) continue;
		int time = solution[other_agent].size() - 1;
		goal_times.insert({ time, solution[other_agent][time]}); //aaa
	}
	return goal_times;
}

//////////////////////////////////////////////////////

std::vector<std::pair<int,int>> Planner::calculate_path(Agent agent, Constraints constraints, std::map<int, std::set<std::pair<int, int>>> goal_times) // 공간이랑 시간에 대해 모든 에이전트들의 충돌이 없는 경로를 반환
{
	std::map<int, std::pair<int, int>> a;
	return this->st_planner.plan(agent.start, agent.goal, constraints.setdault(agent, a), goal_times, this->low_level_max_iter, this->debug); // 충돌없는 경로 반환
}
static std::vector<std::vector<std::vector<int>>> reformat(std::vector<Agent> agents, std::map<Agent, std::vector<std::vector<int>>> solution) // pad로 늘리고 솔루션이 새로 만들어짐
{
	auto solution = Planner::pad(solution);
	std::vector<std::map<int, std::vector<int>>> reformatted_solution;
	for (Agent agent : agents) {
		reformatted_solution.push_back(solution[agent]);
	}
	return reformatted_solution; // 각 agent들의 경로(2차원 벡터)를 저장한 3차원 벡터 반환
}
static map<Agent, std::vector<std::vector<int>>> pad(map<Agent, std::vector<std::vector<int>>> solution) // 경로들을 동일한 크기로 만든다 (먼저 도착한 에이전트는 그자리에 가만히)
{
	int max_ = 0;
	for (auto& elem : solution)
	{
		max_ = std::max(max_, path.shape(0)); // max값으로 만들어준다. 
	}
	for (auto& [agent, path] : solution) {
		if (path.shape(0) == max_) {
			continue;
		}
		np::ndarray padded = np::concatenate(np::array({ path, np::array(np::list<np::ndarray>({path[np::shape(path)[0] - 1]]}) * (max_ - path.shape(0))) }));
		solution[agent] = padded;
	}
}
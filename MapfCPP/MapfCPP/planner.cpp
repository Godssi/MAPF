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

//std::map<Agent, std::vector<int, int>> Planner::plan(std::vector<std::pair<int, int>> starts, std::vector<std::pair<int, int>> goals, int max_iter = 200, int low_level_max_iter = 100, int max_process =10 , bool debug= false)
//{
//
//}
//void Planner::search_node(CTNode best, std::vector<std::pair<CTNode, CTNode>>& results)
//{
//
//}
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

unordered_set<Agent, Agent, int> Planner::validate_paths(std::vector<Agent> agents, CTNode node) // �浹�� �ִ� agent�� ����(������Ʈ��, �ε���) ���� ��ȯ
{
	for (int i = 0; i < combination_num(agents.size(), 2); i++)
	{
		int time_of_conflict = safe_distance(node.solution, result[i][0], result[i][1]);
		if (time_of_conflict == -1) // �ƹ� �̻� ����
			continue;
		return { result[i][0] ,result[i][1] ,time_of_conflict }; // �̻������� �ε��� ��ȯ
	}
	return { nullptr, nullptr, -1 };
}
// -1�� �浹�� ������ ����ε� set�� ���� �浹�� ���� ��츦 �켱���� ����Ѵ�.
// unorderd_set�� ����ϸ� ������ agent ������ �ٲ� �� �ִ�.
// ���� �־��� ������� �ִ´�

int Planner::safe_distance(std::map<Agent, std::vector<std::vector<int>>> solution, Agent agent_i, Agent agent_j)
{
	int idx = -1; // �ε����� 0���� �����ϹǷ� 
	int size = 0;
	std::vector<std::vector<int>> paths_i = solution[agent_i]; // agent_i�� ���� ��� ex) {{1,2},{5,4}} 
	std::vector<std::vector<int>> paths_j = solution[agent_j]; // agent_j�� ���� ��� ex) {{4,6},{3,8}} 
	std::vector< std::vector<std::vector<int>>, std::vector<std::vector<int>>> self = { paths_i, paths_j }; // paths_i�� paths_j�� ��� �ٲ������ for�� �Ἥ 
	if (self[0].size() > self[1].size())
	{
		size = self[1].size(); // size ���� ������ ����
	}
	else
		size = self[0].size(); // size ���� ������ ����

	for (int i = 0; i < size; i++)
	{
		idx++; // �ε��� ��ȣ �˷��ֱ� ���� 
		if (Planner::dist(paths_i[i], paths_j[i]) > 2 * robot_radius) // agent�� solution�� ��δ�� �̵��ϸ鼭 �浹 ���� Ȯ�� (���� �ð����� ��ġ ��)
			continue;
		return idx; // �浹�ϴ� �ε��� ����
	}
	return -1; // �̻� ����
}
static double dist(std::pair<int,int> point1, std::pair<int, int> point2) // �� �� ������ �Ÿ� ��Ŭ���� �Ÿ� ����Ͽ� ��ȯ�ϴ� �Լ�
// �̵������� ������ ���ڷ� ��
{
	double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
	return result;
}



/////////////////////////////////////////////////////////////////////


Constraints Planner::calculate_constraints( CTNode& node,  Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict)  
// cbs�˰��� ���� �� ������Ʈ�� ��θ� ���� �浹�� �����ϸ� �� ��ġ�� �ٲ� ���ο� ��θ� �����.
// Ư�� ������Ʈ�� ���� �������ǿ� �ٸ� ������Ʈ�� ���� ���������� �߰������ش�.
// ��������: �ð��� ���� �� �� ���� ��ǥ��

	std::vector<pair<int, std::pair<int, int>>> constrained_path = node.solution[constrained_agent];
	std::vector<pair<int, std::pair<int, int>>> unchanged_path = node.solution[unchanged_agent];
	std::pair<int, int> pivot; // �浹�� �߻��� �ð����� ��ǥ
	for (auto iter=unchanged_path.begin();iter!=unchanged_path.end();iter++)
		if (iter->first == time_of_conflict) {
			pivot = iter->second;
			break;
		}
	int conflict_end_time = time_of_conflict;
	while (true) { //aaaaa
		auto iter = constrained_path.begin();
		std::pair<int, int> conf;
		for (iter = constrained_path.begin(); iter != constrained_path.end(); iter++) {
			if (iter->first == conflict_end_time) {
				conf = iter->second;
				break;
			}
		}
		if (iter==constrained_path.end()) break;
		if (Planner::dist(conf, pivot) >= 2 * Planner::robot_radius) break;  // �浹 ��Ȳ
		conflict_end_time++;
	}
	return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}

std::map<int, std::pair<int, int>> Planner::calculate_goal_times(CTNode& node, Agent& agent, std::vector<Agent>& agents)
// �ϳ��� ������Ʈ�� ���� �ٸ� ������Ʈ���� ������(key = time, value = ���� ��ǥ)�� dictionary
{
	std::map<Agent, std::vector<std::pair<int, std::pair<int, int>>>> solution = node.solution();
	std::map<int, std::pair<int, int>> goal_times;
	for (Agent& other_agent : agents) {
		if (other_agent == agent) continue;
		int time = solution[other_agent].size() - 1;
		std::pair<int, int> sol;
		for (auto iter = solution[other_agent].begin(); iter != solution[other_agent].end(); iter++)
			if (iter->first == time) {
				sol = iter->second; break;
			}
		goal_times.insert({ time, sol }); //aaa
	}
	return goal_times;
}

//////////////////////////////////////////////////////

std::vector<std::vector<int>> Planner::calculate_path(Agent agent, Constraints constraints, std::map<int, std::set<std::pair<int, int>>> goal_times) // �����̶� �ð��� ���� ��� ������Ʈ���� �浹�� ���� ��θ� ��ȯ
{
	std::map<int, std::pair<int, int>> a;
	return this->st_planner.plan(agent.start, agent.goal, constraints.setdault( agent, a ), goal_times, this->low_level_max_iter, this->debug); // �浹���� ��� ��ȯ
}
static std::vector<std::vector<std::vector<int>>> reformate(std::vector<Agent> agents, std::map<Agent, std::vector<std::vector<int>>> solution) // pad�� �ø��� �ַ���� ���� �������
{
	auto solution = Planner::pad(solution); 
	std::vector<std::map<int,std::vector<int>>> reformatted_solution;
	for (Agent agent : agents) {
		reformatted_solution.push_back(solution[agent]);  
	}
	return reformatted_solution; // �� agent���� ���(2���� ����)�� ������ 3���� ���� ��ȯ
}
static map<Agent, std::vector<std::vector<int>>> pad(map<Agent, std::vector<std::vector<int>>> solution) // ��ε��� ������ ũ��� ����� (���� ������ ������Ʈ�� ���ڸ��� ������)
{
	int max_ = 0; 
	for (auto& elem : solution)
	{
		max_ = std::max(max_, path.shape(0)); // max������ ������ش�. 
	}
	for (auto& [agent, path] : solution) {
		if (path.shape(0) == max_) {
			continue;
		}
		np::ndarray padded = np::concatenate(np::array({ path, np::array(np::list<np::ndarray>({path[np::shape(path)[0] - 1]]}) * (max_ - path.shape(0))) }));
		solution[agent] = padded;
	}
}
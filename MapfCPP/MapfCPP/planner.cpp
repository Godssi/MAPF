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
#include "aplanner.h"




std::vector<std::vector<std::pair<int, int>>> Planner::plan(std::vector<std::pair<int, int>> starts, std::vector<std::pair<int, int>> goals, int max_iter, int low_level_max_iter, int max_process, bool debug)
// ��ε��� ��ȯ���ִ� �Ϲ�
{
	Planner::low_level_max_iter = low_level_max_iter;
	Planner::debug = debug;
	vector<Agent> agents = min_cost(starts, goals);
	Constraints constraints;
	bool tf =true;
	std::map < Agent, std::vector<std::pair<int, int>>> solution;
	for (Agent agent : agents) {
		solution[agent] = Planner::calculate_path(agent, constraints, {});
	}
	std::vector<CTNode> open;
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
	while ((open.size() != 0) && iter_ < max_iter) {
		iter_++;
		std::pair<std::vector<std::pair<CTNode, CTNode>>, std::vector<vector<std::vector<std::pair<int, int>>>>> results;
		int no_process;
		if (open.size() > max_process) no_process = max_process;
		else no_process = open.size();
		for (auto iter = open.begin(); iter != open.end(); iter++) {
			search_node(*iter, results);
		}
		if (results.second.size() != 0) return results.second[0];
		for (auto iter = results.first.begin(); iter != results.first.end(); iter++) {
			open.push_back(iter->first);
			open.push_back(iter->second);
		}
	}

}





void Planner::search_node(CTNode& best, std::pair<std::vector<std::pair<CTNode, CTNode>>, std::vector<vector<std::vector<std::pair<int, int>>>>>& results)
// result�� �ǹ� �ľ� �ʿ�, �浹�� �߻��� ��� ù��° ��ҿ� �� ������Ʈ�� ���� ����� ������ Ž�� ��, �浹�� ���� ��� �� ��° ��ҿ� ��ü�� ��θ� ����. 
{
	std::pair < std::vector<Agent*>, int> val = Planner::validate_paths(Planner::agents, best);
	Agent* agent_i = ((val.first)[0]);
	Agent* agent_j = ((val.first)[1]);
	int time_of_conflict = val.second;
	if (agent_i == nullptr) {
		results.second.push_back(Planner::reformat(Planner::agents, best.solution));
		return;
	}
	Constraints agent_i_constraint = Planner::calculate_constraints(best, *agent_i, *agent_j, time_of_conflict);
	Constraints agent_j_constraint = Planner::calculate_constraints(best, *agent_j, *agent_i, time_of_conflict);
	std::vector<std::pair<int, int>> agent_i_path = Planner::calculate_path(*agent_i, agent_i_constraint, Planner::calculate_goal_times(best, *agent_i, Planner::agents));
	std::vector<std::pair<int, int>> agent_j_path = Planner::calculate_path(*agent_j, agent_j_constraint, Planner::calculate_goal_times(best, *agent_j, Planner::agents));

	std::map<Agent, std::vector<std::pair<int, int>>> solution_i = best.solution;
	std::map<Agent, std::vector<std::pair<int, int>>> solution_j = best.solution;
	solution_i[*agent_i] = agent_i_path;
	solution_j[*agent_j] = agent_j_path;
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
}





std::vector<Agent> res;
std::vector<std::vector<Agent>>result; // ex) {{a,b}, {a,c}...}
std::vector<Agent> Planner::combination(std::vector<Agent> total_agent, int n, int d, int cur)
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





//validate_paths////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::pair < std::vector<Agent*>, int> Planner::validate_paths(std::vector<Agent> agents, CTNode node) 
// �浹�� �ִ� agent�� ����(�浹�� �����ϴ� ������Ʈ �����Ϳ� �ð� ����) ���� ��ȯ
{
	std::vector<Agent*> re1;
	std::pair<std::vector<Agent*>, int> re2;
	for (int i = 0; i < combination_num(agents.size(), 2); i++)
	{
		int time_of_conflict = safe_distance(node.solution, result[i][0], result[i][1]);
		if (time_of_conflict == -1) // �ƹ� �̻� ����
			continue;
		re1 = { &(result[i][0]),&(result[i][1]) };
		re2 = { re1,time_of_conflict };
		return re2; // �浹 �߻��� for�� �ȵ��� ���⼭ ���� (������Ʈ �����Ϳ� �ð� ���� ��ȯ)
	}
	re1 = { nullptr,nullptr };
	re2 = { re1,-1 };
	return re2; // �浹 ������ for�� ������ ���� ���⼭ ����
	// -1�� �浹�� ������ ����ε� set�� ���� �浹�� ���� ��츦 �켱���� ����Ѵ�.
	// unorderd_set�� ����ϸ� ������ agent ������ �ٲ� �� �ִ�.
	// ���� �־��� ������� �ִ´�
}




int Planner::safe_distance(std::map<Agent, std::vector<std::pair<int,int>>> solution, Agent agent_i, Agent agent_j)
{
	int idx = -1; // �ε����� 0���� �����ϹǷ� 
	int size = 0;
	std::vector<std::pair<int,int>> paths_i = solution[agent_i]; // agent_i�� ���� ��� ex) {{1,2},{5,4}} 
	std::vector<std::pair<int,int>> paths_j = solution[agent_j]; // agent_j�� ���� ��� ex) {{4,6},{3,8}} 
	std::vector< std::vector<std::pair<int,int>>> self{ paths_i, paths_j }; // paths_i�� paths_j�� ��� �ٲ������ for�� �Ἥ 
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





double Planner::dist(std::pair<int, int> point1, std::pair<int, int> point2) // �� �� ������ �Ÿ� ��Ŭ���� �Ÿ� ����Ͽ� ��ȯ�ϴ� �Լ�
// �̵������� ������ ���ڷ� ��
{
	double result = sqrt(pow(point2.first - point1.first, 2) + pow(point2.second - point1.second, 2));
	return result;
}
/////////////////////////////////////////////////////////////////////



Constraints Planner::calculate_constraints(CTNode& node, Agent& constrained_agent, Agent& unchanged_agent, int& time_of_conflict){
// cbs�˰��� ���� �� ������Ʈ�� ��θ� ���� �浹�� �����ϸ� �� ��ġ�� �ٲ� ���ο� ��θ� �����.
// Ư�� ������Ʈ�� ���� �������ǿ� �ٸ� ������Ʈ�� ���� ���������� �߰������ش�.
// ��������: �ð��� ���� �� �� ���� ��ǥ��
	std::vector<std::pair<int, int>> constrained_path = node.solution[constrained_agent];
	std::vector< std::pair<int, int>> unchanged_path = node.solution[unchanged_agent];
	std::pair<int, int> pivot= unchanged_path[time_of_conflict]; // �浹�� �߻��� �ð����� ��ǥ//o�ʺ��� ��ġ ����ȴٰ� �������� �� ����1�ʺ��͸� time_of_conflict-1�� �ֱ�
	int conflict_end_time = time_of_conflict;
	while (true) { //aaaaa
		if (conflict_end_time>=constrained_path.size())	break;
		if (Planner::dist(constrained_path[conflict_end_time], pivot) >= 2 * Planner::robot_radius) break;  // �浹 ��Ȳ
		conflict_end_time++;
	}
	return node.constraint.fork(constrained_agent, pivot, time_of_conflict, conflict_end_time);
}






//calculate_goal_times/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::map<int, std::set<std::pair<int, int>>> Planner::calculate_goal_times(CTNode& node, Agent& agent, std::vector<Agent>& agents)
{
	// �ϳ��� ������Ʈ�� ���� �ٸ� ������Ʈ���� ������(key = time, value = ���� ��ǥ)�� dictionary
	std::map<Agent, std::vector<std::pair<int, int>>> solution = node.solution;
	std::map<int, std::set<pair<int, int>>> goal_times;
	for (Agent& other_agent : agents) {
		if (other_agent == agent) continue;
		int time = solution[other_agent].size() - 1;
		if (goal_times.find(time) == goal_times.end()) {
			std::set<pair<int, int>> val{ solution[other_agent][time] };
			goal_times.insert({ time, val });
		}
		else goal_times[time].insert(solution[other_agent][time]);
	}
	return goal_times;
}





//////////////////////////////////////////////////////
// ������ multi agent path�� ��ȯ�ϴ� �ڵ�
std::vector<std::pair<int,int>> Planner::calculate_path(Agent agent, Constraints constraints, std::map<int, std::set<std::pair<int, int>>> goal_times) // �����̶� �ð��� ���� ��� ������Ʈ���� �浹�� ���� ��θ� ��ȯ
{
	map<int, set<p>> a;
	return aplanner(agent.start, agent.goal, constraints.setdefault(agent, a), goal_times, Planner::low_level_max_iter, Planner::debug); // �浹���� ��� ��ȯ
}






//reformat/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// �� agent���� ���(2���� ����) ���� �Ȱ��� ����� ������ 3���� ���� ��ȯ
std::vector<std::vector<std::pair<int, int>>> Planner::reformat(std::vector<Agent> agents, std::map<Agent, std::vector<std::pair<int, int>>>& solution) // pad�� �ø��� �ַ���� ���� �������
{
	Planner::pad(solution);
	std::vector<std::vector<std::pair<int, int>>> reformatted_solution;
	for (Agent agent : agents) {
		reformatted_solution.push_back(solution[agent]);
	}
	return reformatted_solution; 
}



void Planner::pad(map<Agent, std::vector<std::pair<int,int>>>& solution) // ��ε��� ������ ũ��� ����� (���� ������ ������Ʈ�� ���ڸ��� ������)
// ���⼭ solution���� agent�� �Բ� �ִ� path���� ������ vector�ε� path���� ��ġ�ε� �̰� ���� vector<int>�� �ؾߵǳ�?? �̷��� ������ ���� ���� ���� �ʳ�?
{
	// path�� �ִ� ���� �ľ�
	int max_ = 0;
	for (auto& elem : solution)
	{
		std::vector<std::pair<int,int>> path = elem.second;
		int path_length = path.size();
		max_ = std::max(max_, path_length); 
	}

	// path�� ������ ��ġ�� �ִ� ���� ��ŭ �ݺ����� �̿��ؼ� �߰����ִ� �ڵ�
	for (auto& elem : solution) {
		Agent agent = elem.first;
		std::vector<std::pair<int,int>> path = elem.second;
		if (path.size() == max_) 
			continue;
		
		std::vector<std::pair<int,int>> padded = path;
		int added_len = max_ - padded.size();
		std::pair<int,int> end_value = padded.back();
		for (int i = 0; i < added_len; i++);
		padded.push_back(end_value);
		solution[agent] = padded;
	}
}
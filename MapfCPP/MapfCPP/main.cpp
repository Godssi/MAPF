#include "Agent.h"
#include "CTNode.h"
#include "Constraints.h"
#include "Assigner.h"
#include "Hungarian.h"

int main()
{
	Agent a({ 1,5 }, { 3,2 }, "a");
	Agent b({ 1,5 }, { 3,2 }, "b");
	std::vector<std::pair<int, int>> solution1 = { {3,3},{2,3},{3,2} };
	std::vector<std::pair<int, int>> solution2 = { {1,5},{2,3},{4,1} };
	map<Agent, std::vector<pair<int, int>>> sol1;
	sol1[a] = solution1;
	sol1[b] = solution2;
	Constraints con;
	CTNode c(con, sol1);
	std::cout << c;
}
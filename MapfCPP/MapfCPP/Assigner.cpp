#include "Assigner.h"

vecAgent min_cost(vecPInt starts, vecPInt goals)
{
    vecAgent agents;
    if (starts.size() != goals.size()) {
        cout << "Not Matched size";
        return agents;
    }

    auto sqdist = [](pairInt x, pairInt y) -> int {
        return (x.first - y.first) * (x.first - y.first) + (x.second - y.second) * (x.second - y.second);
    };

    vector<vector<double>> cost_vec;
    for (auto iter1 = goals.begin(); iter1 != goals.end(); iter1++) {
        vector<double> tmp;
        for (auto iter2 = starts.begin(); iter2 != starts.end(); iter2++) {
            tmp.push_back(sqdist(*iter1, *iter2));
        }
        cost_vec.push_back(tmp);
    }
    vector<int> col_ind;
    HungarianAlgorithm HHandle;
    HHandle.Solve(cost_vec, col_ind);

    for (int i = 0; i < starts.size(); i++) {
        agents.push_back(Agent(starts[i], goals[col_ind[i]]));
    }
    return agents;
}


vecAgent greedy_assign(vecPInt starts, vecPInt goals) {
    vecAgent agents;
    if (starts.size() != goals.size()) {
        cout << "Not Matched size";
        return agents;
    }
    set<pairInt> goal_set;
    for (auto iter = goals.begin(); iter != goals.end(); iter++) {
        goal_set.insert(*iter);
    }
    auto sqdist = [](pairInt x, pairInt y) -> int {
        return (x.first - y.first) * (x.first - y.first) + (x.second - y.second) * (x.second - y.second);
    };

    for (pairInt start : starts) {
        ll closest = std::numeric_limits<int>::max();
        pairInt closest_goal;
        for (pairInt goal : goal_set) {
            ll d = sqdist(start, goal);
            if (d < closest) {
                closest = d;
                closest_goal = goal;
            }
        }
        goal_set.erase(closest_goal);
        agents.push_back(Agent(start, closest_goal));
    }
    return agents;
}
#include "Assigner.h"

vector<Agent> min_cost(vector<p> starts, vector<p> goals)
{
    if (starts.size() != goals.size()) {
        cout << "Not Matched size";
        return;
    }

    auto sqdist = [](p x, p y) -> int {
        return (x.first - y.first) * (x.first - y.first) + (x.second - y.second) * (x.second - y.second);
    };

    vector<vector<int>> cost_vec;
    for (auto iter1 = goals.begin(); iter1 != goals.end(); iter1++) {
        vector<int> tmp;
        for (auto iter2 = starts.begin(); iter2 != starts.end(); iter2++) {
            tmp.push_back(sqdist(*iter1, *iter2));
        }
        cost_vec.push_back(tmp);
    }
    for (auto iter = cost_vec.begin(); iter != cost_vec.end(); iter++) {
        ll col_ind = accumulate((*iter).begin(), (*iter).end(), 0);
    }

    vector<Agent> agents;
    for (int i = 0; i < starts.size(); i++) {
        agents.push_back(Agent(starts[i], goals[col_ind[i]]));
    }
    return agents;
}

vector<Agent> greedy_assign(vector<p> starts, vector<p> goals) {
    if (starts.size() != goals.size()) {
        cout << "Not Matched size";
        return ;
    }
    set<p> goal_set;
    for (auto iter = goals.begin(); iter != goals.end(); iter++) {
        goal_set.insert(*iter);
    }
    auto sqdist = [](p x, p y) -> int {
        return (x.first - y.first) * (x.first - y.first) + (x.second - y.second) * (x.second - y.second);
    };
    vector<Agent> agents;
    for (p start : starts) {
        ll closest = std::numeric_limits<int>::max();
        p closest_goal;
        for (p goal : goal_set) {
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

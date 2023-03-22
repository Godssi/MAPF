#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <map>
#include <set>
#include <unordered_map>

#include <mapf/Assigner.h>

using namespace std;

typedef long long ll;
typedef pair<int, int> pairInt;
typedef pair<Agent, Agent> pairAgent;
typedef vector<Agent> vecAgent;
typedef vector<pairInt> vecPInt;
typedef vector<pairAgent> vecPAgent;
typedef vector<vecPInt> vec2PInt;
typedef set<pairInt> setPInt;


class Constraints
{
public:
    Constraints() {}
    map<Agent, map<int, setPInt>> agent_constraints;
    Constraints fork(Agent agent, pairInt obstacle, int start, int end);
    map<int, setPInt> setdefault(Agent agent, map<int, setPInt> constraint);
    map<int, setPInt> getitem(Agent);
    vecAgent iter();

    friend std::ostream& operator<< (std::ostream& os, const Constraints& Constraints);
}; 


#endif
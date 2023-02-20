#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>

using namespace std;
typedef long long ll;
typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;
typedef vector<vecPInt> vec2PInt;

typedef vector<pairInt> Path;
typedef vector<vector<ll>> Map;

extern int cnt;

class Node
{
public:  
    Node(pairInt position)
        : position(position), length(0) {}
    Node(pairInt position, Node* parent)
        : position(position), parent(parent)
    {
        length = parent->length + 1;
    }

    Node* parent = nullptr;
    pairInt position;
    double f = 0;
    double g = 0;
    double h = 0;
    int length;

    bool operator==(const Node other);
    bool operator==(const Node* other);
    bool operator^(const vector<Node*>& list);

    friend ostream& operator<<(ostream& os, const Node* node);
};

# endif
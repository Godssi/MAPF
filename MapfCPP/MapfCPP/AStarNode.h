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
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

extern int cnt;

class Node
{
public:  
    Node(p position)
        : position(position), length(0) {}
    Node(p position, Node* parent)
        : position(position), parent(parent)
    {
        length = parent->length + 1;
    }

    Node* parent = nullptr;
    p position;
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
#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <algorithm>

using namespace std;
typedef long long ll;
typedef pair<int, int> p;
typedef vector<p> Path;
typedef vector<vector<ll>> Map;

class Node
{
public:  
    Node(p position)
        : position(position) {}
    Node(p position, Node* parent)
        : position(position), parent(parent) {}

    Node* parent = nullptr;
    p position;
    double f = 0;
    double g = 0;
    double h = 0;

    bool operator==(const Node other);
    bool operator==(const Node* other);
    bool operator^(const vector<Node*>& list);

    friend ostream& operator<<(ostream& os, const Node* node);
};

# endif
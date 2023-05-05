#include <mapf/AStar/Heuristic_func.h>

double heuristic_m(pairInt node, pairInt goal, double d = 1)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d * (dx + dy);
}

double heuristic_d(pairInt node, pairInt goal, double d1 = 1, double d2 = sqrt(2))
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return d1 * (dx + dy) + (d2 - 2 * d1) * min(dx, dy);
}

double heuristic_e(pairInt node, pairInt goal)
{
	int dx = abs(node.first - goal.first);
	int dy = abs(node.second - goal.second);
	return sqrt(dx * dx + dy * dy);
}

double get_heuristic_to_goal_sq(pairInt node, pairInt goal, double R, const Map& static_potential_map)
{
	int len = abs(goal.first - node.first);
	if (!len)
		return R;

	double h_add = R;
	// Quadrant 1
	int x, y;
	if (goal.first - node.first > 0 && goal.second - node.second > 0)
	{
		for (int i = 0; i < abs(goal.first - node.first) - 1; i++)
		{
			x = node.first + i;
			y = node.second + i;
			double r = sqrt(2) * i + node.first;
			h_add += (1 - pow(min(r, R) / (R + 0.01), 3)) * static_potential_map[x][y];
		}
	}
	// Quadrant 2
	else if (goal.first - node.first < 0 && goal.second - node.second > 0)
	{
		for (int i = 0; i < abs(goal.first - node.first) - 1; i++)
		{
			x = node.first - i;
			y = node.second + i;
			double r = sqrt(2) * i + node.first;
			h_add += (1 - pow(min(r, R) / (R + 0.01), 3)) * static_potential_map[x][y];
		}
	}
	// Quadrant 3
	else if (goal.first - node.first < 0 && goal.second - node.second < 0)
	{
		for (int i = 0; i < abs(goal.first - node.first) - 1; i++)
		{
			x = node.first - i;
			y = node.second - i;
			double r = sqrt(2) * i + node.first;
			h_add += (1 - pow(min(r, R) / (R + 0.01), 3)) * static_potential_map[x][y];
		}
	}
	// Quadrant 4
	else
	{
		for (int i = 0; i < abs(goal.first - node.first) - 1; i++)
		{
			x = node.first + i;
			y = node.second - i;
			double r = sqrt(2) * i + node.first;
			h_add += (1 - pow(min(r, R) / (R + 0.01), 3)) * static_potential_map[x][y];
		}
	}

	double h = R;
	h *= (h_add / len) * sqrt(R);
	return h;
}

double get_heuristic_to_goal_rect(pairInt node, pairInt goal, double R, const Map& static_potential_map)
{
	double h_add1 = 0, h_add2 = 0;
	ll upperCnt = 0, lowerCnt = 0;
	// Quadrant 1
	if (node.first <= goal.first && node.second <= goal.second)
	{
		for (int i = node.first; i < goal.first + 1; i++)
		{
			for (int j = node.second; j < goal.second + 1; j++)
			{
				if ((goal.second - node.second + 1) * (i - goal.first) >= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					lowerCnt++;
				}
			}
		}
	}
	// Quadrant 2
	else if (node.first >= goal.first && node.second <= goal.second)
	{
		for (int i = node.first; i > goal.first - 1; i--)
		{
			for (int j = node.second; j < goal.second + 1; j++)
			{
				if ((goal.second - node.second + 1) * (i - goal.first) >= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second + 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					lowerCnt++;
				}
			}
		}
	}
	// Quadrant 3
	else if (node.first >= goal.first && node.second >= goal.second)
	{
		for (int i = node.first; i > goal.first - 1; i--)
		{
			for (int j = node.second; j > goal.second - 1; j--)
			{
				if ((goal.second - node.second - 1) * (i - goal.first) >= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first - 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					lowerCnt++;
				}
			}
		}
	}
	// Quadrant 4
	else if (node.first <= goal.first && node.second >= goal.second)
	{
		for (int i = node.first; i < goal.first + 1; i++)
		{
			for (int j = node.second; j > goal.second - 1; j--)
			{
				if ((goal.second - node.second - 1) * (i - goal.first) >= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add1 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					upperCnt++;
				}
				if ((goal.second - node.second - 1) * (i - goal.first) <= (goal.first - node.first + 1) * (j - goal.second))
				{
					h_add2 += (1 - pow(sqrt(pow(i - node.first, 2) + pow(j - node.second, 2)) / (R + 0.01), 3)) * static_potential_map[i][j];
					lowerCnt++;
				}
			}
		}
	}

	if (!upperCnt || !lowerCnt)
		return R;

	h_add1 = (h_add1 / upperCnt) * sqrt(R);
	h_add2 = (h_add2 / lowerCnt) * sqrt(R);
	return min(h_add1, h_add2);
}

double heuristic_around_obstacle(pairInt node, pairInt goal, ll r, double R, const Map& map, const Map& static_potential_map)
{
	pair<ll, ll> map_size = { map.size() , map.front().size() };
	double h = 0;
	ll aroundCnt = 0;

	for (ll i = node.first - r; i < node.first + r + 1; i++)
	{
		for (ll j = node.second - r; j < node.second + r + 1; j++)
		{
			if ((0 <= i && i < map_size.first) && (0 <= j && j < map_size.second) && (r >= sqrt(pow(i - map_size.first, 2) + pow(j - map_size.second, 2))))
			{
				h += static_potential_map[i][j];
				aroundCnt++;
			}
		}
	}

	if (aroundCnt != 0) {
		h = (h / aroundCnt) * sqrt(R);
	}
	return h;
}

double heuristic(pairInt node, pairInt goal, const Map& map, const Map& static_potential_map, const Map& dynamic_potential_map)
{
	double h = heuristic_e(node, goal);
	double R = h;

	if (goal.first - node.first == goal.second - node.second)
		h = get_heuristic_to_goal_sq(node, goal, R, static_potential_map);
	else
		h = get_heuristic_to_goal_rect(node, goal, R, static_potential_map);

	double r = 3; // around radius
	h += heuristic_around_obstacle(node, goal, r, R, map, static_potential_map);
	h += heuristic_dynamic(node, dynamic_potential_map);
	return h;
}

double heuristic_dynamic(pairInt node, const Map& dynamic_potential_map)
{
	return dynamic_potential_map[node.first][node.second];
}
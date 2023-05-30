#include "PathAnalyzer.h"

void getPathLength(vec2PInt result)
{
	vector<int> length;
	cout << "Path Length :";
	for (auto it = result.begin(); it != result.end(); it++)
	{
		if (it->size() != 0)
		{
			int x = it->front().first;
			int y = it->front().second;
			for (int i = 1; i < it->size(); i++)
			{
				if ((*it)[i].first != x || (*it)[i].second != y)
				{
					x = (*it)[i].first;
					y = (*it)[i].second;
				}
				else
				{
					if (x == -1 && y == -1)
					{
						cout << " " << 0;
						length.push_back(0);
						break;
					}
					cout << " " << i;
					length.push_back(i);
					break;
				}
			}
		}
	}
	cout << "\n";
	cout << "Average Length : " << accumulate(length.begin(), length.end(), 0.0) / length.size() << "\n";
}

void nearestDistance2Obstacle(vec2PInt result)
{
    vector<pairInt> Ob = { {2, 81} , {25, 48}, {51, 6}};   // {2, 2}
    Map map = MAP_GEN::test_maze_gen();
    pair<ll, ll> map_size = { map.size() , map[0].size() };
    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            //if (map[i][j] == Static_Ob)
            //{
            //    Ob.push_back({ i, j });
            //}
            if (map[i][j] == Dynamic_Ob)
            {
               Ob.push_back({ i, j });
            }
        }
    }

    vector<double> NDO;
    for (auto iter1 = result.begin(); iter1 != result.end(); iter1++)
    {
        double distance = MAX;
        for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
        {
            for (auto obIter = Ob.begin(); obIter != Ob.end(); obIter++)
            {
                distance = min(distance, sqrt(pow(iter2->first - obIter->first, 2) + pow(iter2->second - obIter->second, 2)));
            }
        }
        NDO.push_back(distance);
    }

    int i = 1;
    cout << "NDO\n";
    for (auto iter = NDO.begin(); iter != NDO.end(); iter++)
    {
        cout << "agent " << i << " : " << (*iter) << "\n";
        i++;
    }
    cout << "\n";
}

void averageDistance2Obstacle(vec2PInt result)
{
    vector<pairInt> Ob = { {2, 81} , {25, 48}, {51, 6} };
    Map map = MAP_GEN::test_maze_gen();
    pair<ll, ll> map_size = { map.size() , map[0].size() };
    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            //if (map[i][j] == Static_Ob)
            //{
            //    Ob.push_back({ i, j });
            //}
            if (map[i][j] == Dynamic_Ob)
            {
               Ob.push_back({ i, j });
            }
        }
    }

    vector<vector<double>> NDO;
    for (auto iter1 = result.begin(); iter1 != result.end(); iter1++)
    {
        vector<double> Dis;
        for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
        {
            double distance = MAX;
            for (auto obIter = Ob.begin(); obIter != Ob.end(); obIter++)
            {
                distance = min(distance, sqrt(pow(iter2->first - obIter->first, 2) + pow(iter2->second - obIter->second, 2)) + 1);
            }
            Dis.push_back(distance);
        }
        NDO.push_back(Dis);
    }

    vector<double> Dis;
    cout << "ADO\n";
    int i = 1;
    for (auto iter = NDO.begin(); iter != NDO.end(); iter++)
    {
        double tmp = accumulate(iter->begin(), iter->end(), 0.0) / iter->size();
        cout << "agent " << i << " : " << tmp << "\n";
        Dis.push_back(tmp);
        i++;
    }
    cout << "total : " << accumulate(Dis.begin(), Dis.end(), 0.0) / Dis.size() << "\n";
}
#include "Map_gen.h"
#include <cstdlib>
#include <ctime>
#include <cmath>

Map potential_map_generator(Map map)
{
    pair<ll, ll> map_size = { map.size() , map[0].size()};
    Map potential_map (map_size.first, vector<ll>(map_size.second, 0));

    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            if (map[i][j] == 0)
            {
                continue;
            }
            else if(map[i][j] == 1)
            {
                int outer_r = 10;
                int inner_r = 2;
                int alpha = 10;
                
                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        int r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m< map_size.second))
                        {
                            r = sqrt(r);
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }   
                }
            }
            else if (map[i][j] == 2)
            {
                int outer_r = 7;
                int inner_r = 2;
                int alpha = 28;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        int r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = sqrt(r);
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }
                }
            }
            else if (map[i][j] == 3)
            {
                int outer_r = 5;
                int inner_r = 2;
                int alpha = 4;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        int r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = sqrt(r);
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }
                }
            }
        }
    }
    return potential_map;
}

Map test_maze_gen()
{
    Map map = { {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 3},
                {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3} };
    return map;
}

Map random_maze_gen()
{
    srand((unsigned int)time(NULL));
    pair<ll, ll> map_size = { rand() % 20 + 30, rand() % 20 + 30 };

    Map map(map_size.first, vector<ll>(map_size.second, 0));
    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            map[i][j] = rand() % 500 / 495;
        }
    }

    for (ll i = 0; i < map_size.first; i++)
    {
        map[i][0] = 3;
        map[i][map_size.second - 1] = 3;
    }

    for (ll i = 0; i < map_size.second; i++)
    {
        map[0][i] = 3;
        map[map_size.first - 1][i] = 3;
    }

    for (int i = 1; i < 3; i++)
    {
        for (int j = 1; j < 3; j++)
        {
            map[i][j] = 0;
        }
    }
    return map;
}
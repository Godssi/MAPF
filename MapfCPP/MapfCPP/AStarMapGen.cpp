#include "AStarMapGen.h"

Map MAP_GEN::potential_map_generator(Map map)
{
    pair<ll, ll> map_size = { map.size() , map[0].size()};
    Map potential_map(map_size.first, vector<ll>(map_size.second, 0));

    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            if (map[i][j] == Ground)
            {
                continue;
            }
            else if (map[i][j] == Outer_Wall)
            {
                int outer_r = 4;
                int inner_r = 1;
                int alpha = 4;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        ll r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = static_cast<ll>(round(sqrt(r)));
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }
                }
            }
            else if(map[i][j] == Inner_Wall)
            {
                int outer_r = 4;
                int inner_r = 1;
                int alpha = 4;
                
                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        ll r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m< map_size.second))
                        {
                            r = static_cast<ll>(round(sqrt(r)));
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }   
                }
            }
            else if (map[i][j] == Static_Ob)
            {
                int outer_r = 8;
                int inner_r = 1;
                int alpha = 10;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        ll r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = static_cast<ll>(round(sqrt(r)));
                            if (r < 1e-3)
                                r = 1;
                            potential_map[k][m] += (1 / (outer_r - inner_r)) *
                                ((inner_r * outer_r * (alpha - 1)) / r + outer_r - inner_r * alpha);
                        }
                    }
                }
            }
            else if (map[i][j] == Dynamic_Ob)
            {
                int outer_r = 24;
                int inner_r = 3;
                int alpha = 28;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        ll r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = static_cast<ll>(round(sqrt(r)));
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

Map MAP_GEN::modify_potential_map(const Map& map, const Map& static_potential_map)
{
    pair<ll, ll> map_size = { map.size() , map[0].size() };
    Map potential_map = static_potential_map;

    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            if (map[i][j] == Dynamic_Ob)
            {
                int outer_r = 24;
                int inner_r = 3;
                int alpha = 28;

                for (ll k = i - outer_r; k < i + outer_r + 1; k++)
                {
                    for (ll m = j - outer_r; m < j + outer_r + 1; m++)
                    {
                        ll r = (k - i) * (k - i) + (m - j) * (m - j);
                        if (r <= inner_r * inner_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            potential_map[k][m] += alpha;
                        }
                        else if (r <= outer_r * outer_r && (0 <= k && k < map_size.first) && (0 <= m && m < map_size.second))
                        {
                            r = static_cast<ll>(round(sqrt(r)));
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

Map MAP_GEN::moving_obstacle_to_origin_map(Map map, const vecPInt& movePoint)
{
    pair<ll, ll> map_size = { map.size() , map.front().size()};

    for (ll i = 0; i < map_size.first; i++)
    {
        for (ll j = 0; j < map_size.second; j++)
        {
            if (map[i][j] == Dynamic_Ob)
                map[i][j] = Ground;
        }
    }
    for (auto& it : movePoint)
    {
        if (map[it.first][it.second] == Ground)
            map[it.first][it.second] = Dynamic_Ob;
    }
    return map;
}

Map MAP_GEN::test_maze_gen1()
{
    //  y        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42      x
    Map map = { {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 0
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 1
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 2
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 3
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 4
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 5
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 6
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 7
                {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1},  // 8
                {1, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 2, 2, 1},  // 9
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 10
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 11
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 12
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 13
                {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 14
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 15
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 16
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 17
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 18
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 19
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 20
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 21
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 22
                {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 23
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 24
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 25
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 26
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 27
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 28
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 29
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 30
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 31
                {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 32
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 33
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 34
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 35
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 36
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 37
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 38
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 39
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 40
                {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 41
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 42
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 43
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 44
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 1},  // 45
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 0, 1},  // 46
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 47
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 48
                {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},  // 49
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };// 50
    //  y        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42
    return map;
}


Map MAP_GEN::test_maze_gen2()
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

Map MAP_GEN::random_maze_gen()
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

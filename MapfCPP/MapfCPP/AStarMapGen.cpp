#include "AStarMapGen.h"

Map MAP_GEN::potential_map_generator(const Map& map)
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
                ll alpha = 1;
                potential_map[i][j] += alpha;
            }
            else if(map[i][j] == Inner_Wall)
            {
                ll alpha = 1;
                potential_map[i][j] += alpha;
            }
            else if (map[i][j] == Static_Ob)
            {
                ll outer_r = 8;
                ll inner_r = 1;
                ll alpha = 10;

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

// dynamicPotentialMap�� �� �� ������ �� ���� ����� ���;� ������ ������ ��ü ��ο� ���ؼ� ������ dynamicPotentialMap�� ���´�.
// ���� ���� ����� ������ Data ó���� ���� ������ ��� ���ܳ�.
Map MAP_GEN::dynamic_potential_map(const Map& map, const vector<DynamicObstacle>& dynamic_obstacles)
{
    // dynamic_potential_map �� ����� �ڵ�
    pair<ll, ll> map_size = { map.size() , map[0].size() };
    Map dynamicPotentialMap(map_size.first, vector<ll>(map_size.second, 0));  // ������ dynamicPotentialMap���� dynamic_object�� ���� �� ���� ��� �� ��� object�� ���� potential�� ��� �ִ� map

    // potential map�� �������ִ� �ڵ�
    for (auto ob_iter = dynamic_obstacles.begin(); ob_iter != dynamic_obstacles.end(); ob_iter++)
    {
        // �������� �����̴� ��ֹ��� ���� �ӵ� ���� ���͸� �м��Ͽ��� potential map ����
        Map present_dynamicPotentialMap(map_size.first, vector<ll>(map_size.second, 0)); // ���� ��ġ������ ������ ���ư� ���⿡ ���� potential map �⺻��

        int x_pos = (*ob_iter).x_pos; // ���� ��ֹ��� ���� x ��ġ
        int y_pos = (*ob_iter).y_pos; // ���� ��ֹ��� ���� y ��ġ
		int direct = (*ob_iter).velo_theta; // ���� ��ֹ��� �ش� ��ġ������ �ӵ� ���� ����
        double speed = (*ob_iter).velocity; // ���� ��ֹ��� �����ӵ�

		// Ellipse_equation���� ���ٺ� ��ǥ���� range
		int x_range_left = (x_pos - 2 * speed < 0) ? 0 : x_pos - 2 * speed;
		int x_range_right = (x_pos + 5 * speed >= map_size.first) ? map_size.first - 1 : x_pos + 5 * speed;
		int y_range_left = (y_pos - 2 * speed < 0) ? 0 : y_pos - 2 * speed;
		int y_range_right = (y_pos + 5 * speed >= map_size.second) ? map_size.second - 1 : y_pos + 5 * speed;

		// Ÿ���� ������ �κ�
		for (auto x_iter = x_range_left; x_iter <= x_range_right; x_iter++)
		{
			for (auto y_iter = y_range_left; y_iter <= y_range_right; y_iter++) {
				int search_x = x_iter;
				int search_y = y_iter;

				// ū Ÿ���� ����ġ �ο�
				if (Ellipse_equation(x_pos, y_pos, search_x, search_y, direct, speed, 'B')) {
                    dynamicPotentialMap[x_iter][y_iter] += 15;
					present_dynamicPotentialMap[x_iter][y_iter] += 15;
				}

				// ���� Ÿ���� ����ġ �ο�
				if (Ellipse_equation(x_pos, y_pos, search_x, search_y, direct, speed, 'S'))
                    dynamicPotentialMap[x_iter][y_iter] += 15;
					present_dynamicPotentialMap[x_iter][y_iter] += 15;
			}
		}

        // �׻� ����� �� �� ���´�...
		cout << "\n\t\t\t\tPresent Dynamic Potential Map\n";
		for (auto iter1 = present_dynamicPotentialMap.begin(); iter1 != present_dynamicPotentialMap.end(); iter1++)
		{
			cout << "\n\t";
			for (auto iter2 = iter1->begin(); iter2 != iter1->end(); iter2++)
			{
				cout.width(2);
				cout.fill('0');
				cout << *iter2 << " ";
			}
		}
        cout << "\n";
	}

    return dynamicPotentialMap;
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

Map MAP_GEN::test_maze_gen()
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

double MAP_GEN::cosine_degree(int degree)
{
    return cos(degree * PI / 180);
}

double MAP_GEN::sine_degree(int degree)
{
    return sin(degree * PI / 180);
}

bool MAP_GEN::Ellipse_equation(int x, int y, int search_x, int search_y, int direct, int speed, char Big_Small) 
{ // Ellipse_equation �̶� �ش� ��ǥ�� Ÿ�� �ȿ� �����ϴ��� Ȯ���ϴ��� �ľ��ϴ� �Լ�
  // x, y�� Dynamic_Object�� ��ġ, search_x, search_y�� ã�� ���� ��ǥ, direct�� ����, speed : dynamic Object�� �ӵ�, Big_Small�� Ÿ�� ������ �Ǵ� ��
    if (Big_Small == 'B')
    {  
        // Ÿ���� �߽���
        double center_x = x + 1.5 * speed * cosine_degree(45 * (direct-1));  // p ��
        double center_y = y + 1.5 * speed * sine_degree(45 * (direct - 1));  // q ��

        int a1 = 3.5 * speed;
        int b1 = 2 * speed;

        double a1_part = pow(cosine_degree(45 * (direct - 1)) * (search_x - center_x) + sine_degree(search_y - center_y), 2);
        double b1_part = pow( (-1) * sine_degree(45 * (direct - 1)) * (search_x - center_x) + cosine_degree(search_y - center_y), 2);
        if ( (a1_part / pow(a1,2)) + (b1_part / pow(b1,2)) <= 1)
            return true;
        else return false;
    }
    else if (Big_Small == 'S')
    {
        double center_x = x + 0.75 * speed * cos((45 * (direct - 1)) * PI / 180);  // p ��
        double center_y = y + 0.75 * speed * sin((45 * (direct - 1)) * PI / 180);  // q ��

        int a2 = 1.75 * speed;
        int b2 = speed;

        double a2_part = pow(cosine_degree(45 * (direct - 1)) * (search_x - center_x) + sine_degree(search_y - center_y), 2);
        double b2_part = pow((-1) * sine_degree(45 * (direct - 1)) * (search_x - center_x) + cosine_degree(search_y - center_y), 2);
        if ((a2_part / pow(a2, 2)) + (b2_part / pow(b2, 2)) <= 1)
            return true;
        else return false;
    }
}


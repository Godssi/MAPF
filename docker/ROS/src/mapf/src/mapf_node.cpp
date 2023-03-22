#include <mapf/mapf.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapf");

	MAPFController MAPFcontroller;
	vector<pairInt> start = { {2, 2}, {2, 32} };
	vector<pairInt> goal = { {14, 39}, {49, 20} };
	vector<pairInt> static_obstacle = { {24, 12}, {10, 20} };  
	dynamicOb dynamic_obstacle = { {{{6, 6}, {0, 1}}, 3} };
	MAPFcontroller.planner.set_planner(start, goal, 1, static_obstacle, dynamic_obstacle);

	ros::spin();
	return 0;
}
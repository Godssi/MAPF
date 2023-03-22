#ifndef MAPF_H
#define MAPF_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <robot_odom/MultiAgentOdom.h>
#include <mapf/MultiTargetPose.h>
#include <mapf/GoalPose.h>

#include <string>
#include <vector>

#include <mapf/Planner.h>

using namespace std;

typedef pair<int, int> pairInt;
typedef vector<pairInt> vecPInt;

class MAPFController
{
public:
	int n_robot;
	bool mapf_flag = false;
	Planner planner;
	string assigner;
	mapf::GoalPose _goalPose;
	mapf::MultiTargetPose _multiTargetPose;

	ros::NodeHandle _nh;
	ros::Subscriber _goal_sub;
	ros::Subscriber _odom_sub;
	ros::Publisher _path_pub;
	
public:
	MAPFController(int robot_radius = 1, int low_level_max_iter = 1000);
	MAPFController(vecPInt starts, vecPInt goals, int robot_radius, vecPInt static_obstacle, dynamicOb dynamic_obstacle, int low_level_max_iter = 1000);
	~MAPFController() {};

	void resizeMsg();
	void odomCallback(const robot_odom::MultiAgentOdom& multiAgentOdom);
	void goalCallback(const mapf::GoalPose& goalPose);
	void publish();
	void setMaxCore();
};

#endif
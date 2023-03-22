#include <mapf/mapf.h>

MAPFController::MAPFController(int robot_radius, int low_level_max_iter)
{
	planner.set_planner(robot_radius);
	_nh.getParam("/n_robot", n_robot);
	resizeMsg();

	_goal_sub = _nh.subscribe("unity/agent/goal", 1, &MAPFController::goalCallback, this);
	_odom_sub = _nh.subscribe("agent/multi_odom", 1, &MAPFController::odomCallback, this);
	_path_pub = _nh.advertise<mapf::MultiTargetPose>("agent/path", 1);
}

MAPFController::MAPFController(vecPInt starts, vecPInt goals, int robot_radius, vecPInt static_obstacle, dynamicOb dynamic_obstacle, int low_level_max_iter)
{
	planner.set_planner(starts, goals, robot_radius, static_obstacle, dynamic_obstacle, low_level_max_iter);
	_nh.getParam("/n_robot", n_robot);
	resizeMsg();

	_goal_sub = _nh.subscribe("unity/agent/goal", 1, &MAPFController::goalCallback, this);
	_odom_sub = _nh.subscribe("agent/multi_odom", 1, &MAPFController::odomCallback, this);
	_path_pub = _nh.advertise<mapf::MultiTargetPose>("agent/path", 1);
}

void MAPFController::odomCallback(const robot_odom::MultiAgentOdom& multiAgentOdom)
{
	vecPInt updateStarts;
	for (int i = 0; i < n_robot; i++)
	{
		updateStarts.push_back({ multiAgentOdom.pose[i].position.x, multiAgentOdom.pose[i].position.y });
		if (updateStarts[i].first != 0 && updateStarts[i].second != 0)
			mapf_flag = true;
	}
	planner.set_starts(updateStarts);

	if (mapf_flag)
	{
		if (planner.validate_agent_position())
		{
			ROS_INFO("Not vaild robot position");
		}
		else
		{
			vec2PInt result = planner.plan();
			for (int i = 0; i < n_robot; i++)
			{
				_multiTargetPose.header[i] = multiAgentOdom.header[i];
				if (result[i].size() > 0)
				{
					_multiTargetPose.pose[i].position.x = result[i][1].first;
					_multiTargetPose.pose[i].position.y = result[i][1].second;
				}
				else
				{
					_multiTargetPose.pose[i].position.x = result[i][0].first;
					_multiTargetPose.pose[i].position.y = result[i][0].second;
				}
				// ROS_INFO("Goal: (%d, %d)", planner.get_goals()[i].first, planner.get_goals()[i].second);
			}
			publish();
		}
	}
}

void MAPFController::goalCallback(const mapf::GoalPose& goalPose)
{
	vecPInt updateGoals;
	for (int i = 0; i < n_robot; i++)
	{
		updateGoals.push_back({ goalPose.pose[i].position.x, goalPose.pose[i].position.y });
	}
	planner.set_goals(updateGoals);

	if (mapf_flag)
	{
		if (planner.validate_agent_position())
		{
			ROS_INFO("Not vaild robot position");
		}
		else
		{
			ROS_INFO("A new target point has been registered");
		}
	}
}

void MAPFController::publish()
{
	_path_pub.publish(_multiTargetPose);
}

void MAPFController::setMaxCore()
{
	planner.set_max_core();
}

void MAPFController::resizeMsg()
{
	// resize Goal Msg
	_goalPose.pose.resize(n_robot);

	// resize Target Msg
    _multiTargetPose.header.resize(n_robot);
    _multiTargetPose.pose.resize(n_robot);
}

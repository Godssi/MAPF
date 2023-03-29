#include <move_control/move_control.h>

MoveControl::MoveControl()
{
    _nh.getParam("/exist_path", _exist_path);
    _nh.getParam("/n_robot", n_robot);
    resizeMsgs();
    _pidController = new PIDController[n_robot];

    _cmd_pub = _nh.advertise<move_control::MultiTwist>("/unity/agent/cmd_vel", 1);
    _odom_sub = _nh.subscribe("/agent/multi_odom", 1, &MoveControl::odomCallback, this);
    _path_sub = _nh.subscribe("/agent/path", 1, &MoveControl::pathCallback, this);
    pre_time = ros::Time::now();
}

void MoveControl::publish()
{
    if (_exist_path)
    {
        for (int i = 0; i < n_robot; i++)
        {
            time = ros::Time::now();
            p cmd = _pidController[i].calQ( _multiAgentOdom.pose[i], _multiTargetPose.pose[i] );
            _pidController[i].updateDeltaTime((time - pre_time).toSec());
            _multiTwist.linear[i].x = cmd.first;
            _multiTwist.angular[i].z = cmd.second;
        }
        _cmd_pub.publish(_multiTwist);
        pre_time = time;
    }
    else
    {
        // Random Target Position
        for (int i = 0; i < n_robot; i++)
        {
            geometry_msgs::Pose pose;
            pose.position.x = 20;
            pose.position.y = 20;
            p cmd = _pidController[i].calQ( _multiAgentOdom.pose[i], pose );
            _multiTwist.linear[i].x = cmd.first;
            _multiTwist.angular[i].z = cmd.second;
        }
        _cmd_pub.publish(_multiTwist);
    }
}

void MoveControl::odomCallback(const robot_odom::MultiAgentOdom& multiAgentOdom)
{
    _multiAgentOdom = multiAgentOdom;
}

void MoveControl::pathCallback(const mapf::MultiTargetPose& multiTargetPose)
{
    for (int i = 0; i < n_robot; i++)
    {
        _multiTargetPose.pose[i] = multiTargetPose.pose[i];
        _multiTargetPose.header[i] = multiTargetPose.header[i];
    }
}

void MoveControl::resizeMsgs()
{
    // resize Twist Msgs
    _multiTwist.linear.resize(n_robot);
    _multiTwist.angular.resize(n_robot);

    // resize Odom Msgs
    _multiAgentOdom.header.resize(n_robot);
    _multiAgentOdom.pose.resize(n_robot);
    _multiAgentOdom.twist.resize(n_robot);

    // resize Path Msgs
    _multiTargetPose.header.resize(n_robot);
    _multiTargetPose.pose.resize(n_robot);
}

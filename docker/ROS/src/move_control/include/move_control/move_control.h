#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <robot_odom/MultiAgentOdom.h>
#include <mapf/MultiTargetPose.h>
#include <move_control/MultiTwist.h>

#include <move_control/pid_controller.h>

#include <ctime>

typedef std::pair<double, double> p;

class MoveControl
{
private:
    int n_robot;
    PIDController* _pidController;
    move_control::MultiTwist _multiTwist;
    robot_odom::MultiAgentOdom _multiAgentOdom;
    mapf::MultiTargetPose _multiTargetPose;

    ros::NodeHandle _nh;
    ros::Publisher _cmd_pub;
    ros::Subscriber _odom_sub;
    ros::Subscriber _path_sub;
    bool _exist_path;

public:
    MoveControl();
    ~MoveControl() {};

    void publish();
    void resizeMsgs();
    void odomCallback(const robot_odom::MultiAgentOdom& multiAgentOdom);
    void pathCallback(const mapf::MultiTargetPose& multiTargetPose);
};

#endif
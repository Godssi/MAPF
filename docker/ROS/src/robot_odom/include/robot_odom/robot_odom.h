#ifndef MOBILE_ROBOT_ODOM_H
#define MOBILE_ROBOT_ODOM_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <robot_odom/MultiAgentOdom.h>
#include <robot_odom/MultiPoseStamped.h>
#include <robot_odom/MultiTwistStamped.h>

#include <cmath>
#include <ctime>
#include <string>


class OdometryNode
{
private:
    int n_robot;
    robot_odom::MultiAgentOdom multiOdom;

    tf::TransformBroadcaster _br;
    tf::Transform _transform;
    tf::Quaternion _orientation;
    tf::Vector3 _linearVelocity;
    tf::Vector3 _angularVelocity;

    ros::NodeHandle _nh;
    ros::Publisher _odom_pub;
    ros::Subscriber _pose_sub;
    ros::Subscriber _twist_sub;

public:
    OdometryNode();
    ~OdometryNode() {};

    void resizeMultiOdomMsg();
    void poseCallback(const robot_odom::MultiPoseStamped& multiPoseStamped);
    void twistCallback(const robot_odom::MultiTwistStamped& multiTwistStamped);
    void publish();
};


#endif
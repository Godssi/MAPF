#include <robot_odom/robot_odom.h>

OdometryNode::OdometryNode()
{
    // _pose_sub = _nh.subscribe("unity/mobile_robot_pose/1", 1, &OdometryNode::poseCallback, this);
    // _twist_sub = _nh.subscribe("unity/model_twist", 1, &OdometryNode::twistCallback, this);

    _nh.getParam("/n_robot", n_robot);
    resizeMultiOdomMsg();

    _odom_pub = _nh.advertise<robot_odom::MultiAgentOdom>("agent/multi_odom", 1);
    _pose_sub = _nh.subscribe("unity/mobile_robot_pose", 1, &OdometryNode::poseCallback, this);
    _twist_sub = _nh.subscribe("unity/mobile_robot_twist", 1, &OdometryNode::twistCallback, this);
}

void OdometryNode::resizeMultiOdomMsg()
{
    multiOdom.header.resize(n_robot);
    multiOdom.pose.resize(n_robot);
    multiOdom.twist.resize(n_robot);
}

void OdometryNode::poseCallback(const robot_odom::MultiPoseStamped& multiPoseStamped)
{
    if(n_robot == multiPoseStamped.n_robot.data)
    {
        for (int i = 0; i < n_robot; i++)
        {
            multiOdom.header[i] = multiPoseStamped.header[i];
            multiOdom.pose[i] = multiPoseStamped.pose[i];
        }
    }
    else
    {
        ROS_INFO("Num of robot is not matched");
    }
}

void OdometryNode::twistCallback(const robot_odom::MultiTwistStamped& multiTwistStamped)
{
    if(n_robot == multiTwistStamped.n_robot.data)
    {
        for (int i = 0; i < n_robot; i++)
        {
            multiOdom.header[i] = multiTwistStamped.header[i];
            multiOdom.twist[i] = multiTwistStamped.twist[i];
        }
    }
    else
    {
        ROS_INFO("Num of robot is not matched");
    }
}

void OdometryNode::publish()
{
    _odom_pub.publish(multiOdom);
}
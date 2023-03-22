#include <robot_odom/robot_odom.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_odom");

    OdometryNode odometryNode;

    ros::Rate spinRate(1);
    while (ros::ok())
    {
        odometryNode.publish();
        spinRate.sleep();
        ros::spinOnce();
    }
    return 0;
}
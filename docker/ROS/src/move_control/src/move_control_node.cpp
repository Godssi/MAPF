#include <move_control/move_control.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_control");
    MoveControl moveControl;
    
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        moveControl.publish();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
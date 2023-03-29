#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <mapf/MultiTargetPose.h>

#include <cmath>
#include <ctime>
#include <utility>

typedef std::pair<double, double> p;
const double pi = 3.14159265358979;

class PIDController
{
private:
    double _kp[3] = { 0.7, 0.2, 0.2 };
    double _ki[3] = { 0.004, 0.005, 0.001 };
    double _kd[3] = { 0.045, 0.04, 0.007 };

    double _maxQ[3] = { 1, 1, 1 };
    double _minQ[3] = { -1, -1, -1 };

    double _preErr[3] = { 0.0, 0.0, 0.0 };
    double _intErr[3] = { 0.0, 0.0, 0.0 };

    double _dt[3] = { 1, 1, 1 };

    double _overshoot[3];
    double _decayratio[3];
    double _oscillPeriod[3];
    double _tuningInterval[3];

public:
    PIDController() {};
    PIDController(double kp[3], double ki[3], double kd[3], double maxQ[3], double minQ[3]);
    p calQ(const geometry_msgs::Pose& cur, const geometry_msgs::Pose& goal);
    void updateDeltaTime();
    void updateDeltaTime(double updateTime);
    void tuning_gain(double kp[3], double ki[3], double kd[3]);
    void clearController();

    double cutOff(double q, double max, double min);
    double calDist(double p1[3], double p2[3]);
    double calThetaAng(double p[3]);
    double calThetaAng(double x, double y);
    double calAlphaAng(double p1[3], double p2[3], double ang);
    double calBetaAng(double goalAng, double poseAng, double theta);
    double rad2rad(double q);
};

#endif
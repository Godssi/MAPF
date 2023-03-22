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
    p _kp = { 0.0, 0.0 };
    p _ki = { 0.0, 0.0 };
    p _kd = { 0.0, 0.0 };

    p _maxQ = { 0.0, 0.0 };
    p _minQ = { 0.0, 0.0 };

    double _preErr[2] = {};
    double _intErr[2] = {};

    double _dt[2] = { 1, 1 };

    double _overshoot[2] = {};
    double _decayratio[2] = {};
    double _oscillPeriod[2] = {};
    double _tuningInterval[2] = {};

public:
    PIDController(p kp = { 1, 0.88 }, p ki = { 0.004, 0.005 }, p kd = { 0.045, 0.08 }, p maxQ = { 1, 2 }, p minQ = { -1, -2 });
    p calQ(const geometry_msgs::Pose& cur, const geometry_msgs::Pose& goal);
    void updateDeltaTime(double updateTime[]);
    void tuning_gain(p kp, p ki, p kd);
    void clearController();

    double curOff(double q, double max, double min);
    double calDist(p p1, p p2);
    double calAng(p p1, p p2, double ang);
};

#endif
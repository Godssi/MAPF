#include <move_control/pid_controller.h>

PIDController::PIDController(double kp[3], double ki[3], double kd[3], double maxQ[3], double minQ[3])
{
    this->_kp[0] = kp[0];
    this->_kp[1] = kp[1];
    this->_kp[2] = kp[2];
    this->_ki[0] = ki[0];
    this->_ki[1] = ki[1];
    this->_ki[2] = ki[2];
    this->_kd[0] = kd[0];
    this->_kd[1] = kd[1];
    this->_kd[2] = kd[2];

    this->_maxQ[0] = maxQ[0];
    this->_maxQ[1] = maxQ[1];
    this->_maxQ[2] = maxQ[2];
    this->_minQ[0] = minQ[0];
    this->_minQ[1] = minQ[1];
    this->_minQ[2] = minQ[2];
}

p PIDController::calQ(const geometry_msgs::Pose& cur, const geometry_msgs::Pose& goal)
{
    // tf::Quaternion curQuaternion(
    //     cur.orientation.x,
    //     cur.orientation.y,
    //     cur.orientation.z,
    //     cur.orientation.w);
    // double curPose[3] = { cur.position.x, cur.position.y, tf::getYaw(curQuaternion) };
    // double goalPose[3] = { goal.position.x, goal.position.y, calThetaAng(goal.position.x, goal.position.y) };

    // double errDist = calDist(goalPose, curPose);
    // double errAlphaAng = calAlphaAng(goalPose, curPose, curPose[2]);
    // double errBetaAng = calBetaAng(goalPose[2], curPose[2], errAlphaAng);

    // if (errAlphaAng > pi / 2)
    // {
    //     errAlphaAng = rad2rad(pi - errAlphaAng);
    //     errBetaAng = rad2rad(pi - errBetaAng);
    // }

    // double kp[3] = { _kp[0] * errDist, _kp[1] * errAlphaAng, _kp[2] * errBetaAng };

    // _intErr[0] += _kp[0] * errDist * _dt[0];
    // _intErr[1] += _kp[1] * errAlphaAng * _dt[1];
    // _intErr[2] += _kp[2] * errBetaAng * _dt[2];
    // double ki[3] = { _ki[0] * _intErr[0], _ki[1] * _intErr[1], _ki[2] * _intErr[2] };

    // double kd[3] = { (errDist - _preErr[0] / _dt[0]), (errAlphaAng - _preErr[1] / _dt[1]), (errBetaAng - _preErr[2] / _dt[2]) };
    // kd[0] = _kd[0] * kd[0];
    // kd[1] = _kd[1] * kd[1];
    // kd[2] = _kd[2] * kd[2];

    // p cmd = { kp[0] + ki[0] + kd[0], kp[1] + ki[1] + kd[1] + kp[2] + ki[2] + kd[2] };
    // cmd.first = cmd.first > _maxQ[0] ? _maxQ[0] : (cmd.first < _minQ[0] ? _minQ[0] : cmd.first);
    // cmd.second = cmd.second > _maxQ[1] ? _maxQ[1] : (cmd.second < _minQ[1] ? _minQ[1] : cmd.second);

    // // print pid gain and velocity
    // ROS_INFO(" cur: %f, %f, %f", curPose[0], curPose[1], curPose[2]);
    // ROS_INFO("goal: %f, %f, %f", goalPose[0], goalPose[1], goalPose[2]);
    // ROS_INFO(" err: %f, %f, %f", errDist, errAlphaAng, errBetaAng);
    // ROS_INFO(" lienar kp: %f, ki: %f, kd: %f", kp[0], ki[0], kd[0]);
    // ROS_INFO("angular kp: %f, ki: %f, kd: %f", kp[1], ki[1], kd[1]);
    // ROS_INFO("cmd_vel\n                                     linear x: %f\n                                    angular z: %f", cmd.first, cmd.second);

    tf::Quaternion curQuaternion(
        cur.orientation.x,
        cur.orientation.y,
        cur.orientation.z,
        cur.orientation.w);
    double curPose[3] = { cur.position.x, cur.position.y, tf::getYaw(curQuaternion) };
    double goalPose[3] = { goal.position.x, goal.position.y, calThetaAng(goal.position.x, goal.position.y) };

    double errDist = calDist(goalPose, curPose);
    double errAlphaAng = calAlphaAng(goalPose, curPose, curPose[2]);
    double errBetaAng = calBetaAng(goalPose[2], curPose[2], errAlphaAng);

    if (errAlphaAng > pi / 2)
    {
        errAlphaAng = rad2rad(pi - errAlphaAng);
        errBetaAng = rad2rad(pi - errBetaAng);
    }

    double kp[3] = { _kp[0] * errDist, _kp[1] * errAlphaAng, _kp[2] * errBetaAng };

    double kd[3] = { (errDist - _preErr[0] / _dt[0]), (errAlphaAng - _preErr[1] / _dt[1]), (errBetaAng - _preErr[2] / _dt[2]) };
    kd[0] = _kd[0] * kd[0];
    kd[1] = _kd[1] * kd[1];
    kd[2] = _kd[2] * kd[2];

    p cmd = { kp[0] + kd[0], kp[1] + kd[1] + kp[2] + kd[2] };
    cmd.first = cmd.first > _maxQ[0] ? _maxQ[0] : (cmd.first < _minQ[0] ? _minQ[0] : cmd.first);
    cmd.second = cmd.second > _maxQ[1] ? _maxQ[1] : (cmd.second < _minQ[1] ? _minQ[1] : cmd.second);

    // print pid gain and velocity
    ROS_INFO(" cur: %f, %f, %f", curPose[0], curPose[1], curPose[2]);
    ROS_INFO("goal: %f, %f, %f", goalPose[0], goalPose[1], goalPose[2]);
    ROS_INFO(" err: %f, %f, %f", errDist, errAlphaAng, errBetaAng);
    ROS_INFO(" lienar kp: %f, kd: %f", kp[0], kd[0]);
    ROS_INFO("angular kp: %f, kd: %f", kp[1], kd[1]);
    ROS_INFO("cmd_vel\n                                     linear x: %f\n                                    angular z: %f", cmd.first, cmd.second);

    return cmd;
}

void PIDController::updateDeltaTime()
{
    _dt[0] += 1;
    _dt[1] += 1;
    _dt[2] += 1;
}

void PIDController::updateDeltaTime(double updateTime)
{
    _dt[0] += updateTime;
    _dt[1] += updateTime;
    _dt[2] += updateTime;
}

void PIDController::tuning_gain(double kp[3], double ki[3], double kd[3])
{
    _kp[0] = kp[0];
    _kp[1] = kp[1];
    _kp[2] = kp[2];
    _ki[0] = ki[0];
    _ki[1] = ki[1];
    _ki[2] = ki[2];
    _kd[0] = kd[0];
    _kd[1] = kd[1];
    _kd[2] = kd[2];
}

void PIDController::clearController()
{
    memset(_preErr, 0.0, 3 * sizeof(double));
    memset(_intErr, 0.0, 3 * sizeof(double));
    memset(_dt, 1.0, 3 * sizeof(double));

    memset(_overshoot, 0.0, 3 * sizeof(double));
    memset(_decayratio, 0.0, 3 * sizeof(double));
    memset(_oscillPeriod, 0.0, 3 * sizeof(double));
    memset(_tuningInterval, 0.0, 3 * sizeof(double));
}

double PIDController::cutOff(double q, double max, double min)
{
    if (q > max)
        return max;
    else if (q < min)
        return min;
    else if (abs(q) < 1e-5)
        return 0;
    return q;
}

double PIDController::calDist(double p1[3], double p2[3])
{
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

double PIDController::calThetaAng(double p[3])
{
    double ang = atan2(p[1], p[0]);
    ang = rad2rad(ang);
    return ang;
}

double PIDController::calThetaAng(double x, double y)
{
    double ang = atan2(x, y);
    ang = rad2rad(ang);
    return ang;
}

double PIDController::calAlphaAng(double p1[3], double p2[3], double ang)
{
    double x = p2[0] - p1[0];
    double y = p2[1] - p1[1];
    double new_ang = atan2(y, x) - ang;
    new_ang = rad2rad(new_ang);
    return new_ang;
}

double PIDController::calBetaAng(double goalAng, double poseAng, double theta)
{
    double new_ang = goalAng - poseAng;
    new_ang = rad2rad(new_ang);
    new_ang -= theta;
    return new_ang;
}

double PIDController::rad2rad(double q)
{
    q = q > pi / 2 ? q - pi : (q < -pi / 2 ? q + pi : q);
    return q;
}

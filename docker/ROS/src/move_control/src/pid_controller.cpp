#include <move_control/pid_controller.h>

double calAng2(p p1, p p2)
{
    double new_ang = atan2(p2.second - p1.second, p2.first - p1.first);
    new_ang = new_ang > pi / 2 ? new_ang - pi : (new_ang < -pi / 2 ? new_ang + pi : new_ang);
    return new_ang;
}

PIDController::PIDController(p kp, p ki, p kd, p maxQ, p minQ)
{
    this->_kp = kp;
    this->_ki = ki;
    this->_kd = kd;

    this->_maxQ = maxQ;
    this->_minQ = minQ;
}

p PIDController::calQ(const geometry_msgs::Pose& cur, const geometry_msgs::Pose& goal)
{
    p curPose = { cur.position.x, cur.position.y };
    tf::Quaternion curQuaternion(
        cur.orientation.x,
        cur.orientation.y,
        cur.orientation.z,
        cur.orientation.w);
    double curOri = tf::getYaw(curQuaternion);

    p goalPose = { goal.position.x, goal.position.y };

    double errDist = calDist(curPose, goalPose);
    double errAng = calAng(curPose, goalPose, curOri);
    p err = { errDist, errAng };
    p kp = { _kp.first * errDist, _kp.second * errAng };

    _intErr[0] += _kp.first * errDist * _dt[0];
    _intErr[1] += _kp.second * errDist * _dt[1];
    p ki = { _ki.first * _intErr[0], _ki.second * _intErr[1] };

    p kd = { (err.first - _preErr[0] / _dt[0]), (err.second - _preErr[1] / _dt[1]) };
    kd = { _kd.first * kd.first, _kd.second * kd.second };

    p cmd = { kp.first + ki.first + kd.first, kp.second + ki.second + kd.second };
    cmd.first = cmd.first > 0 ? cmd.first + 0.3 : cmd.first - 0.3;
    cmd.first = cmd.first > _maxQ.first ? _maxQ.first : (cmd.first < _minQ.first ? _minQ.first : cmd.first);
    cmd.second = cmd.second > _maxQ.second ? _maxQ.second : (cmd.second < _minQ.second ? _minQ.second : cmd.second);


    // print pid gain and velocity
    // ROS_INFO(" cur: %f, %f, %f", curPose.first, curPose.second, curOri);
    // ROS_INFO("goal: %f, %f, %f", goalPose.first, goalPose.second, errAng);
    // ROS_INFO(" err: %f, %f", errDist, errAng);
    // ROS_INFO(" lienar kp: %f, ki: %f, kd: %f", kp.first, ki.first, kd.first);
    // ROS_INFO("angular kp: %f, ki: %f, kd: %f", kp.second, ki.second, kd.second);
    // ROS_INFO("cmd_vel\n                                     linear x: %f\n                                    angular z: %f", cmd.first, cmd.second);

    return cmd;
}

void PIDController::updateDeltaTime(double updateTime[])
{
    _dt[0] += updateTime[0];
    _dt[1] += updateTime[1];
}

void PIDController::tuning_gain(p kp, p ki, p kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::clearController()
{
    memset(_preErr, 0.0, 2 * sizeof(double));
    memset(_intErr, 0.0, 2 * sizeof(double));
    memset(_dt, 1.0, 2 * sizeof(double));

    memset(_overshoot, 0.0, 2 * sizeof(double));
    memset(_decayratio, 0.0, 2 * sizeof(double));
    memset(_oscillPeriod, 0.0, 2 * sizeof(double));
    memset(_tuningInterval, 0.0, 2 * sizeof(double));
}

double PIDController::curOff(double q, double max, double min)
{
    if (q > max)
        return max;
    else if (q < min)
        return min;
    else if (abs(q) < 1e-5)
        return 0;
    return q;
}

double PIDController::calDist(p p1, p p2)
{
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

double PIDController::calAng(p p1, p p2, double ang)
{
    int x = round(p2.first - p1.first);
    int y = round(p2.second - p1.second);
    double new_ang = atan2(y, x) - ang;
    new_ang = new_ang > pi / 2 ? new_ang - pi : (new_ang < -pi / 2 ? new_ang + pi : new_ang);
    return new_ang;
}

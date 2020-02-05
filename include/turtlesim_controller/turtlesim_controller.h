#ifndef __TURTLESIM_CONTROLLER_H
#define __TURTLESIM_CONTROLLER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class TurtlesimController
{
public:
    TurtlesimController();
    void process();

private:
    // method
    void pose_callback(const turtlesim::PoseConstPtr&);

    // parameter
    ros::NodeHandle private_nh;
    double hz;

    // member
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_sub;
    turtlesim::Pose current_pose;
};

#endif // __TURTLESIM_CONTROLLER_H

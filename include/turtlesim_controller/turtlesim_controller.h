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
    geometry_msgs::Twist draw_circle();
    geometry_msgs::Twist draw_square();
    geometry_msgs::Twist draw_triangle();

    // parameter
    ros::NodeHandle private_nh;
    double hz;
    double square_length;
    double value_x;
    double value_z;
    double triangle_x1;
    double triangle_y1;
    double triangle_x2;
    double triangle_y2;
    double triangle_x3;
    double triangle_y3;
    std::string mode;

    // member
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_sub;
    turtlesim::Pose current_pose;
};

#endif // __TURTLESIM_CONTROLLER_H

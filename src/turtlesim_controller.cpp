#include "turtlesim_controller/turtlesim_controller.h"

TurtlesimController::TurtlesimController() : private_nh("~")
{
    // subscriber
    pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtlesimController::pose_callback, this);

    // publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    // parameter
    private_nh.param("hz", hz, {10});

    // initialize

    // print prameter
    std::cout << "=== turtlesim_controller ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << std::endl;
}

void TurtlesimController::pose_callback(const turtlesim::PoseConstPtr& msg)
{
    current_pose = *msg;
    return;
}

geometry_msgs::Twist TurtlesimController::draw_circle()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0;
    twist.angular.z = 1.0;
    return twist;
}

void TurtlesimController::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist twist = draw_circle();
        cmd_vel_pub.publish(twist);

        std::cout << "---publish---" << std::endl;
        std::cout << "linear.x: " << twist.linear.x << std::endl;
        std::cout << "angular.z: " << twist.angular.z << std::endl;

        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlesim_controller");
    TurtlesimController turtlesim_controller;
    turtlesim_controller.process();
    return 0;
}

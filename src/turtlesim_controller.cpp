#include "turtlesim_controller/turtlesim_controller.h"

TurtlesimController::TurtlesimController() : private_nh("~")
{
    // parameter
    private_nh.param("hz", hz, {10});
    private_nh.param("square_length", square_length, {5.0});
    private_nh.param("value_x", value_x, {1.0});
    private_nh.param("value_z", value_z, {1.0});

    // initialize

    // subscriber
    pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtlesimController::pose_callback, this);

    // publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    // print prameter
    std::cout << "=== turtlesim_controller ===" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << "square_length: " << square_length << std::endl;
    std::cout << "value_x: " << value_x << std::endl;
    std::cout << "value_z: " << value_z << std::endl;
    std::cout << std::endl;
}

void TurtlesimController::pose_callback(const turtlesim::PoseConstPtr& msg)
{
    current_pose = *msg;
    return;
}

geometry_msgs::Twist TurtlesimController::draw_square()
{
    static bool is_vertex = false;
    static turtlesim::Pose record_pose = current_pose;
    geometry_msgs::Twist twist;
    if(is_vertex)
    {
        twist.linear.x = 0.0;
        twist.angular.z = value_z;
    }
    else
    {
        twist.linear.x = value_x;
        twist.angular.z = 0.0;
    }

    if(current_pose.theta < record_pose.theta)
    {
        current_pose.theta += 2.0 * M_PI;
    }

    if(is_vertex && (current_pose.theta - record_pose.theta) >= M_PI / 2.0)
    {
        std::cout << "linear!!!" << std::endl;
        std::cout << "current theta = " << current_pose.theta << std::endl;
        std::cout << "record theta = " << record_pose.theta << std::endl;
        std::cout << "diff theta = " << current_pose.theta - record_pose.theta << std::endl;
        is_vertex = false;
        record_pose = current_pose;
    }
    else if(!is_vertex && sqrt(pow(current_pose.x - record_pose.x, 2) + pow(current_pose.y - record_pose.y, 2)) >= square_length)
    {
        std::cout << "vertex!!!" << std::endl;
        std::cout << "diff dist = " << sqrt(pow(current_pose.x - record_pose.x, 2) + pow(current_pose.y - record_pose.y, 2)) << std::endl;
        is_vertex = true;
        record_pose = current_pose;
    }
    if(record_pose.theta > 2.0 * M_PI)
    {
        record_pose.theta -= 2.0 * M_PI;
    }

    return twist;
}

geometry_msgs::Twist TurtlesimController::draw_circle()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 1.0;
    return twist;
}

void TurtlesimController::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        geometry_msgs::Twist twist = draw_square();
        cmd_vel_pub.publish(twist);

        // std::cout << "---publish---" << std::endl;
        // std::cout << "linear.x: " << twist.linear.x << std::endl;
        // std::cout << "angular.z: " << twist.angular.z << std::endl;

        ros::spinOnce();
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

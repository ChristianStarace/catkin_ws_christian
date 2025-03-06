// !!! Have also a look at the CMakeLists.txt to better undestand this solution

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace ros;

// Pose is a message that contains the position (Point) and the orientation (Quaternion)
geometry_msgs::Pose p_current;
geometry_msgs::Pose p_end;

// Twist is a message that contains the linear and angular velocities of the robot
geometry_msgs::Twist twist;

bool first_time = true;

Publisher p;

// Tip: Start by reading the main() function first to better understand everything
void run()
{
    ROS_INFO_STREAM("running");
    

    // If the current Position was received for the first time
    // the end position is set
    if (first_time)
    {
        first_time = false;

        p_end = p_current;
        p_end.position.x += 1;
    }

    if (p_current.position.x < p_end.position.x)
    {
        // Set x speed to 0.5
        twist.linear.x = 0.5;
    }
    else
    {
        // Stop the robot
        twist.linear.x = 0;
    }

    // Publish twist message to the cmd_vel topic of the robot
    p.publish(twist);
    ROS_INFO_STREAM("published!");
}

// This function is called when Subscriber s (line 70) read a message
void callback(const nav_msgs::Odometry msg)
{
    // Write the current pose of the robot from the message to p_current
    p_current = msg.pose.pose;

    run();
}

int main(int argc, char **argv)
{
    // See WritingPublisherSubscriber Tutorial on ros.org for explanations of the following lines
    ROS_INFO("started");
    init(argc, argv, "solution_node");

    NodeHandle n;
    Rate loop_rate(5);

    // In the following section the publisher and subscriber is set
    // To find out which one to use, enter "rostopic list" in the command line
    // Use "rostopic type <topic>" to see the data type of a topic.
    p = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
    Subscriber s = n.subscribe("/mobile_base_controller/odom", 1, callback);

    // Important: Don't use spinOnce() because you want a continuous Subscriber
    spin();

    return 0;
}
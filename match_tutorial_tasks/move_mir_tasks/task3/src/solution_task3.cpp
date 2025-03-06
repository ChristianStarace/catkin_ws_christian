#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace ros;

// Position of odometery, AMCL and Ground Truth
geometry_msgs::Pose p_current;
geometry_msgs::Pose poseAMCLx, GTx;

// Orientaion of odometery, AMCL and Ground Truth
geometry_msgs::Quaternion q_current;
geometry_msgs::Quaternion poseAMCLo, GTo;

// variable for publishing twist
geometry_msgs::Twist twist;

// pose and orientation of the end goal
geometry_msgs::Pose p_end;
tf2::Quaternion q_end;

// yaw_end will be the end orientation of the rotation
// yaw_current will be the current orientation of the robot
// junk is for not needed references in the GetEulerYPR() function
double yaw_end, yaw_current, junk;

// Quaternion from tf2 namespace to set new orientation with rad (see "ros tf2" tutorial)
tf2::Quaternion q_tf;


int state = 0;

/**
States:
0: Start up, set the first end position
1: Move to the first end position
2: Turn 180 degrees
3: Set the new end pose with position and orientation
4: Move to end position
5: Turn 180 degrees

default: set the state = 0
*/

Publisher p;

int round_num = 1;

/*
round_num % 3:
1: Run using Odometry poses
2: Run using AMCL poses
0: Run using Ground Truth poses
*/


void move(geometry_msgs::Pose pose){

    // move the robot towards the end goal
    if(abs(p_end.position.x - pose.position.x) > 0.1 ){
        twist.linear.x = 0.3;
    }
    else{
        twist.linear.x = 0;
        state++;
    }
}

void turn(geometry_msgs::Quaternion quat){
    // turn the robot's yaw towards yaw end
    tf2::Quaternion tf2q_current(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
    tf2::Matrix3x3 matrix_current(tf2q_current);
    matrix_current.getRPY(junk, junk, yaw_current);
    if (abs(yaw_end - yaw_current) > 0.005)
    {
        twist.angular.z = 0.5;
    }
    else
    {
        twist.angular.z = 0;
        state++;
    }
}


void run(geometry_msgs::Pose pose, geometry_msgs::Quaternion quat)
{   
    // Debug Message
    ROS_INFO("running state: %i", state);

    switch (state)
    {
    case 0:
        {
            p_end = pose;
            p_end.position.x = 1;


            state++;
        }
        break;
    case 1:    
        {
            move(pose);  
        }
        break;
    case 2:
        {   
            ROS_INFO("end: %f", p_end.position.x);
            ROS_INFO("x: %f", pose.position.x);

            yaw_end = M_PI;
            turn(quat);
        }
        break;

    case 3:
        {
            p_end = pose;
            p_end.position.x = 0;
            ROS_INFO("end: %f", p_end.position.x);
            ROS_INFO("x: %f", pose.position.x);
            state++;
        }
        break;
    case 4:
        {
            move(pose);  
        }
        break;
    case 5:
        {
            yaw_end = 0;
            turn(quat);
        }
        break; 
    default:
        {
            state=0;
            round_num++;
        }
        break;       
    }

    p.publish(twist);
}


void OdomCallback(const nav_msgs::Odometry msg)
{
    //Callback function for Odometry data
    p_current = msg.pose.pose;
    q_current = msg.pose.pose.orientation;
    if(round_num % 3 == 1){
        ROS_INFO("odom running");
        run(p_current,q_current);
    }
}



void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    //Callback function for AMCL data
    poseAMCLx = msgAMCL->pose.pose;
    poseAMCLo = msgAMCL->pose.pose.orientation;   
    if(round_num % 3 == 2){
        ROS_INFO("amcl running");
        run(poseAMCLx,poseAMCLo);
    }
}

void GTCallback(const nav_msgs::Odometry msgGT)
{
    //Callback function for Ground Truth data
    GTx = msgGT.pose.pose;
    GTo = msgGT.pose.pose.orientation;   
    if(round_num % 3 == 0){
        ROS_INFO("ground truth running");
        run(GTx,GTo);
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("started");
    init(argc, argv, "solution_node3");
    NodeHandle n;
    Rate loop_rate(5);

    // Publshing Twist
    p = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000); 
    
    // Subscribing to Odometry data
    Subscriber s = n.subscribe("/mobile_base_controller/odom", 1, OdomCallback); 
    
    // Subscribing to AMCL data
    Subscriber sub_amcl = n.subscribe("/amcl_pose", 1, poseAMCLCallback);

    // Subscribing to Ground Truth data
    Subscriber sub_gt = n.subscribe("/ground_truth", 1, GTCallback);

    /* 
    all user callbacks will be called from within the ros::spin() call. 
    ros::spin() will not return until the node has been shutdown, 
    either through a call to ros::shutdown() or a Ctrl-C. 
    */
    spin();

    return 0;
}
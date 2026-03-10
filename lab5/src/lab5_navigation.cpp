//EGRS 372 - MOBILE ROBOTICS
//Oliver Beckett and Nathan Parrish
//Lab 5 - Lidar Navigation and SLAM

//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "math.h"
#define PI 3.14159265359


//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "lab5_navigation");
  ros::NodeHandle n;

  //Declare publisher
  ros::Publisher pose_info_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

  //Get reference frame from user
  std::cout << "Please enter your info" << std::endl;
  std::cout << "reference frame (map or base_link): ";
  std::string ref_frame;
  std::cin >> ref_frame;

  //Get x distance from user
  std::cout << "x (meters): ";
  double x_distance;
  std::cin >> x_distance;

  //Get y distance from user
  std::cout << "y (meters)";
  double y_distance;
  std::cin >> y_distance;

  //Get rotation around z in degrees from user, convert to quaternions
  std::cout << "rotation around z (degrees): ";
  double psi_offset;
  std::cin >> psi_offset;
  psi_offset = psi_offset * (PI/180);

  double z_quat = sin(psi_offset / 2);
  double w_quat = cos(psi_offset / 2);

  //Declare message variable
  geometry_msgs::PoseStamped pstamp;
  
  //Latch all variables to the publishing node
  pstamp.header.frame_id = ref_frame;
  pstamp.header.stamp = ros::Time::now();
  
  pstamp.pose.position.x = x_distance;
  pstamp.pose.position.y = y_distance;
  pstamp.pose.position.z = 0.0;

  pstamp.pose.orientation.x = 0.0;
  pstamp.pose.orientation.y = 0.0;
  pstamp.pose.orientation.z = z_quat;
  pstamp.pose.orientation.w = w_quat;


  


  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //Publish the info to the robot navigation
  pose_info_pub.publish(pstamp);
  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}



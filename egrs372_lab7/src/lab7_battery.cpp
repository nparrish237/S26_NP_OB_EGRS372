//EGRS 372 - MOBILE ROBOTICS
//Oliver Beckett and Nathan Parrish
//Lab 7 - Battery Management

#include "ros/ros.h"
#include <math.h>
#include "std_msgs/Byte.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Int32.h"

#define PI 3.14159265359

// Positions (map frame)
double home_x = 2.23;
double home_y = 5.02;

double pick_x = 2.23;
double pick_y = 2.8;

double place_x = 2.3;
double place_y = 3.97;

// State variables
int state = 1;
int button_value = 0;
int goal_value = 0;
int battery_low = 0;
int btn_count = 0x0000;
int battery_confirm = 0;

// ================= CALLBACKS =================

void button_reaction(const std_msgs::Byte Bumper)
{ 
  button_value = Bumper.data;
}

void goal_reaction(const move_base_msgs::MoveBaseActionResult goal_result)
{ 
  goal_value = goal_result.status.status;
}

void battery_reaction(const sensor_msgs::BatteryState battery_status)
{
  if (battery_status.voltage < 10.0)
    battery_low = 1;
  else
  {
   battery_confirm = battery_confirm + 1;
   if (battery_confirm >= 5)
   {
    battery_low = 0;
    battery_confirm = 0;
   }
  }
}

// ================= MAIN =================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lab7_battery");
  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
  ros::Publisher seg_pub  = n.advertise<std_msgs::Int32>("LED_data",10);

  ros::Subscriber button_sub  = n.subscribe("bumper",1000,button_reaction);
  ros::Subscriber goal_sub    = n.subscribe("/move_base/result",1000,goal_reaction);
  ros::Subscriber battery_sub = n.subscribe("/battery_state",1000,battery_reaction);

  ros::Rate loop_rate(10);

  // Orientation (0 degrees)
  double psi_offset = 1 * (PI/180);
  double z_quat = -0.7;
  double w_quat = 0.7;

  geometry_msgs::PoseStamped pstamp;
  std_msgs::Int32 seg_data;
  seg_data.data = btn_count;


  // Common pose setup
  pstamp.header.frame_id = "map";
  pstamp.pose.orientation.x = 0.0;
  pstamp.pose.orientation.y = 0.0;
  pstamp.pose.orientation.z = z_quat;
  pstamp.pose.orientation.w = w_quat;

  // ================= GO HOME FIRST =================
  pstamp.pose.position.x = home_x;
  pstamp.pose.position.y = home_y;
  pstamp.header.stamp = ros::Time::now();
  goal_value = 0;

  while(goal_value != 3 && ros::ok())
  {
    seg_pub.publish(seg_data);
    pose_pub.publish(pstamp);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ================= MAIN LOOP =================
  while(ros::ok())
  {
    // -------- BATTERY LOW HANDLING --------
    if(battery_low == 1)
    {
      ROS_INFO("Going HOME");

      pstamp.pose.position.x = home_x;
      pstamp.pose.position.y = home_y;
      pstamp.header.stamp = ros::Time::now();
      goal_value = 0;

      while(goal_value != 3 && ros::ok())
      {
        pose_pub.publish(pstamp);
        ros::spinOnce();
        loop_rate.sleep();
      }



      while(battery_low == 1 && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

    }

    // ----------- PICK LOCATION -----------
    if(state == 1)
    {
      pstamp.pose.position.x = pick_x;
      pstamp.pose.position.y = pick_y;
      pstamp.header.stamp = ros::Time::now();
      goal_value = 0;

      while(goal_value != 3 && ros::ok() && battery_low == 0)
      {
        pose_pub.publish(pstamp);
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(battery_low == 1) continue;

      ROS_INFO("Waiting for button at PICK");

      while(button_value == 0 && ros::ok() && battery_low == 0)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(battery_low == 1) continue;

      button_value = 0;
      state = 2;
    }

    // ----------- PLACE LOCATION -----------
    if(state == 2)
    {
      pstamp.pose.position.x = place_x;
      pstamp.pose.position.y = place_y;
      pstamp.header.stamp = ros::Time::now();
      goal_value = 0;

      while(goal_value != 3 && ros::ok() && battery_low == 0)
      {
        pose_pub.publish(pstamp);
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(battery_low == 1) continue;

      ROS_INFO("Waiting for button at PLACE");

      while(button_value == 0 && ros::ok() && battery_low == 0)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(battery_low == 1) continue;

      button_value = 0;

      btn_count++;
      seg_data.data = btn_count;
      seg_pub.publish(seg_data);

      state = 1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

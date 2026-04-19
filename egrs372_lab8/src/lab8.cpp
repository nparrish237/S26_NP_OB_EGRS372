//EGRS 372 - MOBILE ROBOTICS
//Oliver Beckett and Nathan Parrish
//Lab 8 - Battery Management

#include "ros/ros.h"
#include <math.h>
#include "std_msgs/Byte.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Int32.h"
#include "egrs372_lab8/update_count.h"
#include "egrs372_lab8/go_home.h"
#include "egrs372_lab8/return_to_work.h"
#include "egrs372_lab8/turtlebot_status.h"


#define PI 3.14159265359

double pick[3];
double place[3];
double home[3];
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
int places = 0;
int force_home = 0;
float battery_voltage=0;
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
  battery_voltage = battery_status.voltage;
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

//function for the update count service
//the service requests a new count for the LED
//the service responds with the old LED count
bool update_count(egrs372_lab8::update_count::Request  &req, egrs372_lab8::update_count::Response &res)
{
  //creates a new Node handler to publish the data inside this service
  ros::NodeHandle m;

  //creates the publisher for the LED data
  ros::Publisher LED_pub = m.advertise<std_msgs::Int32>("LED_data", 1);

  //creates the message variable
  std_msgs::Int32 LED_update;

  //sets the response to the old number of places
  res.old_count = places;

  //updates the places with the request
  places = req.new_count;

  //updates the message variable with the new number
  LED_update.data=places;

  //publishes the new LED data
  LED_pub.publish(LED_update);

  return true;
}


//Service to force robot to stop whatever it is doing and go home
bool go_home(egrs372_lab8::go_home::Request &req, egrs372_lab8::go_home::Response &res)
{
	res.old_job = state;	
	force_home = 1;
	return true;
}


//Service to let the robot return to whatever task it was working on
bool return_to_work(egrs372_lab8::return_to_work::Request &req, egrs372_lab8::return_to_work::Response &res)
{
	res.old_job = state;
	force_home = 0;
	return true;
}



// ================= MAIN =================

int main(int argc, char **argv)
{

  ros::init(argc, argv, "lab8_battery");
  ros::NodeHandle n;


  ros::ServiceClient count_client = n.serviceClient<egrs372_lab8::update_count>("update_count");

  //Declare publishers
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,true); //Goal for naviagtion
  ros::Publisher seg_pub  = n.advertise<std_msgs::Int32>("LED_data",10); // Seven segment data
  ros::Publisher status_pub = n.advertise<egrs372_lab8::turtlebot_status>("TurtleBot_Status",10,true); //MR status for service

 

  //Declare subscribers
  ros::Subscriber button_sub  = n.subscribe("bumper",1000,button_reaction); //Front bumper
  ros::Subscriber goal_sub    = n.subscribe("/move_base/result",1000,goal_reaction); //Monitor if the MR has reached its goal
  ros::Subscriber battery_sub = n.subscribe("/battery_state",1000,battery_reaction); // Monitor battery voltage
  ros::ServiceServer count_service = n.advertiseService("update_count", update_count); // Service for updating number of completed cycles
  ros::ServiceServer go_home_service = n.advertiseService("go_home", go_home); // Service to return robot to home position
  ros::ServiceServer return_service = n.advertiseService("return_to_work", return_to_work); // Service to let robot resume task

  ros::Rate loop_rate(10);

  // Orientation
  double psi_offset = 1 * (PI/180);
  double z_quat = 0.0;
  double w_quat = 1.0;

  egrs372_lab8::turtlebot_status status_msg; // MR status messege
  geometry_msgs::PoseStamped pstamp; //MR goal messege
  std_msgs::Int32 seg_data; //7Segment led messege
  seg_data.data = btn_count; //Count for displaying proper number of cycles


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

  goal_value = 0; // Reset goal value so robot knows when it reaches its goal
  ros::Duration(2.0).sleep(); //Wait 2 seconds so the goal properly registers
  pose_pub.publish(pstamp);
  status_msg.current_job = "GOING_HOME FIRST"; //Update MR status service
  status_msg.battery = battery_voltage;


//Move to home position
  while(goal_value != 3 && ros::ok())
  {
    status_pub.publish(status_msg);
    seg_pub.publish(seg_data);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ================= MAIN LOOP =================
  while(ros::ok())
  {
	// Positions (map frame)
	  ros::param::get("/pick_location/x",pick[0]);
	  ros::param::get("/pick_location/y",pick[1]);

	  ros::param::get("/place_location/x",place[0]);
	  ros::param::get("/place_location/y",place[1]);

	  ros::param::get("/home_location/x",home[0]);
	  ros::param::get("/home_location/y",home[1]);
	  // Positions (map frame)
	  home_x = home[0];
	  home_y = home[1];

	  pick_x = pick[0];
	  pick_y = pick[1];

	  place_x = place[0];
	  place_y = place[1];
    // -------- BATTERY LOW HANDLING --------
    if(battery_low == 1 || force_home == 1)
    {
      ROS_INFO("Going HOME");
      status_msg.current_job = "GOING_HOME"; //Update MR status service
status_msg.battery = battery_voltage;

      //Set goal variables to home position
      pstamp.pose.position.x = home_x;
      pstamp.pose.position.y = home_y;
      pstamp.header.stamp = ros::Time::now();

      goal_value = 0; // Reset goal value so robot knows when it reaches its goal
      pose_pub.publish(pstamp);

      //Force the robot to go home while either the battery is low or the go home service was called
      while(goal_value != 3 && ros::ok() && (battery_low == 1 || force_home == 1))
      {
	   status_pub.publish(status_msg);
  
        ros::spinOnce();
        loop_rate.sleep();
      }

      //Once the robot has made it home, hold it there while either the battery is low or the go home service was called
      while((battery_low == 1 || force_home == 1) && ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

    }

    // ----------- PICK LOCATION -----------
    if(state == 1)//Variable to keep track of what task the robot should be working on
    {

      //Update goal variables
      pstamp.pose.position.x = pick_x;
      pstamp.pose.position.y = pick_y;
      pstamp.header.stamp = ros::Time::now();
      goal_value = 0; // Reset goal value so robot knows when it reaches its goal
      status_msg.current_job = "PICK CYCLE";//Update MR status service
status_msg.battery = battery_voltage;
        pose_pub.publish(pstamp);

      //Send the robot to the pick location unless the battery is low or the go home service was called
	while(goal_value != 3 && ros::ok() && battery_low == 0 && force_home == 0)
      {
	   status_pub.publish(status_msg);

        ros::spinOnce();
        loop_rate.sleep();
      }
 
      //If the battery is low or the go home service was called, force the while loop to restart and therefore go home
      if(battery_low == 1 || force_home == 1) continue;

      ROS_INFO("Waiting for button at PICK");

      //Wait for button to be pressed once at the pick location unless the battery is low or the go home service was called
      while(button_value == 0 && ros::ok() && battery_low == 0 && force_home == 0)
      {
	   status_pub.publish(status_msg);
        ros::spinOnce();
        loop_rate.sleep();
      }

      //If the battery is low or the go home service was called, force the while loop to restart and therefore go home
      if(battery_low == 1 || force_home == 1) continue;

      button_value = 0;//Make sure the bumper value is zero to not skip any step
      state = 2; //Set the current job to place
      
    }

    // ----------- PLACE LOCATION -----------
    if(state == 2)

    {
      //Update goal locations to place location
      pstamp.pose.position.x = place_x;
      pstamp.pose.position.y = place_y;
      pstamp.header.stamp = ros::Time::now();
      goal_value = 0;

      status_msg.current_job = "PLACE CYCLE";//Update status service
status_msg.battery = battery_voltage;
        pose_pub.publish(pstamp);

       //Send the robot to the place location unless the battery is low or the go home service was called
	while(goal_value != 3 && ros::ok() && battery_low == 0 && force_home == 0)
      {

   	status_pub.publish(status_msg);
        ros::spinOnce();
        loop_rate.sleep();
      }

     //If the battery is low or the go home service was called, force the while loop to restart and therefore go home
      if(battery_low == 1 || force_home == 1) continue;

      ROS_INFO("Waiting for button at PLACE");


      //Wait for button to be pressed once at the place location unless the battery is low or the go home service was called
      while(button_value == 0 && ros::ok() && battery_low == 0 && force_home == 0)
      {
   	status_pub.publish(status_msg);
        ros::spinOnce();
        loop_rate.sleep();
      }

     //If the battery is low or the go home service was called, force the while loop to restart and therefore go home
      if(battery_low == 1 || force_home == 1) continue;

      button_value = 0;

      btn_count++;//Update the number of cycles completed to display on the led
	status_msg.place_count++;

      seg_data.data = btn_count;
      seg_pub.publish(seg_data);

      state = 1;
    }
 
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

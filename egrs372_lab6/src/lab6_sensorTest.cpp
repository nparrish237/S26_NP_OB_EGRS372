//EGRS 372 - MOBILE ROBOTICS
//Oliver Beckett and Nathan Parrish
//Lab 6

//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include <cassert>
#include <cstddef>
#define PI 3.14159265359
int button_value;
int btn_count = 0x0000;


//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h" // PUBLISHING A GEOETRY_MSGS/...
// 							     // TWIST AS THE SPEED OF THE ROBOT
#include "tf/tfMessage.h"  		 // SUBSCRIBING TO TF/TFMESSAGE ...
// 						   	 	 // AS THE TRANSFORMATION MATRICES OF THE ROBOT
#define INITIALIZE_VALUE -1      // VARIABLES INITIALLY HOLD VALUE OF (-1)
// DEFINING GLOBAL VARIABLES
// ------------------------- 
bool flag; 							  // 'FLAG' = FALSE WHEN CONDITIONS ARE MET
double current_angle=0; 			  // CURRENT ANGLE OF THE ROBOT
double target_speed=0.0; 			  // ROBOT TRAVELLING SPEED (METERS/SECOND)
double target_angle=INITIALIZE_VALUE; // DESIRED ANGLE OF ROBOT PATH (THETA)
double target_forward=0.5;            // DISTANCE THE ROBOT IS TRYING TO MOVE
double moved=0; 					  // DISTANCE TRAVELLED (FORWARD, IN METERS) 
double initialx=INITIALIZE_VALUE;     // INITIAL X COORDINATE OF A MOVE
double initialy=INITIALIZE_VALUE;     // INITIAL Y COORDINATE OF A MOVE




void button_reaction(const std_msgs::Byte Bumper)
{ 
  button_value = Bumper.data;

}

void forwardprog(const tf::tfMessage cvalue)
{
  double dx, dy; // VARIABLES FOR X,Y COORDINATES

  // SETS THE INITIAL X AND Y COORDINATES
  if(initialx==INITIALIZE_VALUE || initialy==INITIALIZE_VALUE)
  {
    initialx=cvalue.transforms[0].transform.translation.x;
    initialy=cvalue.transforms[0].transform.translation.y;
  }

  // CALCULATES DISTANCE IN X AND Y TRAVELED (ONLY CARES ABOUT FORWARD MOVEMENT)
  dx = std::abs(cvalue.transforms[0].transform.translation.x-initialx);
  dy = std::abs(cvalue.transforms[0].transform.translation.y-initialy);
  
  // CALCULATES TOTAL DISTANCE THE ROBOT HAS TRAVELLED
  moved = sqrt(dx*dx+dy*dy);
  
  // CALCULATES DISTANCE (METERS) THE ROBOT HAS YET TO TRAVEL
  if(moved>target_forward)
  {
    flag=false;
  }

  // SETS A SPEED PROPORTIONAL TO THE DISTANCE YET TO BE TRAVELED
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS IN M/S
  target_speed = std::abs(target_forward - moved)/4+0.1;
}
 
void Turnprog(const tf::tfMessage cvalue)
{
  double turnz, turnw, mindist;

  // CALCULATE ORIENTATION OF THE ROBOT
  turnz = cvalue.transforms[0].transform.rotation.z;
  turnw = cvalue.transforms[0].transform.rotation.w;

  // CALCULATE CURRENT ANGLE OF THE ROBOT
  current_angle = 2*atan2(turnz,turnw);

  // CONVERTS THE CURRENT ANGLE TO BE BETWEEN 0 AND 2PI
  if(current_angle < 0)
  {
   current_angle =  current_angle + 2*PI;
  }
  if(current_angle >= 2*PI)
  {
    current_angle =  current_angle - 2*PI;
  }
  
  // SETS THE TARGET ANGLE
  if(target_angle == INITIALIZE_VALUE)
  {
    target_angle =  current_angle + PI/2;
  }
  
  // CONVERTS THE TARGET ANGLE TO BE BETWEEN 0 AND 2PI
  if(target_angle < 0)
  {
    target_angle =  target_angle + 2*PI;
  }
  if(target_angle >= 2*PI)
  {
    target_angle =  target_angle - 2*PI;
  }

  // DETERMINES IF THE ROBOT HAS PASSED THE TARGET ANGLE. 

  // (ONLY WORKS FOR TURNING COUNTER-CLOCKWISE)
  // THE LOGIC AFTER && ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE,
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if((current_angle>=target_angle)&&(std::abs(current_angle-target_angle)<0.1))
  {
    flag = false;
  }

  // FINDS THE MINIMUM ANGLE BETWEEN THE CURRENT AND TARGET ANGLE
  mindist = std::abs(target_angle-current_angle);
  
  // ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE, 
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if(std::abs(target_angle-current_angle+2*PI)<mindist)
  {
    mindist=std::abs(target_angle-current_angle+2*PI);
  }

  // SETS THE TARGET SPEED TO BE PROPORTIONAL TO THE NEEDED ANGLE FOR TRAVEL
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS MEASURED IN RAD/S
  target_speed = mindist/PI+0.5;
}



int main(int argc, char **argv)
{
  
	  //names the program for visual purposes
	  ros::init(argc, argv, "lab6_sensorTest");
	  ros::NodeHandle n;

	  //Declare publisher
	  ros::Publisher pose_info_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	  ros::Publisher seg_pub = n.advertise<std_msgs::Int32>("LED_data",10);

	  ros::Subscriber button_status = n.subscribe("bumper",10,button_reaction);

	  // PUBLISHER DECLARATION FOR THE VELOCITY OF THE ROBOT
	  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	  ros::Subscriber tf; // SUBSCRIBER DECLARATION TO GET THE TRANSFORMATION MATRICES OF THE ROBOT

	  ros::Rate loop_rate(1000);

	  // SETS THE CURRENT TIME TO BE 0
	  ros::Time begin = ros::Time::now();

	  // INITIALIZES THE ROBOT VELOCITY
	  geometry_msgs::Twist c;
	  c.linear.x=0.0;
	  c.linear.y=0.0;
	  c.linear.z=0.0;
	  c.angular.x=0.0;
	  c.angular.y=0.0;
	  c.angular.z=0.0;
	  
	  // CREATES A TIME VARIABLE
	  ros::Time go;

    while(ros::ok())
    {
	//-------------------------------------------Forward function-----------------------------------------------------------
	  // INITIALIZES THE NEEDED VALUES
	    flag = true;
	    moved = 0;
	    initialx=INITIALIZE_VALUE;
	    initialy=INITIALIZE_VALUE;
	    std_msgs::Int32 seg_data;

	    seg_data.data = btn_count;
	    //Publish the info to the robot navigation
	    seg_pub.publish(seg_data);

            //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	    ros::spinOnce();
	    loop_rate.sleep();
	    
	    // GOES FORWARD UNTIL THE NECESSARY DISTANCE IS TRAVELED
	    while(!button_value && ros::ok())
	    {
		  //PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
	      c.linear.x=10;
	      c.angular.z=0;
	      vel_pub.publish(c);

	      //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	      ros::spinOnce();
	      loop_rate.sleep(); 

	    }
	    c.linear.x=0;
	    c.angular.z=0;
	    vel_pub.publish(c);
	    btn_count = btn_count + 0x0001;
	    seg_data.data = btn_count;

	    //Publish the info to the robot navigation
	    seg_pub.publish(seg_data);

	    //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	    ros::spinOnce();
	    loop_rate.sleep(); 

	//----------------------------------------------Backup and turn--------------------------------------------------------
	  // INITIALIZES THE NEEDED VALUES
	    flag = true;
	    moved = 0;
	    initialx=INITIALIZE_VALUE;
	    initialy=INITIALIZE_VALUE;

	    // DECLARE SUBSCRIBER TO THE TF TOPIC (MR POSE) WITH THE FUNCTION FORWARDPROG
	    tf = n.subscribe("/tf", 1, forwardprog);
	    ros::spinOnce();
	    loop_rate.sleep();
	    
	    // GOES FORWARD UNTIL THE NECESSARY DISTANCE IS TRAVELED
	    while(flag && ros::ok())
	    {
		  //PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
	      c.linear.x=-target_speed;
	      c.angular.z=0;
	      vel_pub.publish(c);

	      //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	      ros::spinOnce();
	      loop_rate.sleep(); 

	    }
	    // SHUTS DOWN THE SUBSCRIBER TO CHANGE ITS RELATED FUNCTION LATER
	    tf.shutdown();

	    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
	    go = ros::Time::now();
	    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
	    {
		  //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
	      c.linear.x=0.0;
	      c.angular.z=0;
	      vel_pub.publish(c);
	      
		  //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	      ros::spinOnce();
	      loop_rate.sleep();
	    }

	    // INITIALIZES THE FLAG
	    flag = true;
	    // DECLARE SUBSCRIBER TO THE TF TOPIC (MR POSE) WITH THE FUNCTION TURNPROG
	    tf = n.subscribe("/tf", 1, Turnprog);
	    ros::spinOnce();
	    loop_rate.sleep();

	    // SPINS IN PLACE COUNTER CLOCKWISE UNTIL THE DESIRED ANGLE IS REACHED
	    while(flag && ros::ok())
	    {
		  //PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
	      c.linear.x=0.0;
	      c.angular.z=target_speed;
	      vel_pub.publish(c);
		  
		  //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	      ros::spinOnce();
	      loop_rate.sleep();
	    }
	    
		// SHUTS DOWN THE SUBSCRIBER TO CHANGE ITS RELATED FUNCTION LATER
	    tf.shutdown();
	    
		// INCREMENTS THE TARGET ANGLE
	    target_angle=target_angle+PI/2;

	    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
	    go = ros::Time::now();
	    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
	    {
		  //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
	      c.linear.x=0.0;
	      c.angular.z=0;
	      vel_pub.publish(c);

	      //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
	      ros::spinOnce();
	      loop_rate.sleep();
	    }
    }
    c.linear.x=0.0;
    c.angular.z=0;
    vel_pub.publish(c);
    //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
    ros::spinOnce();
    loop_rate.sleep();
  return 0;
}











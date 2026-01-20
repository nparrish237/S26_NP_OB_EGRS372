//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/String.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_String_Input");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare variables

  std::string outString;
  std_msgs::String string_msg;

  //declare publisher "squarenum" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher string = n.advertise<std_msgs::String>("msg", 100);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //clear the input buffer
    std::fflush; //the two “f's” are correct, not a typo

    //prompt the user for an input
    std::cout << "Enter a string: ";
    //get the input from the user
    std::getline(std::cin, outString);

    string_msg.data = outString;


    std::cout << "Sending the string " << outString << std::endl;

    //set the message value
    string_msg.data=outString;
    //publish the data
    string.publish(string_msg);
    

    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}




//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/String.h"

void string_function(const std_msgs::String string_msg)
{
  //declare the variables
  std::string string_output;
  //set the variable to the squared number
  string_output = string_msg.data.c_str();

  //output the data
  std::cout << "The string " << string_output << " was published" <<std::endl;
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_String_Output");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare subsrcriber "squarenum" is the name of the node
  //1 is how many to save in the buffer
  //uint_function is the function called when a value is recieved
  ros::Subscriber string = n.subscribe("msg", 100, string_function);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    //looks for data
    ros::spin();
  }

  return 0;
}



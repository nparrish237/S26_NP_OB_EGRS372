//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/String.h"


std::string prev_barcode;
std::string barcode_confirmed;
std_msgs::String msg;// Create msg variable to be able to publish barcode

int consecutive = 0;
bool first_run = true; // Added to handle initial state correctly

void barcode_counter(const std_msgs::String::ConstPtr& barval) {
    // 1. Use ConstPtr& for efficiency to avoid copying
    std::string current_barcode = barval->data;

    // 2. Fix logic for the very first message
    if (first_run) {
        prev_barcode = current_barcode;
        consecutive = 1; // Or 0, depending on definition
        first_run = false;
        return;
    }

    // 3. Main Logic
    if (current_barcode == prev_barcode) {
        consecutive++;
	if (consecutive == 5) {
	    barcode_confirmed = current_barcode;
            ROS_INFO("CONFIRMED: %s", barcode_confirmed.c_str()); // Info for ROS log
	    consecutive = 0;
	}
    } else {
        consecutive = 1; // Reset to 1 (current barcode is the first in new sequence)
        prev_barcode = current_barcode;
    }

    // Use ROS_INFO for logging instead of std::cout for proper formatting/levels
    ROS_INFO("Barcode: %s, Consecutive: %d", current_barcode.c_str(), consecutive);
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "barcode_reader");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);


  ros::Subscriber barcode = n.subscribe("barcode", 10, barcode_counter);
  ros::Publisher confirmed_pub = n.advertise<std_msgs::String>("confirmed_pub",10);

  



  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    if (consecutive == 0)
    {

    msg.data = barcode_confirmed;

    }
    else
    {

 	msg.data = "0";
    }
    confirmed_pub.publish(msg);
    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}


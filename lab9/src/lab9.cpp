#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

#include <cmath>
#include <sstream>
#include <fstream>
#include <string>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <ros/package.h>
#include "lab9/parameter_update.h"

#define PI 3.141592653589793

//--------------------------------GLOBAL VARIABLES--------------------------------

// Waypoints
double p_x[] = {2.23, 2.23, 2.23, 2.23, 2.23};
double p_y[] = {5.02, 4.0, 3.0, 2.0, 1.0};

// Crosswalk bounds (EDIT THESE)
double X1 = 1.41, X2 = 2.7;
double Y1 = 1.92, Y2 = 3.06;

// Human + robot positions
double human_pos_x = 0, human_pos_y = 0;
double current_x = 0, current_y = 0;

// State flags
int goal_value = 0;
int in_human = 0;
int in_slow = 0;

// Speeds
double slow_speed = 0.13;

// ROS objects
ros::Publisher motor_pub;

// Dynamic reconfigure
dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::Config conf;
dynamic_reconfigure::DoubleParameter double_param;

//----------------------------------CALLBACKS-------------------------------

void goal_cb(const move_base_msgs::MoveBaseActionResult& msg)
{
    goal_value = msg.status.status;
}

void pose_cb(const move_base_msgs::MoveBaseActionFeedback& msg)
{
    current_x = msg.feedback.base_position.pose.position.x;
    current_y = msg.feedback.base_position.pose.position.y;

    // Crosswalk check
    if (current_x >= X1 && current_x <= X2 &&
        current_y >= Y1 && current_y <= Y2)
        in_slow = 1;
    else
        in_slow = 0;
}

void human_cb(const geometry_msgs::Point& msg)
{
    human_pos_x = msg.x;
    human_pos_y = msg.y;

    double dx = current_x - human_pos_x;
    double dy = current_y - human_pos_y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist <= 0.5)
        in_human = 1;
    else
        in_human = 0;
}

//--------------------------------MAIN--------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab9");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Publishers
    ros::Publisher goal_pub =
        n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    motor_pub =
        n.advertise<std_msgs::Bool>("/motor_power",1);
    ros::Publisher param_pub =
        n.advertise<lab9::parameter_update>("parameter_status",1);

    // Subscribers
    ros::Subscriber sub_goal =
        n.subscribe("/move_base/result",10,goal_cb);
    ros::Subscriber sub_pose =
        n.subscribe("/move_base/feedback",10,pose_cb);
    ros::Subscriber sub_human =
        n.subscribe("human",10,human_cb);

    ros::service::waitForService("/move_base/DWAPlannerROS/set_parameters");

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1.0;

    lab9::parameter_update status_msg;

    for (int i = 0; i < 5; i++)
    {
        //--------------------------------READ PARAMETERS FILE--------------------------------
        std::string path = ros::package::getPath("lab9") + "/parameters.txt";
        std::ifstream file(path.c_str());

        double var1 = 0.22;  // max linear
        double var3 = 2.84;  // max angular
        char var2f = 'f';
        char var4f = 'f';

        if (file.is_open())
        {
            std::string line;
            for (int j = 0; j <= i; j++)
                std::getline(file, line);

            std::stringstream ss(line);
            std::string token;

            std::getline(ss, token, ',');
            var1 = atof(token.c_str());

            std::getline(ss, token, ',');
            if (!token.empty()) var2f = token[0];

            std::getline(ss, token, ',');
            var3 = atof(token.c_str());

            std::getline(ss, token, ',');
            if (!token.empty()) var4f = token[0];

            file.close();
        }

        double var2d = (var2f == 't') ? -0.5 : 0.0;
        double var4d = (var4f == 't') ? 0.1 : 10.0;

        // Publish parameter status
        status_msg.max_lin_spd = var1;
        status_msg.max_rot_spd = var3;
        status_msg.backward_move = var2f;
        status_msg.adjust_orient = var4f;
        param_pub.publish(status_msg);
        ROS_INFO("var1 = %f", var1);
        ROS_INFO("var2 = %f", var2d);
        ROS_INFO("var3 = %f", var3);
        ROS_INFO("var4 = %f", var4d);

        // Apply base parameters
        conf.doubles.clear();

        double_param.name = "max_vel_x";
        double_param.value = var1;
        conf.doubles.push_back(double_param);

        double_param.name = "min_vel_x";
        double_param.value = var2d;
        conf.doubles.push_back(double_param);

        double_param.name = "max_rot_vel";
        double_param.value = var3;
        conf.doubles.push_back(double_param);

        double_param.name = "yaw_goal_tolerance";
        double_param.value = var4d;
        conf.doubles.push_back(double_param);

        srv_req.config = conf;
        ros::service::call("/move_base/DWAPlannerROS/set_parameters",
                           srv_req, srv_resp);

        conf.doubles.clear();

        //--------------------------------SEND GOAL--------------------------------
        goal.pose.position.x = p_x[i];
        goal.pose.position.y = p_y[i];
        goal.header.stamp = ros::Time::now();

        goal_value = 0;
        ros::Duration(1.0).sleep();
        goal_pub.publish(goal);

        //--------------------------------NAVIGATION LOOP--------------------------------
        while (goal_value != 3 && ros::ok())
        {
            ros::spinOnce();

            std_msgs::Bool motor_msg;

            //-------------PRIORITY: HUMAN > CROSSWALK > NORMAL--------------------------
            if (in_human)
            {
                motor_msg.data = false;  // STOP
                motor_pub.publish(motor_msg);
            }
            else
            {
                motor_msg.data = true;
                motor_pub.publish(motor_msg);

                // Apply crosswalk speed limit
                double final_speed = var1;

                if (in_slow)
                    final_speed = std::min(var1, slow_speed);

                // Update max_vel_x 
                double_param.name = "max_vel_x";
                double_param.value = final_speed;
                conf.doubles.push_back(double_param);

                srv_req.config = conf;
                ros::service::call("/move_base/DWAPlannerROS/set_parameters",
                                   srv_req, srv_resp);

                conf.doubles.clear();
            }

            loop_rate.sleep();
        }
    }

    return 0;
}

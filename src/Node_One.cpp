//Node_one
/*============================================*/
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Kill.h"
#include <turtlesim/Spawn.h>
#include "string.h"
#include <chrono>
/*============================================*/
//define error stated
enum killing_state{
    killed,
    notkilled
};

enum spawn_state{
    spawn_Done,
    spawn_failed
};
/*============================================*/
//instances of error states
enum killing_state turtle_killer(std::string turtle_name, ros::NodeHandle& NodeHandler_t);
enum spawn_state turtle_spawn(float x, float y, float theta, std::string turtle_name, ros::NodeHandle& NodeHandler_t);
/*============================================*/
//controller function
void controlTurtle(const std::string& turtle1_name, const std::string& turtle2_name, int duration, ros::NodeHandle& NodeHandler_t);
/*============================================*/


int main (int argc, char **argv){
/*============================================*/
//initilizataion
    ros::init(argc, argv, "UI_one");
    ros::NodeHandle NodeHandler_t;
    turtle_killer("turtle1",NodeHandler_t);
    turtle_spawn(5.5,2,0,"turtleOne",NodeHandler_t);
    turtle_spawn(2,2,0,"turtleTwo",NodeHandler_t);
    ros::Rate loop_rate(10);
    //loop rate to do the corol periodically
    while (ros::ok()) {
        controlTurtle("turtleOne", "turtleTwo", 1, NodeHandler_t);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
//function to kill the turtle
enum killing_state turtle_killer(std::string turtle_name, ros::NodeHandle& NodeHandler_t)
{
    ros::ServiceClient turtle_killer_client = NodeHandler_t.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill node_killer_request;
    node_killer_request.request.name=turtle_name;
    if (turtle_killer_client.call(node_killer_request)){
        return killed;
    }
    else{
        ROS_ERROR("Failed to call /kill service.");
        return notkilled;
    }
}
//funtion to spawn new one
enum spawn_state turtle_spawn(float x, float y, float theta, std::string turtle_name, ros::NodeHandle& NodeHandler_t){
    ros::ServiceClient turtle_spawn_client = NodeHandler_t.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_turtle_t;
    spawn_turtle_t.request.x=x;
    spawn_turtle_t.request.y=y;
    spawn_turtle_t.request.theta=theta;
    spawn_turtle_t.request.name=turtle_name;
    if (turtle_spawn_client.call(spawn_turtle_t)){
        return spawn_Done;
    }
    else {
        ROS_ERROR("Failed to call /spawn service.");
        return spawn_failed;
    }
}
//function to do the control
void controlTurtle(const std::string& turtle1_name, const std::string& turtle2_name, int duration, ros::NodeHandle& NodeHandler_t){
    ros::Publisher pub_turtle1 = NodeHandler_t.advertise<geometry_msgs::Twist>("/" + turtle1_name + "/cmd_vel", 10);
    ros::Publisher pub_turtle2 = NodeHandler_t.advertise<geometry_msgs::Twist>("/" + turtle2_name + "/cmd_vel", 10);
    geometry_msgs::Twist velocity;

    while (ros::ok()) {
        std::string selected_turtle;
        double linear_velocity, angular_velocity;

        std::cout << "Which turtle do you want to control? (" << turtle1_name << "/" << turtle2_name << "): or q to exit ";
        std::cin >> selected_turtle;
        
        if (selected_turtle == "q") {
            std::cout << "Exiting program. Goodbye!" << std::endl;
            break;
        }

        if (selected_turtle != turtle1_name && selected_turtle != turtle2_name) {
            std::cout << "Invalid turtle name. Please try again." << std::endl;
            continue;
        }

        std::cout << "Enter linear velocity: ";
        std::cin >> linear_velocity;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular_velocity;

        velocity.linear.x = linear_velocity;
        velocity.angular.z = angular_velocity;

        ros::Publisher selected_pub = (selected_turtle == turtle1_name) ? pub_turtle1 : pub_turtle2;

        std::cout << "Sending command to " << selected_turtle << " for " << duration << " second(s)..." << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < duration) {
            selected_pub.publish(velocity);
            ros::Duration(0.1).sleep();
        }

        velocity.linear.x = 0;
        velocity.angular.z = 0;
        selected_pub.publish(velocity);

        std::cout << selected_turtle << " has stopped. You can insert a new command." << std::endl;
    }
}

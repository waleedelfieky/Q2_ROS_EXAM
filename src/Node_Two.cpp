//node_two
/*=======================================*/
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/TeleportAbsolute.h"
/*=======================================*/
struct TurtleData {
    turtlesim::Pose pose;           
    geometry_msgs::Twist velocity;  
    bool isMoving;                  
};
/*=======================================*/
TurtleData turtles[2]{};
/*=======================================*/
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void teleportTurtle(const std::string& turtle_name, ros::NodeHandle& nodeHandler, float x, float y, float theta);
void turtle_distance_checker(turtlesim::Pose turtle1_pose,turtlesim::Pose turtle2_pose, float threshold, ros::NodeHandle& nodeHandler);
/*=======================================*/



int main(int argc, char** argv) {
/*=======================================*/
    ros::init(argc, argv, "Node_two");
    ros::NodeHandle Node_Two_handler;
    ros::Subscriber turtle1_sub = Node_Two_handler.subscribe("/turtleOne/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = Node_Two_handler.subscribe("/turtleTwo/pose", 10, turtle2PoseCallback);
    ros::Rate loop_rate(1000000);
/*=======================================*/
    while (ros::ok()) {
        turtle_distance_checker(turtles[0].pose, turtles[1].pose, 1.5, Node_Two_handler);
        ros::spinOnce();
        loop_rate.sleep();
    }
/*=======================================*/
    return 0;
}
/*=======================================*/
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtles[0].pose = *msg;
    turtles[0].isMoving = (msg->linear_velocity != 0 || msg->angular_velocity != 0);
}
/*=======================================*/
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtles[1].pose = *msg;
    turtles[1].isMoving = (msg->linear_velocity != 0 || msg->angular_velocity != 0);
}
/*=======================================*/
void turtle_distance_checker(turtlesim::Pose turtle1_pose, turtlesim::Pose turtle2_pose, float threshold, ros::NodeHandle& nodeHandler) {
    float dx = turtle2_pose.x - turtle1_pose.x;
    float dy = turtle2_pose.y - turtle1_pose.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= threshold) {
        if (turtles[0].isMoving) {
            teleportTurtle("turtleOne", nodeHandler, 1.0, 1.0, 0.0);
        } else if (turtles[1].isMoving) {
            teleportTurtle("turtleTwo", nodeHandler, 9.0, 9.0, 0.0);
        }
    }

    if (turtle1_pose.x > 10 || turtle1_pose.y > 10 || turtle1_pose.x < 1 || turtle1_pose.y < 1) {
        teleportTurtle("turtleOne", nodeHandler, 5.5, 5.5, 0.0);
    }

    if (turtle2_pose.x > 10 || turtle2_pose.y > 10 || turtle2_pose.x < 1 || turtle2_pose.y < 1) {
        teleportTurtle("turtleTwo", nodeHandler, 5.5, 5.5, 0.0);
    }
}
/*=======================================*/
void teleportTurtle(const std::string& turtle_name, ros::NodeHandle& nodeHandler, float x, float y, float theta) {
    ros::ServiceClient teleport_client = nodeHandler.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");
    turtlesim::TeleportAbsolute teleport_request;
    teleport_request.request.x = x;
    teleport_request.request.y = y;
    teleport_request.request.theta = theta;
    teleport_client.call(teleport_request);
}

// inter_util.h
#ifndef INTER_UTIL_H
#define INTER_UTIL_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pedsim_msgs/AgentStates.h>

namespace inter_util
{
    struct SimAgentInfo {
        geometry_msgs::Point32 point; 
        std::string social_state;             
        std::string type;             
        std::string id; 
    };


    class InterUtil
    {
    public:
        static std::string getLocalPlanner(const std::string &keyword);
        static double getDangerLevel(const std::vector<double>& terms);
        static void publishSignal(ros::Publisher& publisher);
        static void checkDanger(ros::Publisher& publisher, const std::vector<double>& terms, double threshold);
        static void processAgentStates(const pedsim_msgs::AgentStates::ConstPtr& message, std::vector<SimAgentInfo>& simAgentInfos);
        static bool setSpeed(bool caution, double minDistance, double changed_max_vel_x_param_, double max_vel_x_param_, std::string planner);
        static geometry_msgs::PoseStamped setTempGoal(const geometry_msgs::PoseStamped &start, double theta, double distance, double temp_goal_distance, std::string planner);
        static bool isPedestrian(double detectedRange, double distance);
        static bool checkObstacles(std::vector<double> robotPositionVector, std::vector<SimAgentInfo>& simAgentInfos, double theta, double padding, double temp_goal_distance, double robot_radius, std::vector<double> detectedRanges, std::vector<double> detectedAngles, bool checkPeds, bool checkBehind);
    };
}

#endif // INTER_UTIL_H

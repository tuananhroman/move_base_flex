// inter_util.h
#ifndef INTER_UTIL_H
#define INTER_UTIL_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>

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
    };
}

#endif // INTER_UTIL_H

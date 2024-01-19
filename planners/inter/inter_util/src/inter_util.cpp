// inter_util.cpp
#include "inter_util.h"
#include <vector>
#include <cmath>

namespace inter_util
{
    std::string InterUtil::getLocalPlanner(const std::string &keyword)
    {
        std::string local_planner_name;

        if (keyword == "teb")
        {
            local_planner_name = "TebLocalPlannerROS";
        }
        else if (keyword == "mpc")
        {
            local_planner_name = "MpcLocalPlannerROS";
        }
        else if (keyword == "dwa")
        {
            local_planner_name = "DwaLocalPlannerROS";
        }
        else if (keyword == "cohan")
        {
            local_planner_name = "HATebLocalPlannerROS";
        }

        return local_planner_name;
    }

    double InterUtil::getDangerLevel(const std::vector<double>& terms) {
        double denominator = 0.0;

        for (double term : terms) {
            denominator += 1/pow((term / 3.0), 2);
        }

        double exponent = -1.0 / denominator;

        return exp(exponent);
        }   
    
    void InterUtil::publishSignal(ros::Publisher& publisher)
    {
        std_msgs::String msg;
        msg.data = "SIGNAL";
        publisher.publish(msg);
    }

    void InterUtil::checkDanger(ros::Publisher& publisher, const std::vector<double>& terms, double threshold)
    {
        double dangerLevel = getDangerLevel(terms);

        if (dangerLevel > threshold) {
            publishSignal(publisher);
        }
    }
}
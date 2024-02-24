// inter_util.cpp
#include "inter_util.h"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string> 
#include <pedsim_msgs/AgentStates.h>

namespace inter_util
{
    std::string InterUtil::getLocalPlanner(const std::string &keyword)
    {
        // Create a map to store keyword to local planner name mappings
        std::unordered_map<std::string, std::string> plannerMap = {
            {"teb", "TebLocalPlannerROS"},
            {"mpc", "MpcLocalPlannerROS"},
            {"dwa", "DwaLocalPlannerROS"},
            {"cohan", "HATebLocalPlannerROS"},
            {"rosnav", "TebLocalPlannerROS"},
            {"dragon", "DwaLocalPlannerROS"},
            {"applr", "TrajectoryPlannerROS"},
            {"lflh", "TrajectoryPlannerROS"},
            {"trail", "TrajectoryPlannerROS"}
        };

        // Use the map to find the local planner name
        auto it = plannerMap.find(keyword);
        return (it != plannerMap.end()) ? it->second : "";
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

    void InterUtil::processAgentStates(const pedsim_msgs::AgentStates::ConstPtr& message, std::vector<SimAgentInfo>& simAgentInfos) {
        simAgentInfos.clear();
        for (const auto& agent_state : message->agent_states) {
            SimAgentInfo simAgentInfo;
            geometry_msgs::Point32 pedestrianPoint;
            pedestrianPoint.x = agent_state.pose.position.x;
            pedestrianPoint.y = agent_state.pose.position.y;
            pedestrianPoint.z = agent_state.pose.position.z;
            simAgentInfo.point = pedestrianPoint;
            simAgentInfo.social_state = agent_state.social_state;
            simAgentInfo.type = agent_state.type;
            simAgentInfo.id = agent_state.id;
            simAgentInfos.push_back(simAgentInfo);
        }
    }

}
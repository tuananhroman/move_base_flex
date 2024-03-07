// inter_util.cpp
#include "inter_util.h"

#include <ros/ros.h>
#include <ros/master.h>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <pedsim_msgs/AgentStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

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
            {"trail", "TrajectoryPlannerROS"}};

        // Use the map to find the local planner name
        auto it = plannerMap.find(keyword);
        return (it != plannerMap.end()) ? it->second : "";
    }

    double InterUtil::getDangerLevel(const std::vector<double> &terms)
    {
        double denominator = 0.0;

        for (double term : terms)
        {
            denominator += 1 / pow((term / 3.0), 2);
        }

        double exponent = -1.0 / denominator;

        return exp(exponent);
    }

    void InterUtil::publishSignal(ros::Publisher &publisher)
    {
        std_msgs::String msg;
        msg.data = "SIGNAL";
        publisher.publish(msg);
    }

    void InterUtil::checkDanger(ros::Publisher &publisher, const std::vector<double> &terms, double threshold)
    {
        double dangerLevel = getDangerLevel(terms);

        if (dangerLevel > threshold)
        {
            publishSignal(publisher);
        }
    }

    void InterUtil::processAgentStates(const pedsim_msgs::AgentStates::ConstPtr &message, std::vector<SimAgentInfo> &simAgentInfos)
    {
        simAgentInfos.clear();
        for (const auto &agent_state : message->agent_states)
        {
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

    bool InterUtil::setSpeed(bool caution, double minDistance, double changed_max_vel_x_param_, double max_vel_x_param_, std::string planner)
    {
        double speed = 0;
        if (planner != "aggressive")
        {
            speed = caution ? changed_max_vel_x_param_ : max_vel_x_param_;
        }
        else
        {
            speed = max_vel_x_param_ - (max_vel_x_param_ / (1 + std::pow(minDistance, 2)));
        }
        return speed;
    }

    geometry_msgs::PoseStamped InterUtil::setTempGoal(const geometry_msgs::PoseStamped &start, double theta, double distance, double temp_goal_distance, std::string planner)
    {
        ROS_INFO("Pedestrian detected. Distance: %lf", distance);
        ROS_INFO("Setting new temp_goal");
        geometry_msgs::PoseStamped temp_goal = start;

        if (planner == "polite")
        {
            // calculating position for temporary goal
            temp_goal.pose.position.x -= temp_goal_distance * cos(theta);
            temp_goal.pose.position.y -= temp_goal_distance * sin(theta);
            temp_goal.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(temp_goal.pose.orientation));
        }
        if (planner == "sideways")
        {
            temp_goal.pose.position.x -= 1.5 * temp_goal_distance * cos(theta + M_PI / 4.0);
            temp_goal.pose.position.y -= 1.5 * temp_goal_distance * sin(theta + M_PI / 4.0);
            temp_goal.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        }
        temp_goal.header.frame_id = start.header.frame_id;
        return temp_goal;
    }

    bool InterUtil::isPedestrian(double detectedRange, double distance)
    {
        return (detectedRange - 1.0) <= distance && distance <= (detectedRange + 1.0);
    }

    bool InterUtil::checkObstacles(std::vector<double> robotPositionVector, std::vector<SimAgentInfo> &simAgentInfos, double theta, double padding, double temp_goal_distance, double robot_radius, std::vector<double> detectedRanges, std::vector<double> detectedAngles, bool checkPeds, bool checkBehind)
    {
        for (size_t i = 0; i < detectedRanges.size(); ++i)
        {
            // calculate the shortest angle to determine if obstacle is behind us
            double relative_angle = angles::shortest_angular_distance(theta, detectedAngles[i]);
            // case if we want to check static and dynamic (pedestrian) obstacles
            if (checkPeds)
            {
                // returns true if we detect any obstacle behind us that is nearer than our temp goal and could block us
                if (checkBehind && (2 * std::abs(relative_angle) <= M_PI) && (detectedRanges[i] <= temp_goal_distance) && (detectedRanges[i] <= 2 * robot_radius + padding))
                {
                    return true;
                }
                // returns true if we detect any obstacle in our defined padding range
                else if (!checkBehind && detectedRanges[i] <= padding)
                {
                    return true;
                }
            }
            // case if we want to ignore dynamic (pedestrian) obstacles
            if (!checkPeds)
            {
                // check if scan could pe a pedestrian
                bool isPed = false;
                for (const auto &simAgentInfo : simAgentInfos)
                {
                    geometry_msgs::Point32 point = simAgentInfo.point;
                    double distance = std::sqrt(std::pow(point.x - robotPositionVector[0], 2) + std::pow(point.y - robotPositionVector[1], 2)) + std::pow(point.z - robotPositionVector[2], 2);
                    if (isPedestrian(detectedRanges[i], distance))
                    {
                        isPed = true;
                    }
                }
                // returns true if we detect any static obstacle behind us that is nearer than our temp goal and could block us
                if (checkBehind && !isPed && (2 * std::abs(relative_angle) <= M_PI) && (detectedRanges[i] <= temp_goal_distance) && (detectedRanges[i] <= 2 * robot_radius + padding))
                {
                    return true;
                }
                // returns true if we detect any static obstacle in our defined padding range
                else if (!checkBehind && !isPed && detectedRanges[i] <= padding)
                {
                    ROS_ERROR("%f and here the padding %f", detectedRanges[i], padding);
                    return true;
                }
            }
        }
        return false;
    }
}
#include "../include/aggressive_inter.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
PLUGINLIB_EXPORT_CLASS(aggressive_inter::AggressiveInter, mbf_costmap_core::CostmapInter)

namespace aggressive_inter
{
    std::vector<geometry_msgs::Point32> semanticPoints;
    ros::Subscriber subscriber_;
    ros::ServiceClient setParametersClient_;
    double max_vel_x_param_;
    uint32_t AggressiveInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                       std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;
        double minDistance = 999999; // sufficiently high number
        ros::Subscriber subscriber_;    
        for (const auto &point : semanticPoints)
        {
            double distance = std::sqrt(std::pow(point.x - robot_x, 2) + std::pow(point.y - robot_y, 2)) + std::pow(point.z - robot_z, 2);
            minDistance = std::min(distance, minDistance);

            // Check if the closest pedestrian is in range to slow down
            if (minDistance <= slowdown_distance)
            {
                //speed converges to max_speed_ at around 5 -> adapt function if necessary
                double speed = max_speed_ - (max_speed_ / (1 + std::pow(distance, 2)));
                setMaxVelocity(speed);
            }
            else
            {
                setMaxVelocity(max_speed_);
            }
        }
        plan = plan_;
        return 0;
    }

    bool AggressiveInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        plan_ = plan;
        return true;
    }

    void AggressiveInter::semanticCallback(const pedsim_msgs::SemanticData::ConstPtr &message)
    {
        semanticPoints.clear();
        for (const auto &point : message->points)
        {
            geometry_msgs::Point32 pedestrianPoint;
            pedestrianPoint.x = point.location.x;
            pedestrianPoint.y = point.location.y;
            pedestrianPoint.z = point.location.z;
            semanticPoints.push_back(pedestrianPoint);
        }
    }

    std::string AggressiveInter::get_local_planner(){

        std::string keyword;
        std::string local_planner_name;

        if (!nh_.getParam(node_namespace_+"/local_planner", keyword))
        {
            ROS_ERROR("Failed to get parameter %s/local_planner", node_namespace_.c_str());

        }
        if(keyword=="teb"){
            local_planner_name= "TebLocalPlannerROS";
        }
        if(keyword=="mpc"){
            local_planner_name= "MpcLocalPlannerROS";
        }
        if(keyword=="dwa"){
            local_planner_name= "DwaLocalPlannerROS";
        }
        if(keyword=="cohan"){
            local_planner_name= "HAtebLocalPlannerROS";
        }        

        return local_planner_name;
    }

    void AggressiveInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        std::string local_planner_name = get_local_planner();
        this->name = name;
        std::string node_namespace_ = ros::this_node::getNamespace();
        nh_ = ros::NodeHandle("~");
                std::string semanticLayer = "/pedsim_agents/semantic/pedestrian";
        // get the starting parameter for max_vel_x from our planner
        subscriber_ = nh_.subscribe(semanticLayer, 1, &AggressiveInter::semanticCallback, this);
        if (!nh_.getParam(node_namespace_+"/move_base_flex/"+ local_planner_name+"/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/TebLocalPlannerROS/max_vel_x", node_namespace_.c_str());
            return;
        }
        // Create service clients for the GetDump and Reconfigure services
        setParametersClient_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(node_namespace_+"/move_base_flex/"+ local_planner_name+"/set_parameters");
        dynamic_reconfigure::Server<aggressive_inter::AggressiveInterConfig> server;
        server.setCallback(boost::bind(&AggressiveInter::reconfigure, this, _1, _2));
    }

    void AggressiveInter::setMaxVelocity(double new_max_vel_x)
    {
        ros::ServiceClient client = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/jackal/move_base_flex/TebLocalPlannerROS/set_parameters");
        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double_param.name = "max_vel_x";
        double_param.value = new_max_vel_x;
        conf.doubles.push_back(double_param);
        srv.request.config = conf;

        if (client.call(srv))
        {
            ROS_INFO_ONCE("Dynamic reconfigure request successful");
        }
        else
        {
            ROS_ERROR_ONCE("Failed to call dynamic reconfigure service");
        }
    }

    void AggressiveInter::reconfigure(aggressive_inter::AggressiveInterConfig &config, uint32_t level)
    {
        slowdown_distance = config.slowdown_distance;
        max_speed_ = config.max_speed;
    }
}
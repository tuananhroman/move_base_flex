#include "../include/polite_inter.h"
#include <std_msgs/Int32.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Point32.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{

    uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;

        bool caution = false;

        for (const auto &point : semanticPoints)
        {
            double distance = std::sqrt(std::pow(point.x - robot_x, 2) + std::pow(point.y - robot_y, 2)) + std::pow(point.z - robot_z, 2);
            //calculates if ped is behind the robot to determine if he can continue to drive or set temp_goal
            double angle_to_point = atan2(point.x-robot_x, point.y-robot_y);
            double theta = tf::getYaw(start.pose.orientation);
            double angle_diff = angles::shortest_angular_distance(theta, angle_to_point);
            
            // check speed restriction
            caution |= (distance <= caution_detection_range_);
            
            // Check if the pedestrian is in range to set temporary goal and move back
            if ((!new_goal_set_) && (distance <= ped_minimum_distance_) && (2 * std::abs(angle_diff) <= fov_))
            {
                ROS_INFO("Pedestrian detected. Distance: %lf", distance);
                ROS_INFO("Setting new temp_goal");
                temp_goal_ = start;

                // calculating position for temporary goal
                temp_goal_.pose.position.x -= temp_goal_distance_ * cos(theta);
                temp_goal_.pose.position.y -= temp_goal_distance_ * sin(theta);
                temp_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(temp_goal_.pose.orientation));
                temp_goal_.header.frame_id = start.header.frame_id;
                new_goal_set_ = true;
            }
            
            // nothing else to compute
            if(caution && new_goal_set_)
                break;
        }

        setSpeed(caution ? changed_max_vel_x_param_ : max_vel_x_param_);

        if (new_goal_set_)
        {
            //calculate distance to temporary goal
            double distance_to_temp_goal = std::sqrt(std::pow(temp_goal_.pose.position.x - robot_x, 2) + std::pow(temp_goal_.pose.position.y - robot_y, 2));

            // Clear the existing plan and add temp_goal
            plan.clear();
            plan.push_back(temp_goal_);

            if (distance_to_temp_goal <= temp_goal_tolerance_)
            {
                ROS_INFO("Reached temp_goal. Resetting goal.");
                new_goal_set_ = false;
            }
        }
        else
            plan = plan_;

        return 0;
    }

    bool PoliteInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        plan_ = plan;
        return true;
    }

    std::string PoliteInter::getLocalPlanner(){

        std::string keyword;
        std::string local_planner_name;

        if (!nh_.getParam(node_namespace_+"/local_planner", keyword)){
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
            local_planner_name= "HATebLocalPlannerROS";
        }        

        return local_planner_name;
    }

    void PoliteInter::semanticCallback(const pedsim_msgs::SemanticData::ConstPtr &message)
    //turns our semantic layer data into points we can use to calculate distance
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

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

    void PoliteInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string local_planner_name = getLocalPlanner();
        std::string node_namespace_ = ros::this_node::getNamespace();
        std::string semantic_layer = "/pedsim_agents/semantic/pedestrian";
        nh_ = ros::NodeHandle("~");
        subscriber_ = nh_.subscribe(semantic_layer, 1, &PoliteInter::semanticCallback, this);
        // get the starting parameter for max_vel_x from our planner
        if (!nh_.getParam(node_namespace_+"/move_base_flex/"+ local_planner_name +"/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/TebLocalPlannerROS/max_vel_x", node_namespace_.c_str());
            return;
        }
        // Create service client for the Reconfigure service
        setParametersClient_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(node_namespace_+"/move_base_flex/"+ local_planner_name+"/set_parameters");
        dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
        server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));
        // needs to be declared here because cautious_speed gets declared with reconfigure
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);
    }

    void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        //updating values from config
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        caution_detection_range_ = config.caution_detection_range;
        cautious_speed_ = config.cautious_speed;
        temp_goal_tolerance_ = config.temp_goal_tolerance;

        fov_ = M_PI; //TODO get from dynamic reconf
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);
    }

    void PoliteInter::setSpeed(double speed)
    {
        //TODO make ROS thread safe
        return;
        
        boost::unique_lock<boost::mutex> lock(speed_mtx_);

        if(speed_ != speed){
            speed_ = speed;

            // set max_vel_x parameter
            double_param_.name = "max_vel_x";
            double_param_.value = speed_;
            conf_.doubles.clear();
            conf_.doubles.push_back(double_param_);
            reconfig_.request.config = conf_;
            setParametersClient_.call(reconfig_);
        }
    }

}
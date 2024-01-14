#include "../include/sideways_inter.h"
#include "../../inter_util/include/inter_util.h"

#include <thread>
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

PLUGINLIB_EXPORT_CLASS(sideways_inter::SidewaysInter, mbf_costmap_core::CostmapInter)

namespace sideways_inter
{

    uint32_t SidewaysInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
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

            // Check if the pedestrian is in range to set temporary goal and move aside
            if ((!new_goal_set_) && (distance <= ped_minimum_distance_) && (2 * std::abs(angle_diff) <= fov_))
            {
                ROS_INFO("Pedestrian detected. Distance: %lf", distance);
                ROS_INFO("Setting new temp_goal");
                temp_goal_ = start;

                // calculating position for temporary goal
                temp_goal_.pose.position.x -= 2*temp_goal_distance_ * cos(theta + M_PI / 2.0);
                temp_goal_.pose.position.y -= 2*temp_goal_distance_ * sin(theta + M_PI / 2.0);
                temp_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(temp_goal_.pose.orientation));
                temp_goal_.header.frame_id = start.header.frame_id;
                new_goal_set_ = true;

            }

            // nothing else to compute
            if(caution && new_goal_set_)
                break;

        }

        speed_ = caution ? changed_max_vel_x_param_ : max_vel_x_param_;

        if (new_goal_set_)
        {
            //calculate distance to temporary goal
            double distance_to_temp_goal_ = std::sqrt(std::pow(temp_goal_.pose.position.x - robot_x, 2) + std::pow(temp_goal_.pose.position.y - robot_y, 2));

            // Clear the existing plan and add temp_goal
            plan.clear();
            plan.push_back(temp_goal_);

            if (distance_to_temp_goal_ <= temp_goal_tolerance_)
            {
            // Set speed to 0.0 when reaching temp_goal
            ROS_INFO("Reached temp_goal. Resetting goal and setting speed to 0.0 for 5 seconds.");

            
            speed_ = 0.0;

            // Wait for 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Reset speed to the previous value (last_speed_)
            speed_ = last_speed_;

            ROS_INFO("Resumed with the previous speed.");
            new_goal_set_ = false;
            }
            
        }
        else
            plan = plan_;

        return 0;
    }

    bool SidewaysInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = plan;
        return true;
    }

    void SidewaysInter::semanticCallback(const pedsim_msgs::SemanticData::ConstPtr &message)
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

    void SidewaysInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string node_namespace_ = ros::this_node::getNamespace();
        std::string semantic_layer = "/pedsim_agents/semantic/pedestrian";
        nh_ = ros::NodeHandle("~");
        subscriber_ = nh_.subscribe(semantic_layer, 1, &SidewaysInter::semanticCallback, this);
        // get our local planner name
        std::string planner_keyword;
        if (!nh_.getParam(node_namespace_+"/local_planner", planner_keyword)){
            ROS_ERROR("Failed to get parameter %s/local_planner", node_namespace_.c_str());
        }
        std::string local_planner_name = inter_util::InterUtil::getLocalPlanner(planner_keyword);
        // get the starting parameter for max_vel_x from our planner
        if (!nh_.getParam(node_namespace_+"/move_base_flex/"+ local_planner_name +"/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter %s/move_base_flex/%s/max_vel_x", node_namespace_.c_str(), local_planner_name.c_str());
            return;
        }
        // Create service client for the Reconfigure service
        setParametersClient_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(node_namespace_+"/move_base_flex/"+ local_planner_name+"/set_parameters");
        dynamic_reconfigure::Server<sideways_inter::sidewaysInterConfig> server;
        server.setCallback(boost::bind(&SidewaysInter::reconfigure, this, _1, _2));

        velocity_thread_ = std::thread(&SidewaysInter::setMaxVelocityThread, this);

        // needs to be declared here because cautious_speed gets declared with reconfigure
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);
    }

    void SidewaysInter::reconfigure(sideways_inter::sidewaysInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);

        //updating values from config
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        caution_detection_range_ = config.caution_detection_range;
        cautious_speed_ = config.cautious_speed;
        temp_goal_tolerance_ = config.temp_goal_tolerance;
        fov_ = config.fov;
        changed_max_vel_x_param_ = (cautious_speed_ * max_vel_x_param_);
    }

    void SidewaysInter::setMaxVelocityThread()
    {
        ros::Rate rate(1); // Adjust the rate as needed
        while (ros::ok())
        {
            // Lock to access shared variables
            boost::unique_lock<boost::mutex> lock(speed_mtx_);

            // Check if the speed has changed
            if (speed_ != last_speed_)
            {
                // set max_vel_x parameter
                double_param_.name = "max_vel_x";
                double_param_.value = speed_;
                conf_.doubles.clear();
                conf_.doubles.push_back(double_param_);
                reconfig_.request.config = conf_;

                // Call setParametersClient_ to update parameters
                if (setParametersClient_.call(reconfig_))
                {
                    ROS_INFO_ONCE("Dynamic reconfigure request successful");
                }
                else
                {
                    ROS_ERROR_ONCE("Failed to call dynamic reconfigure service");
                }

                // Update last_speed_ to avoid unnecessary calls
                last_speed_ = speed_;
            }

            if (speed_ == 0.0)
            {
                ROS_INFO("Speed is 0.0. Waiting for 5 seconds...");
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::seconds(5));
                ROS_INFO("Resuming with the previous speed.");
                lock.lock();

                // After waiting, set speed_ back to the previous value
                speed_ = last_speed_;
            }

            // Unlock and sleep
            lock.unlock();
            rate.sleep();
        }
    }

}
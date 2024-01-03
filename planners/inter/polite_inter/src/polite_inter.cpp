#include "../include/polite_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include<costmap_2d/costmap_math.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <costmap_2d/footprint.h>
#include <costmap_2d/array_parser.h>
#include<geometry_msgs/Point32.h>
#include <string>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{

    ros::ServiceClient get_dump_client_;
    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;
    geometry_msgs::PoseStamped temp_goal;
    bool new_goal_set_ = false;
    double max_vel_x_param_;
    double changed_max_vel_x_param_;
    

    uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        if (!nh_.getParam("/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter '/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x'");
            return 0;
        }
        ROS_ERROR("TEST, %f", max_vel_x_param_);
        // Create a request message
        //costmap_2d::GetDump::Request request;
        // No need to set any specific fields in the request for this example

        // Create a response message
        //costmap_2d::GetDump::Response response;
        costmap_2d::GetDump srv;
        // Lock the mutex for plan_
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        
        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;
        //ROS_ERROR("Original Goal at the Start: x: %f, y: %f, z: %f, orientation: %f",
        //            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, tf::getYaw(goal.pose.orientation));
        // Call the GetDump service
        if (get_dump_client_.call(srv))
        {
            //ROS_ERROR("GetDump service call successful");

            // Access the semantic layers from the response
            auto semantic_layers = srv.response.semantic_layers;

            // Process the semantic layers
            for (const auto &semantic_layer : semantic_layers)
            {
                //ROS_ERROR("Semantic Layer Names:");

                // Iterate through the layers
                for (const auto &layer : semantic_layer.layers)
                {
                    //ROS_ERROR("Layer Name: %s", layer.type.c_str());
                    // Iterate through the points in each layer
                    if (layer.type == "pedestrian")
                    {
                        for (const auto &point : layer.points)
                        {
                            double distance = std::sqrt(std::pow(point.location.x - robot_x, 2) + std::pow(point.location.y - robot_y, 2))+ std::pow(point.location.z - robot_z, 2);
                            //ROS_ERROR("Location: x: %f, y: %f, z: %f, Distance: %f", point.location.x, point.location.y, point.location.z, distance);
                            //initialize speed to be normal again
                            // Check if the pedestrian is in detection range to slow down
                            //nh_.setParam("/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x", max_vel_x_param_);
                            if (distance <= caution_detection_range_)
                            {
                                nh_.setParam("/jackal/move_base_flex/TebLocalPlannerROS/weight_max_vel_x", changed_max_vel_x_param_);
                                nh_.setParam("/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x", changed_max_vel_x_param_);
                            }
                            //ROS_ERROR("Current velocity: linear_x = %f, linear_y = %f, linear_z = %f",new_velocity.linear.x, new_velocity.linear.y, new_velocity.linear.z);
                            // Check if the pedestrian is in range to set temp goal and move back
                            if ((distance <= ped_minimum_distance_) && !new_goal_set_)
                            {
                                ROS_INFO("Pedestrian detected. Distance: %f", distance);
                                if(!new_goal_set_){
                                    ROS_INFO("Setting new temp_goal");
                                    temp_goal = start;
                                    double theta = tf::getYaw(temp_goal.pose.orientation);
                                    temp_goal.pose.position.x -= temp_goal_distance_ * cos(theta);
                                    temp_goal.pose.position.y -= temp_goal_distance_ * sin(theta);
                                    temp_goal.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(temp_goal.pose.orientation));
                                    temp_goal.header.frame_id = start.header.frame_id;
                                    //ROS_ERROR("Position: x = %f, y = %f, z = %f", temp_goal.pose.position.x, temp_goal.pose.position.y, temp_goal.pose.position.z);
                                    new_goal_set_ = true;
                                }
                            }
                        }
                    }
                }
            }
            if (new_goal_set_)
            {
                double distance_to_temp_goal = std::sqrt(std::pow(temp_goal.pose.position.x - robot_x, 2) + std::pow(temp_goal.pose.position.y - robot_y, 2));
                
                if (distance_to_temp_goal <= temp_goal_tolerance_)
                {
                    ROS_INFO("Reached temp_goal. Resetting goal.");
                    new_goal_set_ = false;
                }
                // Clear the existing plan and add temp_goal
                plan.clear();
                plan.push_back(temp_goal);
                return 0;
            }
            plan.insert(plan.end(), plan_.begin(), plan_.end());
            return 0;
        }
        else
        {
            ROS_ERROR("Failed to call GetDump service");
            return polite_inter::INTERNAL_ERROR;
        }
    }

    bool PoliteInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = plan;
        return true;
    }

    void PoliteInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string node_namespace = ros::this_node::getNamespace();

        nh_ = ros::NodeHandle("~");

        // Create a service client for the GetDump service
        if (!nh_.getParam("/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x", max_vel_x_param_))
        {
            ROS_ERROR("Failed to get parameter '/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x'");
            return;
        }
        changed_max_vel_x_param_ = (cautious_speed_/max_vel_x_param_);
        get_dump_client_ = nh_.serviceClient<costmap_2d::GetDump>("global_costmap/get_dump");  
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/jackal/move_base_flex/TebLocalPlannerROS/max_vel_x", 1);
        dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
        server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));
    }

    void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        caution_detection_range_ = config.caution_detection_range;
        cautious_speed_ = config.cautious_speed;
        temp_goal_tolerance_ = config.temp_goal_tolerance;
    }
}
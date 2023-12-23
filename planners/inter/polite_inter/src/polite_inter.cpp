#include "../include/polite_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

PLUGINLIB_EXPORT_CLASS(polite_inter::PoliteInter, mbf_costmap_core::CostmapInter)

namespace polite_inter
{

    ros::ServiceClient get_dump_client_;
    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;
    geometry_msgs::PoseStamped temp_goal;
    bool new_goal_set_ = false;
    

    uint32_t PoliteInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
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
                            double distance = std::sqrt(std::pow(point.location.x - robot_x, 2) + std::pow(point.location.y - robot_y, 2));
                            //ROS_ERROR("Location: x: %f, y: %f, z: %f, Distance: %f", point.location.x, point.location.y, point.location.z, distance);
                            // Check if the pedestrian is 2 meters or nearer (adjust to desired distance)
                            if ((distance <= 2.0) && !new_goal_set_)
                            {
                                ROS_ERROR("Condition Satisfied. Distance: %f", distance);
                                if(!new_goal_set_){
                                    ROS_ERROR("Setting new goal");
                                    temp_goal = start;
                                    double theta = tf::getYaw(temp_goal.pose.orientation);
                                    temp_goal.pose.position.x -= 2.0 * cos(theta);
                                    temp_goal.pose.position.y -= 2.0 * sin(theta);
                                    temp_goal.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(temp_goal.pose.orientation));
                                    temp_goal.header.frame_id = start.header.frame_id;
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
                
                if (distance_to_temp_goal <= 0.2) // Adjust the threshold as needed
                {
                    ROS_ERROR("Reached temp_goal. Resetting goal.");
                    new_goal_set_ = false;
                }
                //ROS_ERROR("Setting new goal");
                //ROS_ERROR("Position: x = %f, y = %f, z = %f", temp_goal.pose.position.x, temp_goal.pose.position.y, temp_goal.pose.position.z);

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

        nh_ = ros::NodeHandle("~");

        // Create a service client for the GetDump service
        get_dump_client_ = nh_.serviceClient<costmap_2d::GetDump>("global_costmap/get_dump");
    
        dynamic_reconfigure::Server<polite_inter::PoliteInterConfig> server;
        server.setCallback(boost::bind(&PoliteInter::reconfigure, this, _1, _2));
    }

    void PoliteInter::reconfigure(polite_inter::PoliteInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
        min_poses_ = config.min_poses;
    }
}
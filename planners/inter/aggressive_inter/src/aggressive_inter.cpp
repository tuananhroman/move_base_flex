#include "../include/aggressive_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
PLUGINLIB_EXPORT_CLASS(aggressive_inter::AggressiveInter, mbf_costmap_core::CostmapInter)

namespace aggressive_inter
{

    ros::ServiceClient get_dump_client_;
    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;
    geometry_msgs::PoseStamped temp_goal;
    bool new_goal_set_ = false;
    geometry_msgs::Twist current_cmd_vel_;
    uint32_t AggressiveInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        costmap_2d::GetDump srv;
        // Lock the mutex for plan_
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        geometry_msgs::Twist new_velocity;

        double robot_x = start.pose.position.x;
        double robot_y = start.pose.position.y;
        double robot_z = start.pose.position.z;
        double linear_x = current_cmd_vel_.linear.x;
        double linear_y = current_cmd_vel_.linear.y;
        //ROS_ERROR("Original Goal at the Start: x: %f, y: %f, z: %f, orientation: %f",
        //goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, tf::getYaw(goal.pose.orientation));

        ROS_ERROR("speed: %f", linear_x);

        // Call the GetDump service
        if (get_dump_client_.call(srv))
        {
            // Access the semantic layers from the response
            auto semantic_layers = srv.response.semantic_layers;

            // Process the semantic layers
            for (const auto &semantic_layer : semantic_layers)
            {
                // Iterate through the layers
                for (const auto &layer : semantic_layer.layers)
                {
                    // Iterate through the points in each layer
                    if (layer.type == "pedestrian")
                    {
                        double minDistance = 999999; //sufficiently high number

                        for (const auto &point : layer.points)
                        {
                            double distance = std::sqrt(std::pow(point.location.x - robot_x, 2) + std::pow(point.location.y - robot_y, 2))+ std::pow(point.location.z - robot_z, 2);

                            // Check if the pedestrian is in detection range to slow down
                            if (distance <= 5)
                            {
                                geometry_msgs::Twist new_velocity;
                                minDistance = std::min(distance, minDistance);
                                double speed = max_speed_ - (max_speed_ / (1 + std::pow(distance, 2)));
                                setMaxVelocity(speed);
                            }
                            else
                            {
                                setMaxVelocity(max_speed_);
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
            return aggressive_inter::INTERNAL_ERROR;
        }
    }

    bool AggressiveInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = plan;
        return true;
    }

    void AggressiveInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;
        std::string node_namespace = ros::this_node::getNamespace();
        std::string cmd_vel_topic = node_namespace + "/cmd_vel";

        nh_ = ros::NodeHandle("~");

        // Create a service client for the GetDump service
        get_dump_client_ = nh_.serviceClient<costmap_2d::GetDump>("global_costmap/get_dump");
        vel_sub_ = nh_.subscribe(cmd_vel_topic, 1, &AggressiveInter::cmdVelCallback, this);
        dynamic_reconfigure::Server<aggressive_inter::AggressiveInterConfig> server;
        server.setCallback(boost::bind(&AggressiveInter::reconfigure, this, _1, _2));
    }

    void AggressiveInter::setMaxVelocity(double new_max_vel_x)
    {
        // Create a dynamic reconfigure service client
        ros::ServiceClient client = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/jackal/move_base_flex/TebLocalPlannerROS/set_parameters");

        // Create a dynamic reconfigure request
        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double_param.name = "max_vel_x";
        double_param.value = new_max_vel_x;
        conf.doubles.push_back(double_param);
        srv.request.config = conf;

        // Call the dynamic reconfigure service
        if (client.call(srv))
        {
            //ROS_INFO("Dynamic reconfigure request successful");
        }
        else
        {
            //ROS_ERROR("Failed to call dynamic reconfigure service");
        }
    }

    void AggressiveInter::cmdVelCallback(const geometry_msgs::Twist& msg)
    {
        current_cmd_vel_ = msg;
    }
    void AggressiveInter::reconfigure(aggressive_inter::AggressiveInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
        ped_minimum_distance_ = config.ped_minimum_distance;
        temp_goal_distance_ = config.temp_goal_distance;
        caution_detection_range_ = config.caution_detection_range;
        max_speed_ = config.max_speed;
        temp_goal_tolerance_ = config.temp_goal_tolerance;
    }
}
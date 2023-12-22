#include "../include/sideways_inter.h"
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/GetDump.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sideways_inter::sidewaysInter, mbf_costmap_core::CostmapInter)


namespace sideways_inter
{

    const uint32_t SUCCESS = 0;
    const uint32_t INTERNAL_ERROR = 1;

    uint32_t sidewaysInter::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan, double &cost, std::string &message)
    {
        // Call the GetDump service to obtain semantic layers
        costmap_2d::GetDump getDump_srv;

        // Call the GetDump service
        
        if (get_dump_client_.call(getDump_srv))
        {
            ROS_INFO("We called the service");

            // Access the semantic layers from the response
            auto semantic_layers = getDump_srv.response.semantic_layers;

            // Process the semantic layers
            for (const auto &semantic_layer : semantic_layers)
            {
                ROS_INFO("Semantic Layer Names:");

                // Iterate through the layers
                for (const auto &layer : semantic_layer.layers)
                {
                    ROS_INFO("Layer Name: %s", layer.type.c_str());




                    if (strcmp(layer.type.c_str(), "pedestrian") == 0)
                    {
                     // Iterate through the points in each layer
                    for (const auto &point : layer.points)
                    {
                                                                                                                                         //debug to the position of pedestrians :ROS_INFO("Location: x: %f, y: %f, z: %f", point.location.x, point.location.y, point.location.z);

                    //TODO: When the robot sees a pedestrian he should stop
                    // Calculate the distance between the robot and the pedestrian
                    double distance = std::sqrt(std::pow(start.pose.position.x - point.location.x, 2) +
                                                std::pow(start.pose.position.y - point.location.y, 2));

                    ROS_ERROR("Distance to pedestrian: %f meters", distance);                                                            // debug to see position and goal: ROS_ERROR("Current Position: %.2f, %.2f, Goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
                    

                    if (distance <= 4.0){
                    ROS_ERROR("Pedestrian too close! Stopping and moving backward.");                     
                    }                       
                  
                }
                }
                }
                // Check if the layer is of type "pedestrian" and adjust the plan accordingly

            }

            
            //TODO: Use semantic_layers data to modify the plan as needed

            // Lock the mutexes for plan_ and vision_cfg_mtx_
            boost::unique_lock<boost::mutex> lock2(plan_mtx_);
            boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);

            // Modify the plan based on the semantic_layers data
            size_t limit = std::floor(plan_.size() * vision_limit_);
            limit = std::max(limit, min_poses_);
            limit = std::min(limit, plan_.size());

            plan = std::vector<geometry_msgs::PoseStamped>(plan_.begin(), plan_.begin() + limit);

            // Unlock the mutexes
            lock.unlock();
            lock2.unlock();

            // Return the result code
            return 0;
        }
        
        else
        {
            ROS_ERROR_STREAM("Failed to call GetDump service. Call result: " << get_dump_client_.call(getDump_srv));
          
            // Handle the error and return an appropriate result code
            return sideways_inter::INTERNAL_ERROR;
        }
    }

    bool sidewaysInter::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<boost::mutex> lock(plan_mtx_);
        plan_ = plan;
        return true;
    }

    void sidewaysInter::initialize(std::string name, costmap_2d::Costmap2DROS *global_costmap_ros, costmap_2d::Costmap2DROS *local_costmap_ros)
    {
        this->name = name;

        // Create a service client for the GetDump service
        get_dump_client_ = ros::NodeHandle("~").serviceClient<costmap_2d::GetDump>("global_costmap/get_dump");
     
        dynamic_reconfigure::Server<sideways_inter::sidewaysInterConfig> server;
        server.setCallback(boost::bind(&sidewaysInter::reconfigure, this, _1, _2));
    }

    void sidewaysInter::reconfigure(sideways_inter::sidewaysInterConfig &config, uint32_t level)
    {
        boost::unique_lock<boost::mutex> lock(vision_cfg_mtx_);
        vision_limit_ = config.vision_limit;
        min_poses_ = config.min_poses;
    }

}
